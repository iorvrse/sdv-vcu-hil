#define _GNU_SOURCE
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdatomic.h>
#include <math.h>
#include <signal.h>
#include <errno.h>

// ==========================================
// KONFIGURASI IP & STATE
// ==========================================
const char *IP_CORNERS[CORNER_COUNT] =
{
    "10.252.62.51", // FL
    "10.252.62.52", // FR
    "10.252.62.53", // RL
    "10.252.62.54"  // RR
};

const int PORT_CORNERS[CORNER_COUNT] = {5051, 5052, 5053, 5054};
const int CornerHeaders[CORNER_COUNT] = {0xBA, 0xBB, 0xBC, 0xBD};

const char *IP_LAPTOP = "10.252.62.212";

udp_sock_t rx_matlab;
udp_sock_t tx_corner[CORNER_COUNT];
udp_sock_t tx_matlab;

static volatile int keep_running = 1;
static atomic_uint_fast8_t steer_idx = 0;

typedef struct
{
    uint8_t brake;
    uint16_t accel;
    int16_t steer;
    struct timespec ts;
} steer_sample_t;

static steer_sample_t steer_buf[2];

// ==========================================
// INISIALISASI VARIABEL MATLAB (SIMULINK)
// ==========================================
static RT_MODEL_Torque_Vectoring_T Torque_Vectoring_M_;
static RT_MODEL_Torque_Vectoring_T *const Torque_Vectoring_MPtr = &Torque_Vectoring_M_;
static DW_Torque_Vectoring_T Torque_Vectoring_DW;
static B_Torque_Vectoring_T Torque_Vectoring_B;

static real_T TV_U_Vx_des;
static real_T TV_U_Vx;
static real_T TV_U_Vy;
static real_T TV_U_AngWheel[4];
static real_T TV_U_r;
static real_T TV_U_Fy[4];
static real_T TV_U_Fz[4];

static real_T TV_Y_Tm[4];
static real_T TV_Y_Fx_opt[4];
static real_T TV_Y_Mx_total;
static real_T TV_Y_Fx_total;
static real_T TV_Y_Mzd;
static real_T TV_Y_r_des;
static real_T TV_Y_beta_des;
static real_T TV_Y_Ca[2];
static real_T TV_Y_beta;

// ==========================================
// FUNGSI UTILITAS WAKTU
// ==========================================
static void sleep_ms(long ms)
{
    struct timespec req = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&req, NULL);
}

void try_set_realtime(pthread_t thr, int prio)
{
    struct sched_param sp = {.sched_priority = prio};
    (void)pthread_setschedparam(thr, SCHED_FIFO, &sp);
}

void try_set_affinity(pthread_t thr, int cpu)
{
    cpu_set_t cp;
    CPU_ZERO(&cp);
    CPU_SET(cpu, &cp);
    (void)pthread_setaffinity_np(thr, sizeof(cp), &cp);
}

void sigint_handler(int dummy)
{
    keep_running = 0;
}

// =================================================================
// THREAD 1: PEMBACA SETIR FISIK (USB)
// =================================================================
void *thread_steer_reader(void *arg)
{
    (void)arg;
    steer_ctx_t *my_steer = steer_init();
    
    if (!my_steer || steer_start(my_steer) < 0)
    {
        fprintf(stderr, "[STEER] USB init failed!\n");
        return NULL;
    }

    try_set_affinity(pthread_self(), 0);
    try_set_realtime(pthread_self(), 40);

    steer_data_t moduleData;
    printf("[THREAD] USB Steer aktif.\n");

    while (keep_running)
    {
        int ret = steer_process_events(my_steer, 50); 
        
        if (ret == 0)
        {
            steer_get_latest_data(my_steer, &moduleData);

            uint_fast8_t next = 1 - atomic_load(&steer_idx);
            steer_buf[next].brake = moduleData.brake;
            steer_buf[next].accel = moduleData.accel;
            steer_buf[next].steer = moduleData.steer;
            clock_gettime(CLOCK_MONOTONIC_RAW, &steer_buf[next].ts);
            atomic_store(&steer_idx, next);
        }
    }

    steer_close(my_steer);
    return NULL;
}

// =================================================================
// THREAD 2: NETWORK RX & TORQUE VECTORING COMPUTE
// =================================================================
void *thread_network_rx(void *arg)
{
    (void)arg;
    try_set_affinity(pthread_self(), 1);
    try_set_realtime(pthread_self(), 80); 

    uint8_t rx_buf[1024];
    struct sockaddr_in src;
    ssize_t n;
    uint8_t seq_counter = 0;

    printf("[THREAD] Network RX & Torque Vectoring aktif.\n");

    while (keep_running)
    {
        // 1. TUNGGU DATA SENSOR SIMULASI DARI MATLAB
        n = udp_comm_recv(&rx_matlab, rx_buf, sizeof(rx_buf), &src, 0);
        
        if (n <= 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
            {
                sleep_ms(1);
                continue;
            }
            continue;
        }

        uint8_t incoming_header = rx_buf[0];

        // Jika Paket berasal dari MATLAB (Header 0xCE, ID 0x05)
        if (incoming_header == 0xCE && n == sizeof(Matlab2VCU))
        {
            Matlab2VCU *m2v = (Matlab2VCU*)rx_buf;
            if (m2v->id != 0x05) continue;

            // 2. PARSING DATA SENSOR MATLAB -> INPUT MODEL TV
            TV_U_Vx_des  = m2v->Vx_Thrl / 100.0f;
            TV_U_Vx      = m2v->Vx / 100.0f;
            TV_U_Vy      = m2v->Vy / 100.0f;
            
            // Ambil Input Setir Aktual dari USB (Timpa setir dari Matlab)
            int steer_idx_local = atomic_load(&steer_idx);
            float steer_actual = (float)steer_buf[steer_idx_local].steer / 100.0f; // -40 s.d 40 deg
            
            // Konversi derajat fisik ke radian untuk matriks TV
            for(int i=0; i<4; i++) TV_U_AngWheel[i] = steer_actual * (M_PI / 180.0f);

            TV_U_r = (m2v->yawrate / 1000.0f) - 0.7f;
            
            TV_U_Fy[0] = (m2v->Fy_FL / 10.0f) - 9000.0f;
            TV_U_Fy[1] = (m2v->Fy_FR / 10.0f) - 9000.0f;
            TV_U_Fy[2] = (m2v->Fy_RL / 10.0f) - 9000.0f;
            TV_U_Fy[3] = (m2v->Fy_RR / 10.0f) - 9000.0f;

            TV_U_Fz[0] = m2v->Fz_FL / 10.0f;
            TV_U_Fz[1] = m2v->Fz_FR / 10.0f;
            TV_U_Fz[2] = m2v->Fz_RL / 10.0f;
            TV_U_Fz[3] = m2v->Fz_RR / 10.0f;

            // 3. EKSEKUSI MODEL TORQUE VECTORING
            Torque_Vectoring_step(Torque_Vectoring_MPtr, TV_U_Vx_Thrl, TV_U_Vx, TV_U_Vy, 
                                  TV_U_AngWheel, TV_U_r, TV_U_Fy, TV_U_Fz,
                                  TV_Y_Tm, TV_Y_Fx_opt, &TV_Y_Mx_total, &TV_Y_Fx_total,
                                  &TV_Y_Mzd, &TV_Y_r_des, TV_Y_Ca, &TV_Y_beta);

            seq_counter++;

            // 4. KIRIM DATA KE RODA (STM32 CORNER)
            VCU2Corner_TV corner_frame;
            for (int i = 0; i < CORNER_COUNT; i++)
            {
                corner_frame.Header = CornerHeaders[i]; 
                corner_frame.id     = 0x02; // Biarkan 0x02 atau ikuti STM32
                
                // Scaling Torsi
                corner_frame.Trq = (uint16_t)((TV_Y_Tm[i] + 120.0f) * 100.0f);
                corner_frame.seq = seq_counter;

                udp_comm_send(&tx_corner[i], &tx_corner[i].addr, &corner_frame, sizeof(corner_frame), MSG_DONTWAIT);
            }

            // 5. BALAS DATA KE MATLAB (Opsional, untuk logging di PC)
            VCU2Matlab_TV v2m;
            v2m.Header = 0xAA;
            v2m.id     = 5;
            v2m.Trq_FL = (uint16_t)((TV_Y_Tm[0] + 120.0f) * 100.0f);
            v2m.Trq_FR = (uint16_t)((TV_Y_Tm[1] + 120.0f) * 100.0f);
            v2m.Trq_RL = (uint16_t)((TV_Y_Tm[2] + 120.0f) * 100.0f);
            v2m.Trq_RR = (uint16_t)((TV_Y_Tm[3] + 120.0f) * 100.0f);
            v2m.seq    = seq_counter;

            udp_comm_send(&tx_matlab, &tx_matlab.addr, &v2m, sizeof(v2m), MSG_DONTWAIT);
        }
    }
    
    return NULL;
}

// =================================================================
// MAIN FUNCTION
// =================================================================
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    signal(SIGINT, sigint_handler);

    // 1. Inisialisasi Model MATLAB Coder
    RT_MODEL_Torque_Vectoring_T *const Torque_Vectoring_M = Torque_Vectoring_MPtr;
    Torque_Vectoring_M->blockIO = &Torque_Vectoring_B;
    Torque_Vectoring_M->dwork = &Torque_Vectoring_DW;
    
    Torque_Vectoring_initialize(Torque_Vectoring_M, &TV_U_Vx_des, &TV_U_Vx, &TV_U_Vy, 
                                TV_U_AngWheel, &TV_U_r, TV_U_Fy, TV_U_Fz,
                                TV_Y_Tm, TV_Y_Fx_opt, &TV_Y_Mx_total, &TV_Y_Fx_total,
                                &TV_Y_Mzd, &TV_Y_r_des, &TV_Y_beta_des, TV_Y_Ca, &TV_Y_beta);

    printf("Inisialisasi UDP Jaringan...\n");

    // 2. Inisialisasi Soket RX (Mendengar MATLAB)
    if (udp_rx_init(&rx_matlab, PORT_RX) < 0)
    {
        fprintf(stderr, "Gagal membuka port RX VCU\n");
        return -1;
    }

    // 3. Inisialisasi Soket TX ke Roda (Unicast)
    for (int i = 0; i < CORNER_COUNT; i++)
    {
        // Setiap roda punya port sumber (src_port) sendiri dan port tujuan sendiri
        // PORT_SRC_CORNERS[i] digunakan sebagai ID lokal VCU, 
        // PORT_CORNERS[i] adalah port yang didengar oleh STM32 (5051, 5052, dst)
        
        if (udp_tx_init(&tx_corner[i], PORT_SRC_CORNERS[i], 0, IP_CORNERS[i]) < 0)
        {
            fprintf(stderr, "Gagal membuat soket TX untuk roda %d\n", i);
            return -1;
        }
        
        // Ini kuncinya: Menggunakan port yang berbeda untuk tiap roda
        udp_tx_set_target(&tx_corner[i], IP_CORNERS[i], PORT_CORNERS[i]);
    }

    // 4. Inisialisasi Soket TX ke MATLAB
    if (udp_tx_init(&tx_matlab, PORT_TX_PC, 0, IP_LAPTOP) < 0)
    {
        fprintf(stderr, "Gagal membuat TX ke Matlab\n");
        return -1;
    }
    udp_tx_set_target(&tx_matlab, IP_LAPTOP, PORT_LAPTOP);

    // 5. Menjalankan Thread
    pthread_t t_steer, t_vcu;

    if (pthread_create(&t_steer, NULL, thread_steer_reader, NULL) != 0)
    {
        perror("Gagal memulai thread setir");
        return -1;
    }
    
    if (pthread_create(&t_vcu, NULL, thread_network_rx, NULL) != 0)
    {
        perror("Gagal memulai thread master VCU");
        return -1;
    }

    printf("==========================================\n");
    printf(" VCU SDV RUNNING. Tekan Ctrl+C untuk keluar.\n");
    printf("==========================================\n");

    // 6. Cleanup
    pthread_join(t_vcu, NULL);
    pthread_join(t_steer, NULL);

    printf("\nMematikan sistem...\n");
    udp_comm_close(&rx_matlab.sockfd);
    udp_comm_close(&tx_matlab.sockfd);
    for (int i = 0; i < CORNER_COUNT; i++)
    {
        udp_comm_close(&tx_corner[i].sockfd);
    }
    
    return 0;
}
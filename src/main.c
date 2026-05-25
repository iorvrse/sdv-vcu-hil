#define _GNU_SOURCE
#include "main.h"
#include <pthread.h>
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <math.h>
#include <stdatomic.h>
#include <errno.h>

#include "comm.h"
#include "steer.h"
#include "FWIS.h"

// ==========================================
//  IP Config & state
// ==========================================
const char *corner_ip[CORNER_COUNT] = {
    "10.252.62.51", // FL
    "10.252.62.52", // FR
    "10.252.62.53", // RL
    "10.252.62.54"  // RR
};

const int corner_port[CORNER_COUNT] = {5051, 5052, 5053, 5054};
const int CORNER_HEADER[CORNER_COUNT] = {0xBA, 0xBB, 0xBC, 0xBD};

const char *matlab_ip = "10.252.62.212";

udp_sock_t rx_vcu;
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

fwis_t fwis;
torque_vectoring_t tv = {
    .Torque_Vectoring_MPtr = &tv.Torque_Vectoring_M_
};

static void sleep_ms(long ms)
{
    struct timespec req = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&req, NULL);
}

void sigint_handler(int dummy)
{
    keep_running = 0;
}

void Torque_Vectoring_Compute(matlab_recv_frame_t *m2v)
{
    tv.TV_U_Vx_des  = m2v->Vx_des / 100.0f;
    tv.TV_U_Vx      = m2v->Vx / 100.0f;
    tv.TV_U_Vy      = m2v->Vy / 100.0f;
    
    int steer_idx_local = atomic_load(&steer_idx);
    float steer_actual = (float)steer_buf[steer_idx_local].steer / 100.0f; // -40 to 40 deg
    
    for(int i=0; i<4; i++) tv.TV_U_AngWheel[i] = steer_actual * (M_PI / 180.0f);

    tv.TV_U_r = (m2v->yawRate / 1000.0f) - 0.7f;
    
    tv.TV_U_Fy[0] = (m2v->Fy_FL / 10.0f) - 9000.0f;
    tv.TV_U_Fy[1] = (m2v->Fy_FR / 10.0f) - 9000.0f;
    tv.TV_U_Fy[2] = (m2v->Fy_RL / 10.0f) - 9000.0f;
    tv.TV_U_Fy[3] = (m2v->Fy_RR / 10.0f) - 9000.0f;

    tv.TV_U_Fz[0] = m2v->Fz_FL / 10.0f;
    tv.TV_U_Fz[1] = m2v->Fz_FR / 10.0f;
    tv.TV_U_Fz[2] = m2v->Fz_RL / 10.0f;
    tv.TV_U_Fz[3] = m2v->Fz_RR / 10.0f;

    Torque_Vectoring_step(tv.Torque_Vectoring_MPtr, tv.TV_U_Vx_des, tv.TV_U_Vx, tv.TV_U_Vy,
        tv.TV_U_AngWheel, tv.TV_U_r, tv.TV_U_Fy, tv.TV_U_Fz,
        tv.TV_Y_Tm, tv.TV_Y_Fx_opt, &tv.TV_Y_Mx_total, &tv.TV_Y_Fx_total,
        &tv.TV_Y_Mzd, &tv.TV_Y_r_des, tv.TV_Y_Ca, &tv.TV_Y_beta);
}

// =================================================================
// Steer thread (USB)
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

    steer_data_t moduleData;
    printf("[THREAD] USB steer active.\n");

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
// Network thread
// =================================================================
void *thread_network_rx(void *arg)
{
    (void)arg;

    uint8_t rx_buf[1024];
    struct sockaddr_in src;
    ssize_t n;
    uint16_t seq = 0;

    printf("[THREAD] Network\n");

    while (keep_running)
    {
        n = udp_comm_recv(&rx_vcu, rx_buf, sizeof(rx_buf), &src, 0);
        
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

        if (incoming_header == MATLAB_FRAME_HEADER && n == sizeof(matlab_recv_frame_t))
        {
            matlab_recv_frame_t *m2v = (matlab_recv_frame_t*)rx_buf;
            if (m2v->id != MATLAB_FRAME_ID) continue;

            FWIS_Compute(&fwis, steer_buf[steer_idx].steer / 100.0f, m2v->Vx / 100.0f);
            Torque_Vectoring_Compute(m2v);

            seq++;
            corner_send_frame_t corner_frame = {
                .header = CORNER_FRAME_HEADER,
                .id = CORNER_FRAME_ID
            };

            for (int i = 0; i < CORNER_COUNT; i++)
            {
                corner_frame.Tm_ref = (uint16_t)((tv.TV_Y_Tm[i] + 120.0f) * 100.0f);
                corner_frame.Ang_ref = (uint16_t)(fwis.output[i]);
                corner_frame.seq = seq;
                udp_comm_send(&tx_corner[i], &tx_corner[i].addr, &corner_frame, sizeof(corner_frame), MSG_DONTWAIT);
            }
        }
    }
    
    return NULL;
}

// =================================================================
// Main Function
// =================================================================
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    signal(SIGINT, sigint_handler);

    printf("Initialize Vehicle Dynamics Model...\n");

    // 4WIS
    FWIS_Init(&fwis, FWIS_WB, FWIS_CG, FWIS_WT, FWIS_HWB, FWIS_HWT);

    // Torque vectoring
    RT_MODEL_Torque_Vectoring_T *const Torque_Vectoring_M = tv.Torque_Vectoring_MPtr;
    Torque_Vectoring_M->blockIO = &tv.Torque_Vectoring_B;
    Torque_Vectoring_M->dwork = &tv.Torque_Vectoring_DW;
    
    Torque_Vectoring_initialize(Torque_Vectoring_M, &tv.TV_U_Vx_des, &tv.TV_U_Vx, &tv.TV_U_Vy, 
        tv.TV_U_AngWheel, &tv.TV_U_r, tv.TV_U_Fy, tv.TV_U_Fz,
        tv.TV_Y_Tm, tv.TV_Y_Fx_opt, &tv.TV_Y_Mx_total, &tv.TV_Y_Fx_total,
        &tv.TV_Y_Mzd, &tv.TV_Y_r_des, &tv.TV_Y_beta_des, tv.TV_Y_Ca, &tv.TV_Y_beta);

    printf("Initialize UDP network...\n");

    // Initialize VCU socket
    if (udp_rx_init(&rx_vcu, VCU_PORT) < 0)
    {
        fprintf(stderr, "Failed open port VCU\n");
        return -1;
    }

    // Initialize socket to corner
    for (int i = 0; i < CORNER_COUNT; i++)
    {
        if (udp_tx_init(&tx_corner[i], corner_port[i], 0, corner_ip[i]) < 0)
        {
            fprintf(stderr, "Failed to create TX socket for corner[%d]\n", i);
            return -1;
        }
        udp_tx_set_target(&tx_corner[i], corner_ip[i], corner_port[i]);
    }

    // Initialize socket to matlab
    if (udp_tx_init(&tx_matlab, MATLAB_PORT, 0, matlab_ip) < 0)
    {
        fprintf(stderr, "Failed to create TX socket for Matlab\n");
        return -1;
    }
    udp_tx_set_target(&tx_matlab, matlab_ip, MATLAB_PORT);

    // Run threads
    pthread_t t_steer, t_vcu;
    struct sched_param param;
    pthread_attr_t attr;

    /* Lock memory */
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        printf("mlockall failed: %m\n");
        exit(-2);
    }

    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, PTHREAD_STACK_MIN);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    
    param.sched_priority = 85;
    pthread_attr_setschedparam(&attr, &param);
    if (pthread_create(&t_steer, NULL, thread_steer_reader, NULL) != 0)
    {
        perror("Failed to create steer thread");
        return -1;
    }
    
    param.sched_priority = 90;
    pthread_attr_setschedparam(&attr, &param);
    if (pthread_create(&t_vcu, NULL, thread_network_rx, NULL) != 0)
    {
        perror("Failed to create network thread");
        return -1;
    }

    pthread_attr_destroy(&attr);

    printf("==========================================\n");
    printf(" VCU SDV RUNNING. Press Ctrl+C to leave\n");
    printf("==========================================\n");

    // Cleanup
    pthread_join(t_vcu, NULL);
    pthread_join(t_steer, NULL);

    printf("\nShutting down the system...\n");
    udp_comm_close(&rx_vcu.sockfd);
    udp_comm_close(&tx_matlab.sockfd);
    for (int i = 0; i < CORNER_COUNT; i++)
    {
        udp_comm_close(&tx_corner[i].sockfd);
    }
    
    return 0;
}
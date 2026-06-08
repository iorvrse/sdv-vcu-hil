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
#include <time.h>

#include "comm.h"
#include "steer.h"
#include "FWIS.h"

// ==========================================
//  Debug logging control
// ==========================================
#define DEBUG_ENABLE  1
#if DEBUG_ENABLE
    #define DEBUG_LOG(fmt, ...)   printf("[DBG] " fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG(fmt, ...)   ((void)0)
#endif

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
const char *matlab_ip = "10.252.62.212";

udp_sock_t rx_vcu;
udp_sock_t tx_corner[CORNER_COUNT];
udp_sock_t tx_matlab;

static volatile int keep_running = 1;

// Atomic Counters to monitor RX frequency (Hz)
static atomic_uint_fast32_t rx_matlab_count = 0;
static atomic_uint_fast32_t rx_steer_count  = 0;

static atomic_uint_fast8_t steer_idx = 0;
typedef struct {
    uint8_t brake;
    uint16_t accel;
    int16_t steer;
    struct timespec ts;
} steer_sample_t;
static steer_sample_t steer_buf[2];

static matlab_recv_frame_t matlab_buf[2];
static atomic_uint_fast8_t matlab_idx = 0;

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

// Torque Vectoring
void Torque_Vectoring_Compute(const matlab_recv_frame_t *m2v, float *vx_des)
{
    tv.TV_U_Vx_des  = *vx_des;
    tv.TV_U_Vx      = m2v->Vx / 100.0f;
    tv.TV_U_Vy      = (m2v->Vy / 100.0f) - 10.0f;

    tv.TV_U_AngWheel[0] = (m2v->angWheel[0] / 1000.0f) - 0.7f;
    tv.TV_U_AngWheel[1] = (m2v->angWheel[1] / 1000.0f) - 0.7f;
    tv.TV_U_AngWheel[2] = (m2v->angWheel[2] / 1000.0f) - 0.7f;
    tv.TV_U_AngWheel[3] = (m2v->angWheel[3] / 1000.0f) - 0.7f;

    tv.TV_U_r = (m2v->yawRate / 1000.0f) - 0.7f;

    tv.TV_U_Fy[0] = (m2v->Fy[0] / 10.0f) - 9000.0f;
    tv.TV_U_Fy[1] = (m2v->Fy[1] / 10.0f) - 9000.0f;
    tv.TV_U_Fy[2] = (m2v->Fy[2] / 10.0f) - 9000.0f;
    tv.TV_U_Fy[3] = (m2v->Fy[3] / 10.0f) - 9000.0f;

    tv.TV_U_Fz[0] = m2v->Fz[0] / 10.0f;
    tv.TV_U_Fz[1] = m2v->Fz[1] / 10.0f;
    tv.TV_U_Fz[2] = m2v->Fz[2] / 10.0f;
    tv.TV_U_Fz[3] = m2v->Fz[3] / 10.0f;

    Torque_Vectoring_step(tv.Torque_Vectoring_MPtr,
        tv.TV_U_Vx_des, tv.TV_U_Vx, tv.TV_U_Vy,
        tv.TV_U_AngWheel, tv.TV_U_r,
        tv.TV_U_Fy, tv.TV_U_Fz,
        tv.TV_Y_Tm, tv.TV_Y_Fx_opt,
        &tv.TV_Y_Mx_total, &tv.TV_Y_Fx_total,
        &tv.TV_Y_Mzd, &tv.TV_Y_r_des,
        tv.TV_Y_Ca, &tv.TV_Y_beta);
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
    
    // Print disabled to prevent console conflict with the Dashboard
    // printf("[THREAD] USB steer active.\n");

    while (keep_running)
    {
        int ret = steer_process_events(my_steer, 50);
        if (ret == 0)
        {
            steer_get_latest_data(my_steer, &moduleData);

            uint_fast8_t next = 1 - atomic_load_explicit(&steer_idx, memory_order_acquire);
            steer_buf[next].brake = moduleData.brake;
            steer_buf[next].accel = moduleData.accel;
            steer_buf[next].steer = moduleData.steer;
            clock_gettime(CLOCK_MONOTONIC_RAW, &steer_buf[next].ts);
            atomic_store_explicit(&steer_idx, next, memory_order_release);
            
            // Increment RX Counter
            atomic_fetch_add_explicit(&rx_steer_count, 1, memory_order_relaxed);
        }
    }

    steer_close(my_steer);
    return NULL;
}

// =================================================================
// Network receiver thread
// =================================================================
void *thread_net_rx(void *arg)
{
    (void)arg;

    uint8_t rx_buf[1024];
    struct sockaddr_in src;
    ssize_t n;

    // Print disabled to prevent console conflict with the Dashboard
    // printf("[THREAD] Network RX active.\n");

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

        if (n == sizeof(matlab_recv_frame_t) && rx_buf[0] == MATLAB_FRAME_HEADER)
        {
            matlab_recv_frame_t *m2v = (matlab_recv_frame_t*)rx_buf;
            if (m2v->id != MATLAB_FRAME_ID) continue;

            // Write to inactive MATLAB buffer
            uint_fast8_t next = 1 - atomic_load_explicit(&matlab_idx, memory_order_acquire);
            memcpy(&matlab_buf[next], m2v, sizeof(matlab_recv_frame_t));
            atomic_store_explicit(&matlab_idx, next, memory_order_release);
            
            // Increment RX Counter
            atomic_fetch_add_explicit(&rx_matlab_count, 1, memory_order_relaxed);
        }
    }

    return NULL;
}

// =================================================================
// Control thread
// =================================================================
void *thread_control(void *arg)
{
    (void)arg;

    struct timespec thread_ts;
    clock_gettime(CLOCK_MONOTONIC, &thread_ts);
    uint8_t seq = 0;
    uint32_t cycle = 0;
    const uint32_t debug_interval = 100;   // print every 100 cycles (100 ms)
    struct timespec cycle_start;

    // Buffers to capture transmit (TX) status
    ssize_t tx_matlab_status = 0;
    int     tx_matlab_err    = 0;
    
    ssize_t tx_corner_status[CORNER_COUNT] = {0};
    int     tx_corner_err[CORNER_COUNT]    = {0};

    // Buffers to calculate receive speed (RX Rate)
    uint32_t last_rx_matlab = 0;
    uint32_t last_rx_steer  = 0;

    // Clear the terminal screen on startup
    printf("\033[2J"); 

    while (keep_running)
    {
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);  // for jitter measurement

        // Latest MATLAB
        int matlab_idx_local = atomic_load_explicit(&matlab_idx, memory_order_acquire);
        matlab_recv_frame_t cur_matlab = matlab_buf[matlab_idx_local];

        // Latest steer
        int steer_idx_local = atomic_load_explicit(&steer_idx, memory_order_acquire);
        steer_sample_t cur_steer = steer_buf[steer_idx_local];
        float steer_deg = (float)cur_steer.steer / 100.0f;
        float vx_des = (float)cur_steer.accel / 100.0f;
        float vx_steer = cur_matlab.Vx / 100.0f;

        // Compute
        FWIS_Compute(&fwis, &steer_deg, &vx_steer);
        Torque_Vectoring_Compute(&cur_matlab, &vx_des);

        // Increment Sequence
        seq++;

        // 1. SEND BRAKE TO MATLAB
        vcu_brake_matlab_frame_t brake_frame = {
            .header = MATLAB_BRAKE_HEADER,
            .id     = MATLAB_BRAKE_ID,
            .brake  = cur_steer.brake,
            .seq    = seq
        };
        tx_matlab_status = udp_comm_send(&tx_matlab, &tx_matlab.addr, &brake_frame, sizeof(brake_frame), MSG_DONTWAIT);
        if (tx_matlab_status < 0) tx_matlab_err = errno; 

        // 2. SEND TORQUE TO CORNERS
        corner_send_frame_t corner_frame = {
            .header = CORNER_FRAME_HEADER,
            .Vx = (uint16_t)(cur_matlab.Vx * 100),
            .Vx_des = cur_steer.accel,
            .Mzd = (uint32_t)(tv.TV_Y_Mzd + 40000)
        };

        for (int i = 0; i < CORNER_COUNT; i++)
        {
            corner_frame.id = i + 1; // ID starts from 1 for corners
            corner_frame.Tm_ref  = (uint16_t)((tv.TV_Y_Tm[i] + 120.0f) * 100.0f);
            corner_frame.Ang_ref = (uint16_t)((fwis.output[i] + 40.0f) * 100.0f);
            corner_frame.Vx_wheel = cur_matlab.Vx_wheel[i];
            corner_frame.seq = seq;
            
            tx_corner_status[i] = udp_comm_send(&tx_corner[i], &tx_corner[i].addr, &corner_frame, sizeof(corner_frame), MSG_DONTWAIT);
            if (tx_corner_status[i] < 0) tx_corner_err[i] = errno; // Capture error per wheel
        }

        // =========================================================================
        // STATIC DEBUGGER DASHBOARD (NO SCROLLING)
        // =========================================================================
        cycle++;
        if (DEBUG_ENABLE && (cycle % debug_interval) == 0)
        {
            struct timespec debug_ts;
            clock_gettime(CLOCK_MONOTONIC, &debug_ts);
            long jitter_us = (debug_ts.tv_sec - cycle_start.tv_sec) * 1000000L +
                             (debug_ts.tv_nsec - cycle_start.tv_nsec) / 1000L;

            const char* corner_names[4] = {"FL", "FR", "RL", "RR"};

            // Calculate incoming packet rate (Hz)
            uint32_t current_rx_matlab = atomic_load_explicit(&rx_matlab_count, memory_order_relaxed);
            uint32_t current_rx_steer  = atomic_load_explicit(&rx_steer_count, memory_order_relaxed);
            
            // Formula: (New Packets) * (1000 ms / debug_interval_ms)
            uint32_t rate_matlab = (current_rx_matlab - last_rx_matlab) * (1000 / debug_interval);
            uint32_t rate_steer  = (current_rx_steer - last_rx_steer) * (1000 / debug_interval);
            
            last_rx_matlab = current_rx_matlab;
            last_rx_steer  = current_rx_steer;

            // Lock cursor position to the top-left of the screen
            DEBUG_LOG("\033[H\033[J"); 

            DEBUG_LOG("======================================================================\n");
            DEBUG_LOG("[VCU TELEMETRY] Control Loop: 1000Hz | Jitter: %-4ld us\n", jitter_us);
            DEBUG_LOG("======================================================================\n");
            
            DEBUG_LOG("\n>>> RX: INCOMING DATA (RECEIVE) <<<\n");
            DEBUG_LOG("----------------------------------------------------------------------\n");
            if (rate_matlab > 0) {
                DEBUG_LOG("  MATLAB CARSIM : [ACTIVE ] %-4u Hz | Total Pkt: %-6u | Seq: %-3u\n", rate_matlab, current_rx_matlab, cur_matlab.seq);
            } else {
                DEBUG_LOG("  MATLAB CARSIM : [WAITING] No incoming data (0 Hz)\n");
            }
            
            if (rate_steer > 0) {
                DEBUG_LOG("  PXN STEER USB : [ACTIVE ] %-4u Hz | Total Pkt: %-6u\n", rate_steer, current_rx_steer);
            } else {
                DEBUG_LOG("  PXN STEER USB : [WAITING] Steering not detected / idle (0 Hz)\n");
            }
            
            DEBUG_LOG("\n>>> COMPUTATION: SENSOR STATE <<<\n");
            DEBUG_LOG("----------------------------------------------------------------------\n");
            DEBUG_LOG("  Steer: %-6.2f° | Accel: %-5.2f | Brake: %-1.2f | Vx_Sim: %-5.2f km/h\n", 
                      steer_deg, vx_des, cur_steer.brake / 100.0f, cur_matlab.Vx / 100.0f);
            
            DEBUG_LOG("\n>>> TX: OUTGOING DATA (TRANSMIT) <<<\n");
            DEBUG_LOG("----------------------------------------------------------------------\n");
            if (tx_matlab_status > 0) {
                DEBUG_LOG("  TO MATLAB     : [SUCCESS] Brake Frame sent (Seq: %-3u)\n", seq);
            } else {
                DEBUG_LOG("  TO MATLAB     : [FAILED ] Error: %s\n", strerror(tx_matlab_err));
            }
            
            DEBUG_LOG("  TO STM32 CORNER:\n");
            for (int i = 0; i < CORNER_COUNT; i++) {
                if (tx_corner_status[i] > 0) {
                    DEBUG_LOG("  %-2s (ID %d)     : [SUCCESS] | Tm_ref: %-7.2f Nm | Ang_ref: %-7.2f°\n", 
                              corner_names[i], i, tv.TV_Y_Tm[i], fwis.output[i]);
                } else {
                    DEBUG_LOG("  %-2s (ID %d)     : [FAILED ] | Error: %s\n", 
                              corner_names[i], i, strerror(tx_corner_err[i]));
                }
            }
            DEBUG_LOG("======================================================================\n");
        }

        // 2 ms sleep
        thread_ts.tv_nsec += 2000000;
        if (thread_ts.tv_nsec >= 1000000000)
        {
            thread_ts.tv_sec++;
            thread_ts.tv_nsec -= 1000000000;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &thread_ts, NULL);
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

    FWIS_Init(&fwis, FWIS_WB, FWIS_CG, FWIS_WT, FWIS_HWB, FWIS_HWT);

    RT_MODEL_Torque_Vectoring_T *const Torque_Vectoring_M = tv.Torque_Vectoring_MPtr;
    Torque_Vectoring_M->blockIO = &tv.Torque_Vectoring_B;
    Torque_Vectoring_M->dwork = &tv.Torque_Vectoring_DW;

    Torque_Vectoring_initialize(Torque_Vectoring_M,
        &tv.TV_U_Vx_des, &tv.TV_U_Vx, &tv.TV_U_Vy,
        tv.TV_U_AngWheel, &tv.TV_U_r,
        tv.TV_U_Fy, tv.TV_U_Fz,
        tv.TV_Y_Tm, tv.TV_Y_Fx_opt,
        &tv.TV_Y_Mx_total, &tv.TV_Y_Fx_total,
        &tv.TV_Y_Mzd, &tv.TV_Y_r_des,
        &tv.TV_Y_beta_des, tv.TV_Y_Ca, &tv.TV_Y_beta);

    steer_buf[0] = (steer_sample_t){0};
    steer_buf[1] = (steer_sample_t){0};
    atomic_store(&steer_idx, 0);

    memset(&matlab_buf[0], 0, sizeof(matlab_recv_frame_t));
    memset(&matlab_buf[1], 0, sizeof(matlab_recv_frame_t));
    atomic_store(&matlab_idx, 0);

    printf("Initialize UDP network...\n");

    if (udp_rx_init(&rx_vcu, VCU_PORT) < 0)
    {
        fprintf(stderr, "Failed open port VCU\n");
        return -1;
    }

    for (int i = 0; i < CORNER_COUNT; i++)
    {
        if (udp_tx_init(&tx_corner[i], 0, 0, corner_ip[i]) < 0)
        {
            fprintf(stderr, "Failed to create TX socket for corner[%d]\n", i);
            return -1;
        }
        udp_tx_set_target(&tx_corner[i], corner_ip[i], corner_port[i]);
    }

    // Set local_port parameter to 0 for MATLAB (PC) sender to use a random OS port (Prevents conflict)
    if (udp_tx_init(&tx_matlab, 0, 0, matlab_ip) < 0)
    {
        fprintf(stderr, "Failed to create TX socket for Matlab\n");
        return -1;
    }
    udp_tx_set_target(&tx_matlab, matlab_ip, MATLAB_PORT);

    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1)
    {
        printf("mlockall failed: %m\n");
        exit(-2);
    }

    pthread_t t_steer, t_net, t_ctrl;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setstacksize(&attr, 65536);

    param.sched_priority = 85;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_steer, &attr, thread_steer_reader, NULL);

    param.sched_priority = 80;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_net, &attr, thread_net_rx, NULL);

    param.sched_priority = 95;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_ctrl, &attr, thread_control, NULL);

    pthread_attr_destroy(&attr);

    // Waiting prompt before dashboard appears
    printf("Waiting for synchronization...\n");

    pthread_join(t_ctrl, NULL);
    pthread_join(t_net, NULL);
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
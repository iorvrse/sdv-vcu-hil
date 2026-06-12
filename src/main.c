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
#include <fcntl.h>
#include <linux/rpmsg.h>

#include "comm.h"
#include "steer.h"

//  Debug logging control
#define DEBUG_ENABLE  1
#if DEBUG_ENABLE
    #define DEBUG_LOG(fmt, ...)   printf("[DBG] " fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG(fmt, ...)   ((void)0)
#endif

//  IP Config & state
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

// RPMsg Character Device File Descriptors
int fd_rpmsg_tv = -1;
int fd_rpmsg_fwis = -1;

static volatile int keep_running = 1;

static steer_sample_t steer_buf[2];

// Atomic Counters to monitor RX frequency (Hz)
static atomic_uint_fast32_t rx_matlab_count = 0;
static atomic_uint_fast32_t rx_steer_count  = 0;
static atomic_uint_fast32_t rx_tv_count     = 0;
static atomic_uint_fast32_t rx_fwis_count   = 0;

// Double Buffers for Lock-Free Thread Sync
static atomic_uint_fast8_t steer_idx = 0;

static matlab_recv_frame_t matlab_buf[2];
static atomic_uint_fast8_t matlab_idx = 0;

static rpmsg_tv_out_t tv_buf[2];
static atomic_uint_fast8_t tv_idx = 0;

static rpmsg_fwis_out_t fwis_buf[2];
static atomic_uint_fast8_t fwis_idx = 0;

// RPMsg Char Device Initialization Helper
int init_rpmsg_char(const char *dev_path)
{
    // Open in standard Read/Write mode.
    // Without O_NONBLOCK, read() operations will naturally block 
    // keeping the dedicated RX threads asleep until data arrives.
    int fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        return -1;
    }
    return fd;
}

static void sleep_ms(long ms)
{
    struct timespec req = { ms / 1000, (ms % 1000) * 1000000L };
    nanosleep(&req, NULL);
}

void sigint_handler(int dummy)
{
    keep_running = 0;
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

            uint_fast8_t next = 1 - atomic_load_explicit(&matlab_idx, memory_order_acquire);
            memcpy(&matlab_buf[next], m2v, sizeof(matlab_recv_frame_t));
            atomic_store_explicit(&matlab_idx, next, memory_order_release);
            
            atomic_fetch_add_explicit(&rx_matlab_count, 1, memory_order_relaxed);
        }
    }
    return NULL;
}

// =================================================================
// IPC Receiver Thread: Cortex-M4_0 (Torque Vectoring)
// =================================================================
void *thread_tv_rx(void *arg)
{
    (void)arg;
    rpmsg_tv_out_t res;
    
    while (keep_running)
    {
        // Block indefinitely until M4_0 computes the frame
        int bytes = read(fd_rpmsg_tv, &res, sizeof(rpmsg_tv_out_t));
        
        if (bytes == sizeof(rpmsg_tv_out_t))
        {
            uint_fast8_t next = 1 - atomic_load_explicit(&tv_idx, memory_order_acquire);
            memcpy(&tv_buf[next], &res, sizeof(rpmsg_tv_out_t));
            atomic_store_explicit(&tv_idx, next, memory_order_release);
            
            atomic_fetch_add_explicit(&rx_tv_count, 1, memory_order_relaxed);
        }
    }
    return NULL;
}

// =================================================================
// IPC Receiver Thread: Cortex-M4_1 (FWIS)
// =================================================================
void *thread_fwis_rx(void *arg)
{
    (void)arg;
    rpmsg_fwis_out_t res;

    while (keep_running)
    {
        // Block indefinitely until M4_1 computes the frame
        int bytes = read(fd_rpmsg_fwis, &res, sizeof(rpmsg_fwis_out_t));
        
        if (bytes == sizeof(rpmsg_fwis_out_t))
        {
            uint_fast8_t next = 1 - atomic_load_explicit(&fwis_idx, memory_order_acquire);
            memcpy(&fwis_buf[next], &res, sizeof(rpmsg_fwis_out_t));
            atomic_store_explicit(&fwis_idx, next, memory_order_release);
            
            atomic_fetch_add_explicit(&rx_fwis_count, 1, memory_order_relaxed);
        }
    }
    return NULL;
}

// =================================================================
// Control thread (Routing & Dispatch Manager)
// =================================================================
void *thread_control(void *arg)
{
    (void)arg;

    struct timespec thread_ts;
    clock_gettime(CLOCK_MONOTONIC, &thread_ts);
    uint8_t seq = 0;
    uint32_t cycle = 0;
    const uint32_t debug_interval = 100;
    struct timespec cycle_start;

    ssize_t tx_matlab_status = 0;
    int     tx_matlab_err    = 0;
    ssize_t tx_corner_status[CORNER_COUNT] = {0};
    int     tx_corner_err[CORNER_COUNT]    = {0};

    uint32_t last_rx_matlab = 0;
    uint32_t last_rx_steer  = 0;
    uint32_t last_rx_tv     = 0;
    uint32_t last_rx_fwis   = 0;

    printf("\033[2J"); 

    while (keep_running)
    {
        clock_gettime(CLOCK_MONOTONIC, &cycle_start);
        seq++;

        // Fetch Latest Inputs (From UDP/USB Threads)
        int matlab_idx_local = atomic_load_explicit(&matlab_idx, memory_order_acquire);
        matlab_recv_frame_t cur_matlab = matlab_buf[matlab_idx_local];

        int steer_idx_local = atomic_load_explicit(&steer_idx, memory_order_acquire);
        steer_sample_t cur_steer = steer_buf[steer_idx_local];

        // Dispatch IPC Commands to Hardware (Non-Blocking)
        rpmsg_tv_in_t tv_job;
        tv_job.Vx_des  = cur_steer.accel;
        tv_job.Vx = cur_matlab.Vx;
        tv_job.Vy = cur_matlab.Vy;
        tv_job.angWheel[0] = cur_matlab.angWheel[0];
        tv_job.angWheel[1] = cur_matlab.angWheel[1];
        tv_job.angWheel[2] = cur_matlab.angWheel[2];
        tv_job.angWheel[3] = cur_matlab.angWheel[3];
        tv_job.yawRate = cur_matlab.yawRate;
        tv_job.Fy[0] = cur_matlab.Fy[0];
        tv_job.Fy[1] = cur_matlab.Fy[1];
        tv_job.Fy[2] = cur_matlab.Fy[2];
        tv_job.Fy[3] = cur_matlab.Fy[3];
        tv_job.Fz[0] = cur_matlab.Fz[0];
        tv_job.Fz[1] = cur_matlab.Fz[1];
        tv_job.Fz[2] = cur_matlab.Fz[2];
        tv_job.Fz[3] = cur_matlab.Fz[3];

        rpmsg_fwis_in_t fwis_job;
        fwis_job.steer_angle = cur_steer.steer + 4000;
        fwis_job.Vx          = cur_matlab.Vx;

        write(fd_rpmsg_tv, &tv_job, sizeof(rpmsg_tv_in_t));
        write(fd_rpmsg_fwis, &fwis_job, sizeof(rpmsg_fwis_in_t));

        // Send Brake to MATLAB
        vcu_brake_matlab_frame_t brake_frame = {
            .header = MATLAB_BRAKE_HEADER,
            .id     = MATLAB_BRAKE_ID,
            .brake  = cur_steer.brake,
            .seq    = seq
        };

        tx_matlab_status = udp_comm_send(&tx_matlab, &tx_matlab.addr, &brake_frame, sizeof(brake_frame), MSG_DONTWAIT);
        if (tx_matlab_status < 0) tx_matlab_err = errno;

        // Fetch Latest Coprocessor Results (From IPC Threads)
        int tv_idx_local = atomic_load_explicit(&tv_idx, memory_order_acquire);
        rpmsg_tv_out_t cur_tv_res = tv_buf[tv_idx_local];

        int fwis_idx_local = atomic_load_explicit(&fwis_idx, memory_order_acquire);
        rpmsg_fwis_out_t cur_fwis_res = fwis_buf[fwis_idx_local];

        // Route Complete Package to STM32 Zone Controllers
        corner_send_frame_t corner_frame = {
            .header = CORNER_FRAME_HEADER,
            .Vx     = cur_matlab.Vx,
            .Vx_des = cur_steer.accel,
            .Mzd    = cur_tv_res.Mzd
        };

        for (int i = 0; i < CORNER_COUNT; i++)
        {
            corner_frame.id = i + 1;
            corner_frame.Vx_wheel = cur_matlab.Vx_wheel[i];
            corner_frame.seq = seq;
            
            // Map values directly from asynchronous answers
            corner_frame.Tm_ref  = cur_tv_res.Tm_ref[i];
            corner_frame.Ang_ref = cur_fwis_res.Ang_ref[i];
            
            tx_corner_status[i] = udp_comm_send(&tx_corner[i], &tx_corner[i].addr, &corner_frame, sizeof(corner_frame), MSG_DONTWAIT);
            if (tx_corner_status[i] < 0) tx_corner_err[i] = errno;
        }

        // STATIC DEBUGGER DASHBOARD
        cycle++;
        if (DEBUG_ENABLE && (cycle % debug_interval) == 0)
        {
            struct timespec debug_ts;
            clock_gettime(CLOCK_MONOTONIC, &debug_ts);
            long jitter_us = (debug_ts.tv_sec - cycle_start.tv_sec) * 1000000L +
                             (debug_ts.tv_nsec - cycle_start.tv_nsec) / 1000L;

            const char* corner_names[4] = {"FL", "FR", "RL", "RR"};

            // Read hardware frequencies
            uint32_t current_rx_matlab = atomic_load_explicit(&rx_matlab_count, memory_order_relaxed);
            uint32_t current_rx_steer  = atomic_load_explicit(&rx_steer_count, memory_order_relaxed);
            uint32_t current_rx_tv     = atomic_load_explicit(&rx_tv_count, memory_order_relaxed);
            uint32_t current_rx_fwis   = atomic_load_explicit(&rx_fwis_count, memory_order_relaxed);
            
            uint32_t rate_matlab = (current_rx_matlab - last_rx_matlab) * (1000 / debug_interval);
            uint32_t rate_steer  = (current_rx_steer - last_rx_steer) * (1000 / debug_interval);
            uint32_t rate_tv     = (current_rx_tv - last_rx_tv) * (1000 / debug_interval);
            uint32_t rate_fwis   = (current_rx_fwis - last_rx_fwis) * (1000 / debug_interval);
            
            last_rx_matlab = current_rx_matlab;
            last_rx_steer  = current_rx_steer;
            last_rx_tv     = current_rx_tv;
            last_rx_fwis   = current_rx_fwis;

            DEBUG_LOG("\033[H\033[J"); 

            DEBUG_LOG("======================================================================\n");
            DEBUG_LOG("[VCU TELEMETRY] Control Loop: 1000Hz | Jitter: %-4ld us\n", jitter_us);
            DEBUG_LOG("======================================================================\n");
            
            DEBUG_LOG("\n>>> RX: INCOMING DATA (RECEIVE) <<<\n");
            DEBUG_LOG("----------------------------------------------------------------------\n");
            
            if (rate_matlab > 0) DEBUG_LOG("  MATLAB CARSIM : [ACTIVE ] %-4u Hz | Total Pkt: %-6u\n", rate_matlab, current_rx_matlab);
            else DEBUG_LOG("  MATLAB CARSIM : [WAITING] No incoming data (0 Hz)\n");
            
            if (rate_steer > 0) DEBUG_LOG("  PXN STEER USB : [ACTIVE ] %-4u Hz | Total Pkt: %-6u\n", rate_steer, current_rx_steer);
            else DEBUG_LOG("  PXN STEER USB : [WAITING] Steering not detected / idle (0 Hz)\n");
            
            DEBUG_LOG("\n>>> IPC: CORTEX-M COPROCESSOR STATUS <<<\n");
            DEBUG_LOG("----------------------------------------------------------------------\n");
            if (rate_tv > 0) DEBUG_LOG("  M4_0 (TV)     : [ACTIVE ] %-4u Hz | Mzd: %-6u\n", rate_tv, cur_tv_res.Mzd);
            else DEBUG_LOG("  M4_0 (TV)     : [TIMEOUT] Node stalled or crashed (0 Hz)\n");

            if (rate_fwis > 0) DEBUG_LOG("  M4_1 (FWIS)   : [ACTIVE ] %-4u Hz\n", rate_fwis);
            else DEBUG_LOG("  M4_1 (FWIS)   : [TIMEOUT] Node stalled or crashed (0 Hz)\n");

            DEBUG_LOG("\n>>> TX: OUTGOING DATA (TRANSMIT) <<<\n");
            DEBUG_LOG("----------------------------------------------------------------------\n");
            if (tx_matlab_status > 0) DEBUG_LOG("  TO MATLAB     : [SUCCESS] Brake Frame sent (Seq: %-3u)\n", seq);
            else DEBUG_LOG("  TO MATLAB     : [FAILED ] Error: %s\n", strerror(tx_matlab_err));

            DEBUG_LOG("  TO STM32 CORNER:\n");
            for (int i = 0; i < CORNER_COUNT; i++)
            {
                if (tx_corner_status[i] > 0)
                {
                    DEBUG_LOG("  %-2s (ID %d)     : [SUCCESS] | Tm_ref: %-7d | Ang_ref: %-7d\n", 
                              corner_names[i], i+1, corner_frame.Tm_ref, corner_frame.Ang_ref);
                }
                else
                {
                    DEBUG_LOG("  %-2s (ID %d)     : [FAILED ] | Error: %s\n", 
                              corner_names[i], i+1, strerror(tx_corner_err[i]));
                }
            }
            DEBUG_LOG("======================================================================\n");
        }

        thread_ts.tv_nsec += 2000000; // 2 ms frame pacing
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

    printf("Initialize IPC connections to Cortex-M4...\n");

    fd_rpmsg_tv = init_rpmsg_char("/dev/rpmsg0");
    if (fd_rpmsg_tv < 0) {
        fprintf(stderr, "FATAL: Failed to open M4_0 endpoint at /dev/rpmsg0\n");
        return -1;
    }

    fd_rpmsg_fwis = init_rpmsg_char("/dev/rpmsg1");
    if (fd_rpmsg_fwis < 0) {
        fprintf(stderr, "FATAL: Failed to open M4_1 endpoint at /dev/rpmsg1\n");
        return -1;
    }

    // Init Double Buffers
    steer_buf[0] = (steer_sample_t){0};
    steer_buf[1] = (steer_sample_t){0};
    atomic_store(&steer_idx, 0);

    memset(&matlab_buf[0], 0, sizeof(matlab_recv_frame_t));
    memset(&matlab_buf[1], 0, sizeof(matlab_recv_frame_t));
    atomic_store(&matlab_idx, 0);

    memset(&tv_buf[0], 0, sizeof(rpmsg_tv_out_t));
    memset(&tv_buf[1], 0, sizeof(rpmsg_tv_out_t));
    atomic_store(&tv_idx, 0);

    memset(&fwis_buf[0], 0, sizeof(rpmsg_fwis_out_t));
    memset(&fwis_buf[1], 0, sizeof(rpmsg_fwis_out_t));
    atomic_store(&fwis_idx, 0);

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

    pthread_t t_steer, t_net, t_tv_rx, t_fwis_rx, t_ctrl;
    pthread_attr_t attr;
    struct sched_param param;

    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setstacksize(&attr, 65536);

    // Launch I/O Polling Threads
    param.sched_priority = 85;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_steer, &attr, thread_steer_reader, NULL);

    param.sched_priority = 80;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_net, &attr, thread_net_rx, NULL);

    // Launch IPC Receiver Threads
    param.sched_priority = 90;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_tv_rx, &attr, thread_tv_rx, NULL);
    pthread_create(&t_fwis_rx, &attr, thread_fwis_rx, NULL);

    // Launch Main Router
    param.sched_priority = 95;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_ctrl, &attr, thread_control, NULL);

    pthread_attr_destroy(&attr);

    printf("Waiting for synchronization...\n");

    pthread_join(t_ctrl, NULL);
    pthread_join(t_net, NULL);
    pthread_join(t_steer, NULL);
    pthread_join(t_tv_rx, NULL);
    pthread_join(t_fwis_rx, NULL);

    printf("\nShutting down the system...\n");
    udp_comm_close(&rx_vcu.sockfd);
    udp_comm_close(&tx_matlab.sockfd);
    for (int i = 0; i < CORNER_COUNT; i++)
    {
        udp_comm_close(&tx_corner[i].sockfd);
    }

    close(fd_rpmsg_tv);
    close(fd_rpmsg_fwis);

    return 0;
}
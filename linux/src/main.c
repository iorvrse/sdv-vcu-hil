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
#include <poll.h>
#include <linux/rpmsg.h>

#include "comm.h"
#include "steer.h"

// Debug logging control
#define DEBUG_ENABLE  1
#if DEBUG_ENABLE
    #define DEBUG_LOG(fmt, ...)   printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_LOG(fmt, ...)   ((void)0)
#endif

// IP Config and state
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

// RPMsg character device file descriptors
int fd_rpmsg_tv = -1;
int fd_rpmsg_fwis = -1;

static volatile int keep_running = 1;

// Lock-free double buffers for asynchronous network and USB inputs
static steer_sample_t steer_buf[2];
static atomic_uint_fast8_t steer_idx = 0;

static matlab_recv_frame_t matlab_buf[2];
static atomic_uint_fast8_t matlab_idx = 0;

// RPMsg char device initialization
int init_rpmsg_char(const char *dev_path)
{
    int fd = open(dev_path, O_RDWR | O_NONBLOCK);
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
    (void)dummy;
    keep_running = 0;
}

// Steering wheel input polling thread
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
            atomic_store_explicit(&steer_idx, next, memory_order_release);
        }
    }

    steer_close(my_steer);
    return NULL;
}

// Matlab UDP receiver thread
void *thread_net_rx(void *arg)
{
    (void)arg;
    uint8_t rx_buf[1024];
    struct sockaddr_in src;
    ssize_t n;

    while (keep_running)
    {
        n = udp_comm_recv(&rx_vcu, rx_buf, sizeof(rx_buf), &src, MSG_DONTWAIT);

        if (n <= 0)
        {
            sleep_ms(1);
            continue;
        }

        if (n == sizeof(matlab_recv_frame_t) && rx_buf[0] == MATLAB_FRAME_HEADER)
        {
            matlab_recv_frame_t *m2v = (matlab_recv_frame_t*)rx_buf;
            if (m2v->id != MATLAB_FRAME_ID) continue;

            uint_fast8_t next = 1 - atomic_load_explicit(&matlab_idx, memory_order_acquire);
            memcpy(&matlab_buf[next], m2v, sizeof(matlab_recv_frame_t));
            atomic_store_explicit(&matlab_idx, next, memory_order_release);
        }
    }
    return NULL;
}

// Main real-time dispatch and routing loop
void *thread_control(void *arg)
{
    (void)arg;

    uint8_t seq = 0;
    uint32_t cycle = 0;
    const uint32_t debug_interval = 500; // Increased to prevent flickering at high speeds
    
    printf("\033[2J");

    // Initialize blank results so we don't send garbage if M4 is slow on boot
    rpmsg_tv_out_t cur_tv_res = {0};
    rpmsg_fwis_out_t cur_fwis_res = {0};

    while (keep_running)
    {
        seq++;

        // Read latest asynchronous inputs
        int matlab_idx_local = atomic_load_explicit(&matlab_idx, memory_order_acquire);
        matlab_recv_frame_t cur_matlab = matlab_buf[matlab_idx_local];

        int steer_idx_local = atomic_load_explicit(&steer_idx, memory_order_acquire);
        steer_sample_t cur_steer = steer_buf[steer_idx_local];

        // Format and send jobs to Cortex-M coprocessors
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

        ssize_t w1 = write(fd_rpmsg_tv, &tv_job, sizeof(rpmsg_tv_in_t));
        ssize_t w2 = write(fd_rpmsg_fwis, &fwis_job, sizeof(rpmsg_fwis_in_t));
        (void)w1; (void)w2;

        struct pollfd pfds[2] = {
            { .fd = fd_rpmsg_tv, .events = POLLIN },
            { .fd = fd_rpmsg_fwis, .events = POLLIN }
        };

        // Wait up to 1ms for the coprocessors to finish computing
        int poll_ret = poll(pfds, 2, 1); 

        if (poll_ret > 0)
        {
            if (pfds[0].revents & POLLIN)
            {
                read(fd_rpmsg_tv, &cur_tv_res, sizeof(cur_tv_res));
            }
            if (pfds[1].revents & POLLIN)
            {
                read(fd_rpmsg_fwis, &cur_fwis_res, sizeof(cur_fwis_res));
            }
        }

        // Transmit brake status to Matlab
        vcu_brake_matlab_frame_t brake_frame = {
            .header = MATLAB_BRAKE_HEADER,
            .id     = MATLAB_BRAKE_ID,
            .brake  = cur_steer.brake,
            .seq    = seq
        };
        udp_comm_send(&tx_matlab, &tx_matlab.addr, &brake_frame, sizeof(brake_frame), MSG_DONTWAIT);

        // Map fresh values and dispatch to STM32 Corner Controllers
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
            corner_frame.Tm_ref  = cur_tv_res.Tm_ref[i];
            corner_frame.Ang_ref = cur_fwis_res.Ang_ref[i];

            udp_comm_send(&tx_corner[i], &tx_corner[i].addr, &corner_frame, sizeof(corner_frame), MSG_DONTWAIT);
        }

        // Live Debug Dashboard
        cycle++;
        if (DEBUG_ENABLE && (cycle % debug_interval) == 0)
        {
            DEBUG_LOG("\033[H\033[J");
            DEBUG_LOG("=== VCU DASHBOARD (MAX SPEED) ===\n");
            
            // Format raw integer steering value to real degrees (-40.00 to 40.00)
            DEBUG_LOG("[STEER IN]  Ang: %-6.2f deg | Accel: %-5d | Brake: %-5d\n", 
                      cur_steer.steer / 100.0f, cur_steer.accel, cur_steer.brake);
            
            // Decode Matlab integers
            DEBUG_LOG("[MATLAB IN] Vx: %-6.2f m/s | Vy: %-6.2f m/s\n", 
                      cur_matlab.Vx / 100.0f, 
                      (cur_matlab.Vy / 100.0f) - 10.0f);
                      
            DEBUG_LOG("            Ang: %.2f, %.2f, %.2f, %.2f rad\n", 
                      (cur_matlab.angWheel[0] / 1000.0f) - 0.7f, 
                      (cur_matlab.angWheel[1] / 1000.0f) - 0.7f, 
                      (cur_matlab.angWheel[2] / 1000.0f) - 0.7f, 
                      (cur_matlab.angWheel[3] / 1000.0f) - 0.7f);
                      
            DEBUG_LOG("=================================\n");
        }

        usleep(200); 
    }

    return NULL;
}

// System entry point
int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

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

    // Initialize network buffers
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

    if (udp_tx_init(&tx_matlab, 0, 0, matlab_ip) < 0)
    {
        fprintf(stderr, "Failed to create TX socket for Matlab\n");
        return -1;
    }
    udp_tx_set_target(&tx_matlab, matlab_ip, MATLAB_PORT);

    // Lock memory to prevent swap delays in real-time execution
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

    // Launch background asynchronous I/O threads
    param.sched_priority = 95;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_steer, &attr, thread_steer_reader, NULL);

    param.sched_priority = 93;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_net, &attr, thread_net_rx, NULL);

    // Launch critical real-time routing control loop
    param.sched_priority = 99;
    pthread_attr_setschedparam(&attr, &param);
    pthread_create(&t_ctrl, &attr, thread_control, NULL);

    pthread_attr_destroy(&attr);

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

    close(fd_rpmsg_tv);
    close(fd_rpmsg_fwis);

    return 0;
}
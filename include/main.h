#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <time.h>

#define CORNER_COUNT 4

#define MATLAB_PORT 4093
#define VCU_PORT    5055

#define MATLAB_FRAME_HEADER 0xAC
#define CORNER_FRAME_HEADER 0xCB
#define MATLAB_BRAKE_HEADER 0xCA

#define MATLAB_FRAME_ID 0x05
#define MATLAB_BRAKE_ID 0x06

typedef struct
{
    uint8_t brake;
    uint16_t accel;
    int16_t steer;
    struct timespec ts;
} steer_sample_t;

// Network Frames (UDP)
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t id;
    uint16_t Vx;
    uint16_t Vy;
    uint16_t angWheel[4];
    uint16_t yawRate;
    uint32_t Fy[4];
    uint32_t Fz[4];
    uint16_t Vx_wheel[4];
    uint8_t seq;
} matlab_recv_frame_t;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t id;
    uint16_t Tm_ref;
    uint16_t Vx;
    uint16_t Vx_wheel;
    uint16_t Vx_des;
    uint16_t Ang_ref;
    uint32_t Mzd;
    uint8_t seq;
} corner_send_frame_t;

typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t id;
    uint8_t brake;
    uint8_t seq;
} vcu_brake_matlab_frame_t;


// IPC Frames
typedef struct __attribute__((packed))
{
    uint16_t Vx_des;
    uint16_t Vx;
    uint16_t Vy;
    uint16_t angWheel[4];
    uint16_t yawRate;
    uint32_t Fy[4];
    uint32_t Fz[4];
} rpmsg_tv_in_t;

typedef struct __attribute__((packed))
{
    uint16_t Tm_ref[4];
    uint32_t Mzd;
} rpmsg_tv_out_t;

// --- Cortex-M4_1 (4WIS Node) ---
typedef struct __attribute__((packed))
{
    uint16_t steer_angle;
    uint16_t Vx;
} rpmsg_fwis_in_t;

typedef struct __attribute__((packed))
{
    uint16_t Ang_ref[4];
} rpmsg_fwis_out_t;

#endif // MAIN_H
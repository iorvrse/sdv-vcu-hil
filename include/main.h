#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include "Torque_Vectoring.h"

// Network and communication config
#define CORNER_COUNT 4

#define MATLAB_PORT 4093
#define VCU_PORT    5055

#define MATLAB_FRAME_HEADER 0xAA
#define CORNER_FRAME_HEADER 0xCA

#define MATLAB_FRAME_ID 0x05

// 4WIS
#define FWIS_WB     3.1f
#define FWIS_CG     1.55f
#define FWIS_WT     1.49f
#define FWIS_HWB    1.55f
#define FWIS_HWT    0.745f

typedef enum
{
    ID_FRONT_LEFT_WHEEL = 0x01,
    ID_FRONT_RIGHT_WHEEL,
    ID_REAR_LEFT_WHEEL,
    ID_REAR_RIGHT_WHEEL,
    ID_PC_MATLAB,
    ID_VCU
} DeviceID;

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
    uint8_t brake;
    uint8_t seq;
} corner_send_frame_t;

typedef struct
{
    RT_MODEL_Torque_Vectoring_T Torque_Vectoring_M_;
    RT_MODEL_Torque_Vectoring_T *const Torque_Vectoring_MPtr;
    DW_Torque_Vectoring_T Torque_Vectoring_DW;
    B_Torque_Vectoring_T Torque_Vectoring_B;

    real_T TV_U_Vx_des;
    real_T TV_U_Vx;
    real_T TV_U_Vy;
    real_T TV_U_AngWheel[4];
    real_T TV_U_r;
    real_T TV_U_Fy[4];
    real_T TV_U_Fz[4];

    real_T TV_Y_Tm[4];
    real_T TV_Y_Fx_opt[4];
    real_T TV_Y_Mx_total;
    real_T TV_Y_Fx_total;
    real_T TV_Y_Mzd;
    real_T TV_Y_r_des;
    real_T TV_Y_beta_des;
    real_T TV_Y_Ca[2];
    real_T TV_Y_beta;
} torque_vectoring_t;

#endif // MAIN_H
#ifndef SPEEDCONTROL_H
#define SPEEDCONTROL_H

typedef struct {
    float FL;
    float FR;
    float RL;
    float RR;
} SC;

typedef struct {
    float wb;
    float Hwb;
    float Hwt;

    float sfl;
    float sfr;
    float srl;
    float srr;
} SpeedControl_Context;

void SpeedControl_Init(SpeedControl_Context *ctx);
void SpeedControl_Compute(SpeedControl_Context *ctx, float Vx, float fl_deg, float fr_deg, float rl_deg, float rr_deg);
SC SpeedControl_GetValues(const SpeedControl_Context *ctx);

#endif

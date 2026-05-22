#ifndef FWS_H
#define FWS_H

typedef struct {
    float fl;
    float fr;
    float rl;
    float rr;
} FWS;

typedef struct {
    float wb;   // wheelbase
    float CG;   // center of gravity
    float wt;   // track width
    float Hwb;
    float Hwt;

    float xo;
    float yo;
    float R1;

    float fl, fr, rl, rr; // output (deg)
} FWS_Context;

void FWS_Init(FWS_Context *ctx);
void FWS_Compute(FWS_Context *ctx, float steer_deg, float speed);
FWS FWS_GetValues(const FWS_Context *ctx);

#endif
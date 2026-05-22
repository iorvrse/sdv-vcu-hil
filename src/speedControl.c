#include "speedControl.h"
#include <math.h>

#define EPS 1e-9f

static inline float deg2rad(float deg) {
    return deg * (M_PI / 180.0f);
}

void SpeedControl_Init(SpeedControl_Context *ctx) {
    ctx->wb  = 3.1f;
    ctx->Hwb = 1.55f;
    ctx->Hwt = 0.745f;
    ctx->sfl = ctx->sfr = ctx->srl = ctx->srr = 0.0f;
}

void SpeedControl_Compute(SpeedControl_Context *ctx, float Vx_kmh, float fl_deg, float fr_deg, float rl_deg, float rr_deg) {
    float Vcg = Vx_kmh * 1000.0f / 3600.0f;  // convert km/h → m/s

    float dfl = deg2rad(fl_deg);
    float dfr = deg2rad(fr_deg);
    float drl = deg2rad(rl_deg);
    float drr = deg2rad(rr_deg);

    float delta_f = 0.5f * (dfl + dfr);
    float delta_r = 0.5f * (drl + drr);

    float a = ctx->wb / 2.0f;
    float b = ctx->wb / 2.0f;

    float yfl = -ctx->Hwt;
    float yfr = +ctx->Hwt;
    float yrl = -ctx->Hwt;
    float yrr = +ctx->Hwt;

    float denom = tanf(delta_f) - tanf(delta_r);

    // Jika steer sama, semua roda dapat kecepatan sama
    if (fabsf(denom) < EPS) {
        ctx->sfl = ctx->sfr = ctx->srl = ctx->srr = Vx_kmh;
        return;
    }

    float R = ctx->wb / denom;
    float Rcg = fabsf(R);

    // Radius ke masing-masing roda
    float R_FL = sqrtf((R - yfl)*(R - yfl) + a*a);
    float R_FR = sqrtf((R - yfr)*(R - yfr) + a*a);
    float R_RL = sqrtf((R - yrl)*(R - yrl) + b*b);
    float R_RR = sqrtf((R - yrr)*(R - yrr) + b*b);

    // Kecepatan per roda (m/s)
    float vms_FL = Vcg * (R_FL / Rcg);
    float vms_FR = Vcg * (R_FR / Rcg);
    float vms_RL = Vcg * (R_RL / Rcg);
    float vms_RR = Vcg * (R_RR / Rcg);

    // Konversi kembali ke km/h
    ctx->sfl = vms_FL * 3.6f;
    ctx->sfr = vms_FR * 3.6f;
    ctx->srl = vms_RL * 3.6f;
    ctx->srr = vms_RR * 3.6f;
}

SC SpeedControl_GetValues(const SpeedControl_Context *ctx) {
    SC s;
    s.FL = ctx->sfl;
    s.FR = ctx->sfr;
    s.RL = ctx->srl;
    s.RR = ctx->srr;
    return s;
}
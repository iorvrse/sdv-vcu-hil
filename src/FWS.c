#include "FWS.h"
#include <math.h>

static float map_value(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float deg2rad(float deg) { return deg * (M_PI / 180.0f); }
static inline float rad2deg(float rad) { return rad * (180.0f / M_PI); }

void FWS_Init(FWS_Context *ctx) {
    ctx->wb  = 3.1f;
    ctx->CG  = 1.55f;
    ctx->wt  = 1.49f;
    ctx->Hwb = 1.55f;
    ctx->Hwt = 0.745f;

    ctx->xo = 0;
    ctx->yo = 0;
    ctx->R1 = 0;
    ctx->fl = ctx->fr = ctx->rl = ctx->rr = 0;
}

void FWS_Compute(FWS_Context *ctx, float steer_deg, float speed) {
    // --- step 1: map speed ke panjang wheelbase efektif
    float eff_len = map_value(speed, 0, 60, 1.55f, 3.855f);
    if (eff_len > 3.855f) eff_len = 3.855f;
    if (eff_len < 1.55f)  eff_len = 1.55f;

    ctx->xo = -(eff_len - ctx->CG);
    ctx->R1 = 0;

    // --- step 2: hitung radius belok
    float steer_rad = deg2rad(steer_deg);
    float tan_val = tanf(fabsf(steer_rad));
    if (fabsf(steer_deg) < 1e-3) ctx->R1 = 0;
    else ctx->R1 = (ctx->Hwb / tan_val) + ctx->Hwt;

    ctx->yo = (steer_deg >= 0) ? ctx->R1 : -ctx->R1;

    // --- step 3: hitung sudut tiap roda
    float of = rad2deg(atanf(ctx->CG / (ctx->R1 + ctx->Hwt)));
    float ir = rad2deg(atanf((eff_len - ctx->wb) / (ctx->R1 - ctx->Hwt)));
    float or = rad2deg(atanf((eff_len - ctx->wb) / (ctx->R1 + ctx->Hwt)));

    if (steer_deg < 0) {
        of = -of; ir = -ir; or = -or;
    }

    if (steer_deg > 0) {
        ctx->fl = of; ctx->fr = steer_deg;
        ctx->rl = or; ctx->rr = ir;
    } else if (steer_deg < 0) {
        ctx->fl = steer_deg; ctx->fr = of;
        ctx->rl = ir; ctx->rr = or;
    } else {
        ctx->fl = ctx->fr = ctx->rl = ctx->rr = 0;
    }
}

FWS FWS_GetValues(const FWS_Context *ctx){
    FWS out;
    out.fl = ctx->fl;
    out.fr = ctx->fr;
    out.rl = ctx->rl;
    out.rr = ctx->rr;
    return out;
}

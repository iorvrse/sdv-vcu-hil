#include "FWIS.h"
#include <math.h>

static float map_value(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static inline float deg2rad(float deg) { return deg * (M_PI / 180.0f); }
static inline float rad2deg(float rad) { return rad * (180.0f / M_PI); }

void FWIS_Init(fwis_t *fwis, float wb, float cg, float wt, float Hwb, float Hwt)
{
    fwis->wb  = wb;
    fwis->cg  = cg;
    fwis->wt  = wt;
    fwis->Hwb = Hwb;
    fwis->Hwt = Hwt;

    fwis->xo = 0;
    fwis->yo = 0;
    fwis->R1 = 0;
    fwis->output[0] = fwis->output[1] = fwis->output[2] = fwis->output[3] = 0;
}

void FWIS_Compute(fwis_t *fwis, float steer_deg, float Vx)
{
    // map Vx to effective wheelbase length
    float eff_len = map_value(Vx, 0, 60, 1.55f, 3.855f);
    if (eff_len > 3.855f) eff_len = 3.855f;
    if (eff_len < 1.55f)  eff_len = 1.55f;

    fwis->xo = -(eff_len - fwis->cg);
    fwis->R1 = 0;

    // measure turning radius
    float steer_rad = deg2rad(steer_deg);
    float tan_val = tanf(fabsf(steer_rad));

    if (fabsf(steer_deg) < 1e-3) fwis->R1 = 0;
    else fwis->R1 = (fwis->Hwb / tan_val) + fwis->Hwt;

    fwis->yo = (steer_deg >= 0) ? fwis->R1 : -fwis->R1;

    // measure wheel angle
    float of = rad2deg(atanf(fwis->cg / (fwis->R1 + fwis->Hwt)));
    float ir = rad2deg(atanf((eff_len - fwis->wb) / (fwis->R1 - fwis->Hwt)));
    float or = rad2deg(atanf((eff_len - fwis->wb) / (fwis->R1 + fwis->Hwt)));

    if (steer_deg < 0)
    {
        of = -of; ir = -ir; or = -or;
    }

    if (steer_deg > 0)
    {
        fwis->output[0] = of; fwis->output[1] = steer_deg;
        fwis->output[2] = or; fwis->output[3] = ir;
    }
    else if (steer_deg < 0)
    {
        fwis->output[0] = steer_deg; fwis->output[1] = of;
        fwis->output[2] = ir; fwis->output[3] = or;
    }
    else
    {
        fwis->output[0] = fwis->output[1] = fwis->output[2] = fwis->output[3] = 0;
    }
}

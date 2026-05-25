#ifndef FWIS_H
#define FWIS_H

typedef struct
{
    float wb;   // wheelbase
    float cg;   // center of gravity
    float wt;   // track width
    float Hwb;
    float Hwt;

    float xo;
    float yo;
    float R1;

    float output[4]; // output (deg) for FL, FR, RL, RR
} fwis_t;

void FWIS_Init(fwis_t *fwis, float wb, float cg, float wt, float Hwb, float Hwt);
void FWIS_Compute(fwis_t *fwis, float steer_deg, float Vx);

#endif
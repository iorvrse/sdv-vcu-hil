/*
 * File: Torque_Vectoring_private.h
 *
 * Code generated for Simulink model 'Torque_Vectoring'.
 *
 * Model version                  : 1.46
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Thu May 14 10:54:03 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Torque_Vectoring_private_h_
#define Torque_Vectoring_private_h_
#include "rtwtypes.h"
#include "Torque_Vectoring_types.h"
#include "Torque_Vectoring.h"

extern real_T rt_hypotd(real_T u0, real_T u1);
extern real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
  uint32_T maxIndex);

#endif                                 /* Torque_Vectoring_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

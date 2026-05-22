/*
 * File: Torque_Vectoring_data.c
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

#include "Torque_Vectoring.h"

/* Invariant block signals (default storage) */
const ConstB_Torque_Vectoring_T Torque_Vectoring_ConstB = {
  480.0,                               /* '<S3>/Gain1' */
  5.88                                 /* '<S13>/Gain' */
};

/* Constant parameters (default storage) */
const ConstP_Torque_Vectoring_T Torque_Vectoring_ConstP = {
  /* Expression: Cij
   * Referenced by: '<S2>/LUT_Fz2Ca'
   */
  { 24909.74315, 48762.76026, 71392.32045, 92702.93068, 112627.3638, 131116.3455,
    148133.3975, 163651.2085 },

  /* Expression: Breakpoints_Fz
   * Referenced by: '<S2>/LUT_Fz2Ca'
   */
  { 1961.33, 3922.66, 5883.99, 7845.32, 9806.65, 11767.98, 13729.31, 15690.64 }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

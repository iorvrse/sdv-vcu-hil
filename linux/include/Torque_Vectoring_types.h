/*
 * File: Torque_Vectoring_types.h
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

#ifndef Torque_Vectoring_types_h_
#define Torque_Vectoring_types_h_
#include "rtwtypes.h"
#ifndef struct_tag_AhH5oYMN4rYqloefe9mTIC
#define struct_tag_AhH5oYMN4rYqloefe9mTIC

struct tag_AhH5oYMN4rYqloefe9mTIC
{
  int32_T isInitialized;
  boolean_T isSetupComplete;
  real_T pWinLen;
  real_T pBuf[3];
  real_T pHeap[3];
  real_T pMidHeap;
  real_T pIdx;
  real_T pPos[3];
  real_T pMinHeapLength;
  real_T pMaxHeapLength;
};

#endif                                 /* struct_tag_AhH5oYMN4rYqloefe9mTIC */

#ifndef typedef_e_dsp_internal_codegen_Median_T
#define typedef_e_dsp_internal_codegen_Median_T

typedef struct tag_AhH5oYMN4rYqloefe9mTIC e_dsp_internal_codegen_Median_T;

#endif                             /* typedef_e_dsp_internal_codegen_Median_T */

#ifndef struct_tag_BlgwLpgj2bjudmbmVKWwDE
#define struct_tag_BlgwLpgj2bjudmbmVKWwDE

struct tag_BlgwLpgj2bjudmbmVKWwDE
{
  uint32_T f1[8];
};

#endif                                 /* struct_tag_BlgwLpgj2bjudmbmVKWwDE */

#ifndef typedef_cell_wrap_Torque_Vectoring_T
#define typedef_cell_wrap_Torque_Vectoring_T

typedef struct tag_BlgwLpgj2bjudmbmVKWwDE cell_wrap_Torque_Vectoring_T;

#endif                                /* typedef_cell_wrap_Torque_Vectoring_T */

#ifndef struct_emxArray_boolean_T_8
#define struct_emxArray_boolean_T_8

struct emxArray_boolean_T_8
{
  boolean_T data[8];
  int32_T size;
};

#endif                                 /* struct_emxArray_boolean_T_8 */

#ifndef typedef_emxArray_boolean_T_8_Torque_V_T
#define typedef_emxArray_boolean_T_8_Torque_V_T

typedef struct emxArray_boolean_T_8 emxArray_boolean_T_8_Torque_V_T;

#endif                             /* typedef_emxArray_boolean_T_8_Torque_V_T */

#ifndef struct_tag_WD2Tt0dMKq3JKeNecCs1vH
#define struct_tag_WD2Tt0dMKq3JKeNecCs1vH

struct tag_WD2Tt0dMKq3JKeNecCs1vH
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  cell_wrap_Torque_Vectoring_T inputVarSize;
  int32_T NumChannels;
  e_dsp_internal_codegen_Median_T pMID;
};

#endif                                 /* struct_tag_WD2Tt0dMKq3JKeNecCs1vH */

#ifndef typedef_dsp_simulink_MedianFilter_Tor_T
#define typedef_dsp_simulink_MedianFilter_Tor_T

typedef struct tag_WD2Tt0dMKq3JKeNecCs1vH dsp_simulink_MedianFilter_Tor_T;

#endif                             /* typedef_dsp_simulink_MedianFilter_Tor_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_Torque_Vectoring_T RT_MODEL_Torque_Vectoring_T;

#endif                                 /* Torque_Vectoring_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

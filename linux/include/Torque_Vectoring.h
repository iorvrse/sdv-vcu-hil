/*
 * File: Torque_Vectoring.h
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

#ifndef Torque_Vectoring_h_
#define Torque_Vectoring_h_
#ifndef Torque_Vectoring_COMMON_INCLUDES_
#define Torque_Vectoring_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Torque_Vectoring_COMMON_INCLUDES_ */

#include "Torque_Vectoring_types.h"
#include <string.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real_T l[40];
  real_T b[16];
  real_T Linv[16];
  real_T RLinv[16];
  real_T D[16];
  real_T H[16];
  real_T U[16];
  real_T TL[16];
  real_T Q[16];
  real_T R[16];
  real_T lam[10];
  real_T Fx_max[10];
  real_T cTol[10];
  real_T Opt[8];
  real_T Rhs[8];
  int32_T iC[10];
  real_T Mx_coeff[4];
  real_T LUT_Fz2Ca[4];                 /* '<S2>/LUT_Fz2Ca' */
  real_T dv[4];
  real_T r[4];
  real_T tau[4];
  real_T work[4];
  boolean_T iA1_data[10];
  boolean_T b_iA[10];
  real_T Vx_ms;
  real_T vprev;
  real_T p;
  real_T Sum1;                         /* '<S5>/Sum1' */
  real_T FilterCoefficient;            /* '<S53>/Filter Coefficient' */
  real_T Gain2;                        /* '<S13>/Gain2' */
  real_T Divide_n;                     /* '<S8>/Divide' */
  real_T Divide1;                      /* '<S8>/Divide1' */
  real_T front;                        /* '<S2>/Add' */
  real_T rear;                         /* '<S2>/Add1' */
  real_T TSamp;                        /* '<S71>/TSamp' */
  real_T ind2;
  real_T d;
  real_T d1;
  real_T rMin;
  real_T Xnorm0;
  real_T cMin;
  real_T cVal;
  real_T t1;
  real_T rVal;
  real_T t;
  real_T z_idx_2;
  real_T z_idx_3;
  real_T z;
  real_T z_tmp;
  real_T xnorm;
  real_T d2;
  real_T RLinv_m;
  real_T RLinv_c;
  real_T RLinv_k;
  real_T scale;
  real_T absxk;
  real_T t_c;
  real_T temp;
  real_T c;
  int32_T jmax;
  int32_T idxAjj;
  int32_T d_j;
  int32_T iac;
  int32_T ia;
  int32_T e_k;
  int32_T b_c_tmp;
  int32_T b_tmp;
  int32_T iA1_size;
  int32_T nA;
  int32_T kDrop;
  int32_T iSave;
  int32_T i;
  int32_T iC_b;
} B_Torque_Vectoring_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  dsp_simulink_MedianFilter_Tor_T obj; /* '<S69>/Median Filter' */
  emxArray_boolean_T_8_Torque_V_T iA0_mem;/* '<S3>/MATLAB Function1' */
  real_T Integrator_DSTATE;            /* '<S50>/Integrator' */
  real_T Filter_DSTATE;                /* '<S45>/Filter' */
  real_T UD_DSTATE;                    /* '<S71>/UD' */
  real_T UD_DSTATE_l;                  /* '<S72>/UD' */
  real_T UD_DSTATE_j;                  /* '<S73>/UD' */
  boolean_T iA0_mem_not_empty;         /* '<S3>/MATLAB Function1' */
} DW_Torque_Vectoring_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real_T Gain1;                  /* '<S3>/Gain1' */
  const real_T Gain;                   /* '<S13>/Gain' */
} ConstB_Torque_Vectoring_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: Cij
   * Referenced by: '<S2>/LUT_Fz2Ca'
   */
  real_T LUT_Fz2Ca_tableData[8];

  /* Expression: Breakpoints_Fz
   * Referenced by: '<S2>/LUT_Fz2Ca'
   */
  real_T LUT_Fz2Ca_bp01Data[8];
} ConstP_Torque_Vectoring_T;

/* Real-time Model Data Structure */
struct tag_RTM_Torque_Vectoring_T {
  const char_T * volatile errorStatus;
  B_Torque_Vectoring_T *blockIO;
  DW_Torque_Vectoring_T *dwork;
};

extern const ConstB_Torque_Vectoring_T Torque_Vectoring_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_Torque_Vectoring_T Torque_Vectoring_ConstP;

/* Model entry point functions */
extern void Torque_Vectoring_initialize(RT_MODEL_Torque_Vectoring_T *const
  Torque_Vectoring_M, real_T *Torque_Vectoring_U_Vx_des, real_T
  *Torque_Vectoring_U_Vx, real_T *Torque_Vectoring_U_Vy, real_T
  Torque_Vectoring_U_AngWheel[4], real_T *Torque_Vectoring_U_r, real_T
  Torque_Vectoring_U_Fy[4], real_T Torque_Vectoring_U_Fz[4], real_T
  Torque_Vectoring_Y_Tm[4], real_T Torque_Vectoring_Y_Fx_opt[4], real_T
  *Torque_Vectoring_Y_Mx_total, real_T *Torque_Vectoring_Y_Fx_total, real_T
  *Torque_Vectoring_Y_Mzd, real_T *Torque_Vectoring_Y_r_des, real_T
  *Torque_Vectoring_Y_beta_des, real_T Torque_Vectoring_Y_Ca[2], real_T
  *Torque_Vectoring_Y_beta);
extern void Torque_Vectoring_step(RT_MODEL_Torque_Vectoring_T *const
  Torque_Vectoring_M, real_T Torque_Vectoring_U_Vx_des, real_T
  Torque_Vectoring_U_Vx, real_T Torque_Vectoring_U_Vy, real_T
  Torque_Vectoring_U_AngWheel[4], real_T Torque_Vectoring_U_r, real_T
  Torque_Vectoring_U_Fy[4], real_T Torque_Vectoring_U_Fz[4], real_T
  Torque_Vectoring_Y_Tm[4], real_T Torque_Vectoring_Y_Fx_opt[4], real_T
  *Torque_Vectoring_Y_Mx_total, real_T *Torque_Vectoring_Y_Fx_total, real_T
  *Torque_Vectoring_Y_Mzd, real_T *Torque_Vectoring_Y_r_des, real_T
  Torque_Vectoring_Y_Ca[2], real_T *Torque_Vectoring_Y_beta);
extern void Torque_Vectoring_terminate(RT_MODEL_Torque_Vectoring_T *const
  Torque_Vectoring_M);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S2>/Scope1' : Unused code path elimination
 * Block '<S3>/Display2' : Unused code path elimination
 * Block '<S3>/Scope' : Unused code path elimination
 * Block '<S3>/Scope1' : Unused code path elimination
 * Block '<S3>/Scope2' : Unused code path elimination
 * Block '<S3>/Scope3' : Unused code path elimination
 * Block '<S3>/Scope4' : Unused code path elimination
 * Block '<S3>/Sum of Elements' : Unused code path elimination
 * Block '<S4>/Gain5' : Unused code path elimination
 * Block '<S11>/Data Type Duplicate' : Unused code path elimination
 * Block '<S11>/Data Type Propagation' : Unused code path elimination
 * Block '<S12>/Data Type Duplicate' : Unused code path elimination
 * Block '<S12>/Data Type Propagation' : Unused code path elimination
 * Block '<S12>/LowerRelop1' : Unused code path elimination
 * Block '<S12>/Switch' : Unused code path elimination
 * Block '<S12>/Switch2' : Unused code path elimination
 * Block '<S12>/UpperRelop' : Unused code path elimination
 * Block '<S4>/Scope' : Unused code path elimination
 * Block '<S4>/Scope1' : Unused code path elimination
 * Block '<S4>/Scope2' : Unused code path elimination
 * Block '<S4>/Scope3' : Unused code path elimination
 * Block '<S13>/Atan' : Unused code path elimination
 * Block '<S13>/Gain4' : Unused code path elimination
 * Block '<S5>/Display' : Unused code path elimination
 * Block '<S5>/Display1' : Unused code path elimination
 * Block '<S5>/Scope' : Unused code path elimination
 * Block '<S5>/Scope1' : Unused code path elimination
 * Block '<S5>/Scope2' : Unused code path elimination
 * Block '<S68>/Scope' : Unused code path elimination
 * Block '<S68>/Scope1' : Unused code path elimination
 * Block '<S68>/Scope2' : Unused code path elimination
 * Block '<S71>/Data Type Duplicate' : Unused code path elimination
 * Block '<S72>/Data Type Duplicate' : Unused code path elimination
 * Block '<S73>/Data Type Duplicate' : Unused code path elimination
 * Block '<S69>/Scope' : Unused code path elimination
 * Block '<S69>/Scope1' : Unused code path elimination
 * Block '<S69>/Scope2' : Unused code path elimination
 * Block '<S69>/Scope3' : Unused code path elimination
 * Block '<S70>/Scope' : Unused code path elimination
 * Block '<S70>/Scope1' : Unused code path elimination
 * Block '<S70>/Scope2' : Unused code path elimination
 * Block '<S70>/Scope3' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Torque_Vectoring'
 * '<S1>'   : 'Torque_Vectoring/Torque Vectoring'
 * '<S2>'   : 'Torque_Vectoring/Torque Vectoring/Ca Calculations'
 * '<S3>'   : 'Torque_Vectoring/Torque Vectoring/Lower-Level Control'
 * '<S4>'   : 'Torque_Vectoring/Torque Vectoring/Reference Model'
 * '<S5>'   : 'Torque_Vectoring/Torque Vectoring/Speed Control'
 * '<S6>'   : 'Torque_Vectoring/Torque Vectoring/Upper-Level Control'
 * '<S7>'   : 'Torque_Vectoring/Torque Vectoring/beta Calculations'
 * '<S8>'   : 'Torque_Vectoring/Torque Vectoring/delta_in Calculations'
 * '<S9>'   : 'Torque_Vectoring/Torque Vectoring/Lower-Level Control/MATLAB Function1'
 * '<S10>'  : 'Torque_Vectoring/Torque Vectoring/Reference Model/Reference Model'
 * '<S11>'  : 'Torque_Vectoring/Torque Vectoring/Reference Model/Saturation Dynamic'
 * '<S12>'  : 'Torque_Vectoring/Torque Vectoring/Reference Model/Saturation Dynamic1'
 * '<S13>'  : 'Torque_Vectoring/Torque Vectoring/Reference Model/r&beta Limiter'
 * '<S14>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/MATLAB Function'
 * '<S15>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4'
 * '<S16>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Anti-windup'
 * '<S17>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/D Gain'
 * '<S18>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/External Derivative'
 * '<S19>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Filter'
 * '<S20>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Filter ICs'
 * '<S21>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/I Gain'
 * '<S22>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Ideal P Gain'
 * '<S23>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Ideal P Gain Fdbk'
 * '<S24>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Integrator'
 * '<S25>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Integrator ICs'
 * '<S26>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/N Copy'
 * '<S27>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/N Gain'
 * '<S28>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/P Copy'
 * '<S29>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Parallel P Gain'
 * '<S30>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Reset Signal'
 * '<S31>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Saturation'
 * '<S32>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Saturation Fdbk'
 * '<S33>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Sum'
 * '<S34>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Sum Fdbk'
 * '<S35>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Tracking Mode'
 * '<S36>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Tracking Mode Sum'
 * '<S37>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Tsamp - Integral'
 * '<S38>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Tsamp - Ngain'
 * '<S39>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/postSat Signal'
 * '<S40>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/preInt Signal'
 * '<S41>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/preSat Signal'
 * '<S42>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Anti-windup/Passthrough'
 * '<S43>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/D Gain/Internal Parameters'
 * '<S44>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/External Derivative/Error'
 * '<S45>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Filter/Disc. Forward Euler Filter'
 * '<S46>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Filter ICs/Internal IC - Filter'
 * '<S47>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/I Gain/Internal Parameters'
 * '<S48>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Ideal P Gain/Passthrough'
 * '<S49>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Ideal P Gain Fdbk/Disabled'
 * '<S50>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Integrator/Discrete'
 * '<S51>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Integrator ICs/Internal IC'
 * '<S52>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/N Copy/Disabled'
 * '<S53>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/N Gain/Internal Parameters'
 * '<S54>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/P Copy/Disabled'
 * '<S55>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Parallel P Gain/Internal Parameters'
 * '<S56>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Reset Signal/Disabled'
 * '<S57>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Saturation/Passthrough'
 * '<S58>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Saturation Fdbk/Disabled'
 * '<S59>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Sum/Sum_PID'
 * '<S60>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Sum Fdbk/Disabled'
 * '<S61>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Tracking Mode/Disabled'
 * '<S62>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Tracking Mode Sum/Passthrough'
 * '<S63>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Tsamp - Integral/TsSignalSpecification'
 * '<S64>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/Tsamp - Ngain/Passthrough'
 * '<S65>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/postSat Signal/Forward_Path'
 * '<S66>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/preInt Signal/Internal PreInt'
 * '<S67>'  : 'Torque_Vectoring/Torque Vectoring/Speed Control/PID Controller4/preSat Signal/Forward_Path'
 * '<S68>'  : 'Torque_Vectoring/Torque Vectoring/Upper-Level Control/Reaching Law (sat)'
 * '<S69>'  : 'Torque_Vectoring/Torque Vectoring/Upper-Level Control/r_dot Calculation'
 * '<S70>'  : 'Torque_Vectoring/Torque Vectoring/Upper-Level Control/s Calculation'
 * '<S71>'  : 'Torque_Vectoring/Torque Vectoring/Upper-Level Control/r_dot Calculation/Discrete Derivative'
 * '<S72>'  : 'Torque_Vectoring/Torque Vectoring/Upper-Level Control/r_dot Calculation/Discrete Derivative1'
 * '<S73>'  : 'Torque_Vectoring/Torque Vectoring/Upper-Level Control/r_dot Calculation/Discrete Derivative2'
 */
#endif                                 /* Torque_Vectoring_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

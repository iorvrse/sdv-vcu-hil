/*
 * File: Torque_Vectoring.c
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
#include "rtwtypes.h"
#include "Torque_Vectoring_types.h"
#include <math.h>
#include <string.h>
#include "Torque_Vectoring_private.h"

/* Forward declaration for local functions */
static void Torque_V_MedianFilter_resetImpl(e_dsp_internal_codegen_Median_T *obj);
static void Tor_MedianFilter_trickleDownMax(e_dsp_internal_codegen_Median_T *obj,
  real_T i);
static void Tor_MedianFilter_trickleDownMin(e_dsp_internal_codegen_Median_T *obj,
  real_T i);
static void Torque_Vectoring_trisolve(const real_T A[16], real_T B[16]);
static real_T Torque_Vectoring_norm(const real_T x[4]);
static real_T Torque_Vectoring_xnrm2(int32_T n, const real_T x[16], int32_T ix0,
  B_Torque_Vectoring_T *Torque_Vectoring_B);
static void Torque_Vectoring_xgemv(int32_T m, int32_T n, const real_T A[16],
  int32_T ia0, const real_T x[16], int32_T ix0, real_T y[4],
  B_Torque_Vectoring_T *Torque_Vectoring_B);
static void Torque_Vectoring_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T
  ix0, const real_T y[4], real_T A[16], int32_T ia0, B_Torque_Vectoring_T
  *Torque_Vectoring_B);
static real_T Torque_Vectoring_KWIKfactor(const real_T Ac[40], const int32_T iC
  [10], int32_T nA, const real_T Linv[16], real_T RLinv[16], real_T D[16],
  real_T H[16], B_Torque_Vectoring_T *Torque_Vectoring_B);
static void Torque_Vectoring_DropConstraint(int32_T kDrop, boolean_T iA_data[],
  int32_T *nA, int32_T iC[10]);
static void Torque_Vectori_ResetToColdStart(boolean_T iA[10], int32_T iC[10]);
static void Torque_Vectoring_qpkwik(const real_T Linv[16], const real_T Hinv[16],
  const real_T f[4], const real_T Ac[40], const real_T b[10], boolean_T iA_data[],
  int32_T *iA_size, real_T x[4], real_T lambda[10], int32_T *status,
  B_Torque_Vectoring_T *Torque_Vectoring_B);
real_T look1_binlxpw(real_T u0, const real_T bp0[], const real_T table[],
                     uint32_T maxIndex)
{
  real_T frac;
  real_T yL_0d0;
  uint32_T iLeft;

  /* Column-major Lookup 1-D
     Search method: 'binary'
     Use previous index: 'off'
     Interpolation method: 'Linear point-slope'
     Extrapolation method: 'Linear'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Linear'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    uint32_T bpIdx;
    uint32_T iRght;

    /* Binary Search */
    bpIdx = maxIndex >> 1U;
    iLeft = 0U;
    iRght = maxIndex;
    while (iRght - iLeft > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (iRght + iLeft) >> 1U;
    }

    frac = (u0 - bp0[iLeft]) / (bp0[iLeft + 1U] - bp0[iLeft]);
  } else {
    iLeft = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[iLeft];
  return (table[iLeft + 1U] - yL_0d0) * frac + yL_0d0;
}

static void Torque_V_MedianFilter_resetImpl(e_dsp_internal_codegen_Median_T *obj)
{
  /* Start for MATLABSystem: '<S69>/Median Filter' */
  obj->pBuf[0] = 0.0;
  obj->pPos[0] = 0.0;
  obj->pHeap[0] = 0.0;
  obj->pBuf[1] = 0.0;
  obj->pPos[1] = 0.0;
  obj->pHeap[1] = 0.0;
  obj->pBuf[2] = 0.0;
  obj->pPos[2] = 0.0;
  obj->pHeap[2] = 0.0;
  obj->pWinLen = 3.0;
  obj->pIdx = obj->pWinLen;

  /* Start for MATLABSystem: '<S69>/Median Filter' */
  obj->pMidHeap = ceil((obj->pWinLen + 1.0) / 2.0);
  obj->pMinHeapLength = trunc((obj->pWinLen - 1.0) / 2.0);
  obj->pMaxHeapLength = trunc(obj->pWinLen / 2.0);
  obj->pPos[2] = obj->pWinLen;
  obj->pHeap[(int32_T)obj->pPos[2] - 1] = 3.0;
  obj->pPos[1] = 1.0;
  obj->pHeap[(int32_T)obj->pPos[1] - 1] = 2.0;
  obj->pPos[0] = obj->pWinLen - 1.0;
  obj->pHeap[(int32_T)obj->pPos[0] - 1] = 1.0;
}

static void Tor_MedianFilter_trickleDownMax(e_dsp_internal_codegen_Median_T *obj,
  real_T i)
{
  boolean_T exitg1;
  exitg1 = false;
  while ((!exitg1) && (i >= -obj->pMaxHeapLength)) {
    real_T ind1;
    real_T ind2;
    real_T tmp;
    real_T tmp_0;
    if ((i < -1.0) && (i > -obj->pMaxHeapLength) && (obj->pBuf[(int32_T)
         obj->pHeap[(int32_T)(i + obj->pMidHeap) - 1] - 1] < obj->pBuf[(int32_T)
         obj->pHeap[(int32_T)((i - 1.0) + obj->pMidHeap) - 1] - 1])) {
      i--;
    }

    ind1 = trunc(i / 2.0) + obj->pMidHeap;
    ind2 = i + obj->pMidHeap;
    tmp = obj->pHeap[(int32_T)ind1 - 1];
    tmp_0 = obj->pHeap[(int32_T)ind2 - 1];
    if (obj->pBuf[(int32_T)tmp - 1] >= obj->pBuf[(int32_T)tmp_0 - 1]) {
      exitg1 = true;
    } else {
      obj->pHeap[(int32_T)ind1 - 1] = tmp_0;
      obj->pHeap[(int32_T)ind2 - 1] = tmp;
      obj->pPos[(int32_T)obj->pHeap[(int32_T)ind1 - 1] - 1] = ind1;
      obj->pPos[(int32_T)obj->pHeap[(int32_T)ind2 - 1] - 1] = ind2;
      i *= 2.0;
    }
  }
}

static void Tor_MedianFilter_trickleDownMin(e_dsp_internal_codegen_Median_T *obj,
  real_T i)
{
  boolean_T exitg1;
  exitg1 = false;
  while ((!exitg1) && (i <= obj->pMinHeapLength)) {
    real_T ind1;
    real_T ind2;
    real_T tmp;
    real_T tmp_0;
    if ((i > 1.0) && (i < obj->pMinHeapLength) && (obj->pBuf[(int32_T)obj->
         pHeap[(int32_T)((i + 1.0) + obj->pMidHeap) - 1] - 1] < obj->pBuf
         [(int32_T)obj->pHeap[(int32_T)(i + obj->pMidHeap) - 1] - 1])) {
      i++;
    }

    ind1 = i + obj->pMidHeap;
    ind2 = trunc(i / 2.0) + obj->pMidHeap;
    tmp = obj->pHeap[(int32_T)ind1 - 1];
    tmp_0 = obj->pHeap[(int32_T)ind2 - 1];
    if (obj->pBuf[(int32_T)tmp - 1] >= obj->pBuf[(int32_T)tmp_0 - 1]) {
      exitg1 = true;
    } else {
      obj->pHeap[(int32_T)ind1 - 1] = tmp_0;
      obj->pHeap[(int32_T)ind2 - 1] = tmp;
      obj->pPos[(int32_T)obj->pHeap[(int32_T)ind1 - 1] - 1] = ind1;
      obj->pPos[(int32_T)obj->pHeap[(int32_T)ind2 - 1] - 1] = ind2;
      i *= 2.0;
    }
  }
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static void Torque_Vectoring_trisolve(const real_T A[16], real_T B[16])
{
  int32_T b_k;
  int32_T i;
  int32_T j;
  for (j = 0; j < 4; j++) {
    int32_T jBcol;
    jBcol = j << 2;
    for (b_k = 0; b_k < 4; b_k++) {
      real_T B_0;
      int32_T B_tmp;
      int32_T kAcol;
      kAcol = b_k << 2;
      B_tmp = b_k + jBcol;
      B_0 = B[B_tmp];
      if (B_0 != 0.0) {
        B[B_tmp] = B_0 / A[b_k + kAcol];
        for (i = b_k + 2; i < 5; i++) {
          int32_T tmp;
          tmp = (i + jBcol) - 1;
          B[tmp] -= A[(i + kAcol) - 1] * B[B_tmp];
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static real_T Torque_Vectoring_norm(const real_T x[4])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * sqrt(y);
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static real_T Torque_Vectoring_xnrm2(int32_T n, const real_T x[16], int32_T ix0,
  B_Torque_Vectoring_T *Torque_Vectoring_B)
{
  real_T y;
  int32_T k;
  int32_T kend;
  y = 0.0;
  if (n >= 1) {
    if (n == 1) {
      y = fabs(x[ix0 - 1]);
    } else {
      Torque_Vectoring_B->scale = 3.3121686421112381E-170;
      kend = ix0 + n;
      for (k = ix0; k < kend; k++) {
        Torque_Vectoring_B->absxk = fabs(x[k - 1]);
        if (Torque_Vectoring_B->absxk > Torque_Vectoring_B->scale) {
          Torque_Vectoring_B->t_c = Torque_Vectoring_B->scale /
            Torque_Vectoring_B->absxk;
          y = y * Torque_Vectoring_B->t_c * Torque_Vectoring_B->t_c + 1.0;
          Torque_Vectoring_B->scale = Torque_Vectoring_B->absxk;
        } else {
          Torque_Vectoring_B->t_c = Torque_Vectoring_B->absxk /
            Torque_Vectoring_B->scale;
          y += Torque_Vectoring_B->t_c * Torque_Vectoring_B->t_c;
        }
      }

      y = Torque_Vectoring_B->scale * sqrt(y);
    }
  }

  return y;
}

real_T rt_hypotd(real_T u0, real_T u1)
{
  real_T a;
  real_T b;
  real_T y;
  a = fabs(u0);
  b = fabs(u1);
  if (a < b) {
    a /= b;
    y = sqrt(a * a + 1.0) * b;
  } else if (a > b) {
    b /= a;
    y = sqrt(b * b + 1.0) * a;
  } else {
    y = a * 1.4142135623730951;
  }

  return y;
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static void Torque_Vectoring_xgemv(int32_T m, int32_T n, const real_T A[16],
  int32_T ia0, const real_T x[16], int32_T ix0, real_T y[4],
  B_Torque_Vectoring_T *Torque_Vectoring_B)
{
  int32_T b;
  int32_T b_iy;
  int32_T e;
  int32_T ia;
  if (n != 0) {
    memset(&y[0], 0, (uint8_T)n * sizeof(real_T));
    b = ((n - 1) << 2) + ia0;
    for (b_iy = ia0; b_iy <= b; b_iy += 4) {
      Torque_Vectoring_B->c = 0.0;
      e = b_iy + m;
      for (ia = b_iy; ia < e; ia++) {
        Torque_Vectoring_B->c += x[((ix0 + ia) - b_iy) - 1] * A[ia - 1];
      }

      ia = (b_iy - ia0) >> 2;
      y[ia] += Torque_Vectoring_B->c;
    }
  }
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static void Torque_Vectoring_xgerc(int32_T m, int32_T n, real_T alpha1, int32_T
  ix0, const real_T y[4], real_T A[16], int32_T ia0, B_Torque_Vectoring_T
  *Torque_Vectoring_B)
{
  int32_T b;
  int32_T c;
  int32_T ijA;
  int32_T j;
  int32_T jA;
  if (alpha1 != 0.0) {
    jA = ia0;
    b = (uint8_T)n;
    for (j = 0; j < b; j++) {
      Torque_Vectoring_B->temp = y[j];
      if (Torque_Vectoring_B->temp != 0.0) {
        Torque_Vectoring_B->temp *= alpha1;
        c = m + jA;
        for (ijA = jA; ijA < c; ijA++) {
          A[ijA - 1] += A[((ix0 + ijA) - jA) - 1] * Torque_Vectoring_B->temp;
        }
      }

      jA += 4;
    }
  }
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static real_T Torque_Vectoring_KWIKfactor(const real_T Ac[40], const int32_T iC
  [10], int32_T nA, const real_T Linv[16], real_T RLinv[16], real_T D[16],
  real_T H[16], B_Torque_Vectoring_T *Torque_Vectoring_B)
{
  real_T Status;
  int32_T b;
  int32_T b_lastv;
  int32_T c;
  int32_T c_lastc;
  int32_T exitg1;
  int32_T ii;
  int32_T j_i;
  int32_T knt;
  int32_T l;
  boolean_T exitg2;
  Status = 1.0;
  memset(&RLinv[0], 0, sizeof(real_T) << 4U);
  b = (uint8_T)nA;
  for (ii = 0; ii < b; ii++) {
    b_lastv = iC[ii];
    Torque_Vectoring_B->xnorm = 0.0;
    Torque_Vectoring_B->RLinv_m = 0.0;
    Torque_Vectoring_B->RLinv_c = 0.0;
    Torque_Vectoring_B->RLinv_k = 0.0;
    for (j_i = 0; j_i < 4; j_i++) {
      Torque_Vectoring_B->d2 = Ac[(10 * j_i + b_lastv) - 1];
      c_lastc = j_i << 2;
      Torque_Vectoring_B->xnorm += Linv[c_lastc] * Torque_Vectoring_B->d2;
      Torque_Vectoring_B->RLinv_m += Linv[c_lastc + 1] * Torque_Vectoring_B->d2;
      Torque_Vectoring_B->RLinv_c += Linv[c_lastc + 2] * Torque_Vectoring_B->d2;
      Torque_Vectoring_B->RLinv_k += Linv[c_lastc + 3] * Torque_Vectoring_B->d2;
    }

    j_i = ii << 2;
    RLinv[j_i + 3] = Torque_Vectoring_B->RLinv_k;
    RLinv[j_i + 2] = Torque_Vectoring_B->RLinv_c;
    RLinv[j_i + 1] = Torque_Vectoring_B->RLinv_m;
    RLinv[j_i] = Torque_Vectoring_B->xnorm;
  }

  memcpy(&Torque_Vectoring_B->TL[0], &RLinv[0], sizeof(real_T) << 4U);
  Torque_Vectoring_B->tau[0] = 0.0;
  Torque_Vectoring_B->work[0] = 0.0;
  Torque_Vectoring_B->tau[1] = 0.0;
  Torque_Vectoring_B->work[1] = 0.0;
  Torque_Vectoring_B->tau[2] = 0.0;
  Torque_Vectoring_B->work[2] = 0.0;
  Torque_Vectoring_B->tau[3] = 0.0;
  Torque_Vectoring_B->work[3] = 0.0;
  for (j_i = 0; j_i < 4; j_i++) {
    ii = (j_i << 2) + j_i;
    if (j_i + 1 < 4) {
      Torque_Vectoring_B->RLinv_m = Torque_Vectoring_B->TL[ii];
      b_lastv = ii + 2;
      Torque_Vectoring_B->tau[j_i] = 0.0;
      Torque_Vectoring_B->xnorm = Torque_Vectoring_xnrm2(3 - j_i,
        Torque_Vectoring_B->TL, ii + 2, Torque_Vectoring_B);
      if (Torque_Vectoring_B->xnorm != 0.0) {
        Torque_Vectoring_B->RLinv_c = Torque_Vectoring_B->TL[ii];
        Torque_Vectoring_B->xnorm = rt_hypotd(Torque_Vectoring_B->RLinv_c,
          Torque_Vectoring_B->xnorm);
        if (Torque_Vectoring_B->RLinv_c >= 0.0) {
          Torque_Vectoring_B->xnorm = -Torque_Vectoring_B->xnorm;
        }

        if (fabs(Torque_Vectoring_B->xnorm) < 1.0020841800044864E-292) {
          knt = 0;
          l = (ii - j_i) + 4;
          do {
            knt++;
            for (c_lastc = b_lastv; c_lastc <= l; c_lastc++) {
              Torque_Vectoring_B->TL[c_lastc - 1] *= 9.9792015476736E+291;
            }

            Torque_Vectoring_B->xnorm *= 9.9792015476736E+291;
            Torque_Vectoring_B->RLinv_m *= 9.9792015476736E+291;
          } while ((fabs(Torque_Vectoring_B->xnorm) < 1.0020841800044864E-292) &&
                   (knt < 20));

          Torque_Vectoring_B->xnorm = rt_hypotd(Torque_Vectoring_B->RLinv_m,
            Torque_Vectoring_xnrm2(3 - j_i, Torque_Vectoring_B->TL, ii + 2,
            Torque_Vectoring_B));
          if (Torque_Vectoring_B->RLinv_m >= 0.0) {
            Torque_Vectoring_B->xnorm = -Torque_Vectoring_B->xnorm;
          }

          Torque_Vectoring_B->tau[j_i] = (Torque_Vectoring_B->xnorm -
            Torque_Vectoring_B->RLinv_m) / Torque_Vectoring_B->xnorm;
          Torque_Vectoring_B->RLinv_m = 1.0 / (Torque_Vectoring_B->RLinv_m -
            Torque_Vectoring_B->xnorm);
          for (c_lastc = b_lastv; c_lastc <= l; c_lastc++) {
            Torque_Vectoring_B->TL[c_lastc - 1] *= Torque_Vectoring_B->RLinv_m;
          }

          for (b_lastv = 0; b_lastv < knt; b_lastv++) {
            Torque_Vectoring_B->xnorm *= 1.0020841800044864E-292;
          }

          Torque_Vectoring_B->RLinv_m = Torque_Vectoring_B->xnorm;
        } else {
          Torque_Vectoring_B->tau[j_i] = (Torque_Vectoring_B->xnorm -
            Torque_Vectoring_B->RLinv_c) / Torque_Vectoring_B->xnorm;
          Torque_Vectoring_B->RLinv_m = 1.0 / (Torque_Vectoring_B->RLinv_c -
            Torque_Vectoring_B->xnorm);
          knt = (ii - j_i) + 4;
          for (c_lastc = b_lastv; c_lastc <= knt; c_lastc++) {
            Torque_Vectoring_B->TL[c_lastc - 1] *= Torque_Vectoring_B->RLinv_m;
          }

          Torque_Vectoring_B->RLinv_m = Torque_Vectoring_B->xnorm;
        }
      }

      Torque_Vectoring_B->TL[ii] = 1.0;
      if (Torque_Vectoring_B->tau[j_i] != 0.0) {
        b_lastv = 4 - j_i;
        c_lastc = (ii - j_i) + 3;
        while ((b_lastv > 0) && (Torque_Vectoring_B->TL[c_lastc] == 0.0)) {
          b_lastv--;
          c_lastc--;
        }

        c_lastc = 3 - j_i;
        exitg2 = false;
        while ((!exitg2) && (c_lastc > 0)) {
          knt = (((c_lastc - 1) << 2) + ii) + 4;
          l = knt;
          do {
            exitg1 = 0;
            if (l + 1 <= knt + b_lastv) {
              if (Torque_Vectoring_B->TL[l] != 0.0) {
                exitg1 = 1;
              } else {
                l++;
              }
            } else {
              c_lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        c_lastc = 0;
      }

      if (b_lastv > 0) {
        Torque_Vectoring_xgemv(b_lastv, c_lastc, Torque_Vectoring_B->TL, ii + 5,
          Torque_Vectoring_B->TL, ii + 1, Torque_Vectoring_B->work,
          Torque_Vectoring_B);
        Torque_Vectoring_xgerc(b_lastv, c_lastc, -Torque_Vectoring_B->tau[j_i],
          ii + 1, Torque_Vectoring_B->work, Torque_Vectoring_B->TL, ii + 5,
          Torque_Vectoring_B);
      }

      Torque_Vectoring_B->TL[ii] = Torque_Vectoring_B->RLinv_m;
    } else {
      Torque_Vectoring_B->tau[3] = 0.0;
    }
  }

  for (j_i = 0; j_i < 4; j_i++) {
    for (ii = 0; ii <= j_i; ii++) {
      b_lastv = j_i << 2;
      Torque_Vectoring_B->R[ii + b_lastv] = Torque_Vectoring_B->TL[b_lastv + ii];
    }

    for (ii = j_i + 2; ii < 5; ii++) {
      Torque_Vectoring_B->R[(ii + (j_i << 2)) - 1] = 0.0;
    }

    Torque_Vectoring_B->work[j_i] = 0.0;
  }

  for (j_i = 3; j_i >= 0; j_i--) {
    ii = (j_i << 2) + j_i;
    if (j_i + 1 < 4) {
      Torque_Vectoring_B->TL[ii] = 1.0;
      if (Torque_Vectoring_B->tau[j_i] != 0.0) {
        b_lastv = 4 - j_i;
        c_lastc = (ii - j_i) + 3;
        while ((b_lastv > 0) && (Torque_Vectoring_B->TL[c_lastc] == 0.0)) {
          b_lastv--;
          c_lastc--;
        }

        c_lastc = 3 - j_i;
        exitg2 = false;
        while ((!exitg2) && (c_lastc > 0)) {
          knt = (((c_lastc - 1) << 2) + ii) + 4;
          l = knt;
          do {
            exitg1 = 0;
            if (l + 1 <= knt + b_lastv) {
              if (Torque_Vectoring_B->TL[l] != 0.0) {
                exitg1 = 1;
              } else {
                l++;
              }
            } else {
              c_lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);

          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        b_lastv = 0;
        c_lastc = 0;
      }

      if (b_lastv > 0) {
        Torque_Vectoring_xgemv(b_lastv, c_lastc, Torque_Vectoring_B->TL, ii + 5,
          Torque_Vectoring_B->TL, ii + 1, Torque_Vectoring_B->work,
          Torque_Vectoring_B);
        Torque_Vectoring_xgerc(b_lastv, c_lastc, -Torque_Vectoring_B->tau[j_i],
          ii + 1, Torque_Vectoring_B->work, Torque_Vectoring_B->TL, ii + 5,
          Torque_Vectoring_B);
      }

      c_lastc = (ii - j_i) + 4;
      for (b_lastv = ii + 2; b_lastv <= c_lastc; b_lastv++) {
        Torque_Vectoring_B->TL[b_lastv - 1] *= -Torque_Vectoring_B->tau[j_i];
      }
    }

    Torque_Vectoring_B->TL[ii] = 1.0 - Torque_Vectoring_B->tau[j_i];
    for (b_lastv = 0; b_lastv < j_i; b_lastv++) {
      Torque_Vectoring_B->TL[(ii - b_lastv) - 1] = 0.0;
    }
  }

  for (j_i = 0; j_i < 4; j_i++) {
    ii = j_i << 2;
    Torque_Vectoring_B->Q[ii] = Torque_Vectoring_B->TL[ii];
    Torque_Vectoring_B->Q[ii + 1] = Torque_Vectoring_B->TL[ii + 1];
    Torque_Vectoring_B->Q[ii + 2] = Torque_Vectoring_B->TL[ii + 2];
    Torque_Vectoring_B->Q[ii + 3] = Torque_Vectoring_B->TL[ii + 3];
  }

  j_i = 0;
  do {
    exitg1 = 0;
    if (j_i <= (uint8_T)nA - 1) {
      if (fabs(Torque_Vectoring_B->R[(j_i << 2) + j_i]) < 1.0E-12) {
        Status = -2.0;
        exitg1 = 1;
      } else {
        j_i++;
      }
    } else {
      for (j_i = 0; j_i < 4; j_i++) {
        ii = j_i << 2;
        Torque_Vectoring_B->xnorm = Linv[ii + 1];
        Torque_Vectoring_B->RLinv_m = Linv[ii];
        Torque_Vectoring_B->RLinv_c = Linv[ii + 2];
        Torque_Vectoring_B->RLinv_k = Linv[ii + 3];
        for (ii = 0; ii < 4; ii++) {
          b_lastv = ii << 2;
          Torque_Vectoring_B->TL[j_i + b_lastv] = ((Torque_Vectoring_B->
            Q[b_lastv + 1] * Torque_Vectoring_B->xnorm +
            Torque_Vectoring_B->RLinv_m * Torque_Vectoring_B->Q[b_lastv]) +
            Torque_Vectoring_B->Q[b_lastv + 2] * Torque_Vectoring_B->RLinv_c) +
            Torque_Vectoring_B->Q[b_lastv + 3] * Torque_Vectoring_B->RLinv_k;
        }
      }

      memset(&RLinv[0], 0, sizeof(real_T) << 4U);
      for (b_lastv = nA; b_lastv >= 1; b_lastv--) {
        j_i = (b_lastv - 1) << 2;
        ii = (b_lastv + j_i) - 1;
        RLinv[ii] = 1.0;
        for (c_lastc = b_lastv; c_lastc <= nA; c_lastc++) {
          l = (((c_lastc - 1) << 2) + b_lastv) - 1;
          RLinv[l] /= Torque_Vectoring_B->R[ii];
        }

        if (b_lastv > 1) {
          c = (uint8_T)(b_lastv - 1);
          for (c_lastc = 0; c_lastc < c; c_lastc++) {
            for (knt = b_lastv; knt <= nA; knt++) {
              ii = (knt - 1) << 2;
              l = ii + c_lastc;
              RLinv[l] -= RLinv[(ii + b_lastv) - 1] * Torque_Vectoring_B->R[j_i
                + c_lastc];
            }
          }
        }
      }

      if (nA > 2147483646) {
        l = MAX_int32_T;
      } else {
        l = nA + 1;
      }

      for (b_lastv = 0; b_lastv < 4; b_lastv++) {
        for (c_lastc = b_lastv + 1; c_lastc < 5; c_lastc++) {
          j_i = ((c_lastc - 1) << 2) + b_lastv;
          H[j_i] = 0.0;
          for (knt = l; knt < 5; knt++) {
            ii = (knt - 1) << 2;
            H[j_i] -= Torque_Vectoring_B->TL[(ii + c_lastc) - 1] *
              Torque_Vectoring_B->TL[ii + b_lastv];
          }

          H[(c_lastc + (b_lastv << 2)) - 1] = H[j_i];
        }
      }

      for (b_lastv = 0; b_lastv < b; b_lastv++) {
        for (c_lastc = 0; c_lastc < 4; c_lastc++) {
          j_i = (b_lastv << 2) + c_lastc;
          D[j_i] = 0.0;
          for (knt = b_lastv + 1; knt <= nA; knt++) {
            ii = (knt - 1) << 2;
            D[j_i] += Torque_Vectoring_B->TL[ii + c_lastc] * RLinv[ii + b_lastv];
          }
        }
      }

      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return Status;
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static void Torque_Vectoring_DropConstraint(int32_T kDrop, boolean_T iA_data[],
  int32_T *nA, int32_T iC[10])
{
  int32_T i;
  if (kDrop > 0) {
    iA_data[iC[kDrop - 1] - 1] = false;
    if (kDrop < *nA) {
      int32_T b;
      if (*nA < -2147483647) {
        i = MIN_int32_T;
      } else {
        i = *nA - 1;
      }

      b = i + 1;
      for (i = kDrop; i < b; i++) {
        iC[i - 1] = iC[i];
      }
    }

    iC[*nA - 1] = 0;
    if (*nA < -2147483647) {
      *nA = MIN_int32_T;
    } else {
      (*nA)--;
    }
  }
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static void Torque_Vectori_ResetToColdStart(boolean_T iA[10], int32_T iC[10])
{
  int32_T i;
  for (i = 0; i < 10; i++) {
    iA[i] = false;
    iC[i] = 0;
  }

  iA[8] = true;
  iC[0] = 9;
  iA[9] = true;
  iC[1] = 10;
}

/* Function for MATLAB Function: '<S3>/MATLAB Function1' */
static void Torque_Vectoring_qpkwik(const real_T Linv[16], const real_T Hinv[16],
  const real_T f[4], const real_T Ac[40], const real_T b[10], boolean_T iA_data[],
  int32_T *iA_size, real_T x[4], real_T lambda[10], int32_T *status,
  B_Torque_Vectoring_T *Torque_Vectoring_B)
{
  int32_T U_tmp;
  int32_T exitg1;
  int32_T exitg3;
  int32_T z_idx_0_tmp;
  boolean_T ColdReset;
  boolean_T DualFeasible;
  boolean_T cTolComputed;
  boolean_T exitg2;
  boolean_T exitg4;
  boolean_T guard1;
  boolean_T guard2;
  x[0] = 0.0;
  x[1] = 0.0;
  x[2] = 0.0;
  x[3] = 0.0;
  *status = 1;
  Torque_Vectoring_B->r[0] = 0.0;
  Torque_Vectoring_B->r[1] = 0.0;
  Torque_Vectoring_B->r[2] = 0.0;
  Torque_Vectoring_B->r[3] = 0.0;
  Torque_Vectoring_B->rMin = 0.0;
  cTolComputed = false;
  for (Torque_Vectoring_B->i = 0; Torque_Vectoring_B->i < 10;
       Torque_Vectoring_B->i++) {
    lambda[Torque_Vectoring_B->i] = 0.0;
    Torque_Vectoring_B->cTol[Torque_Vectoring_B->i] = 1.0;
    Torque_Vectoring_B->iC[Torque_Vectoring_B->i] = 0;
  }

  Torque_Vectoring_B->nA = 0;
  for (Torque_Vectoring_B->i = 0; Torque_Vectoring_B->i < 10;
       Torque_Vectoring_B->i++) {
    if (iA_data[Torque_Vectoring_B->i]) {
      Torque_Vectoring_B->nA++;
      Torque_Vectoring_B->iC[Torque_Vectoring_B->nA - 1] = Torque_Vectoring_B->i
        + 1;
    }
  }

  guard1 = false;
  if (Torque_Vectoring_B->nA > 0) {
    memset(&Torque_Vectoring_B->Opt[0], 0, sizeof(real_T) << 3U);
    Torque_Vectoring_B->Rhs[0] = f[0];
    Torque_Vectoring_B->Rhs[4] = 0.0;
    Torque_Vectoring_B->Rhs[1] = f[1];
    Torque_Vectoring_B->Rhs[5] = 0.0;
    Torque_Vectoring_B->Rhs[2] = f[2];
    Torque_Vectoring_B->Rhs[6] = 0.0;
    Torque_Vectoring_B->Rhs[3] = f[3];
    Torque_Vectoring_B->Rhs[7] = 0.0;
    DualFeasible = false;
    ColdReset = false;
    do {
      exitg3 = 0;
      if ((!DualFeasible) && (Torque_Vectoring_B->nA > 0) && (*status <= 10)) {
        Torque_Vectoring_B->Xnorm0 = Torque_Vectoring_KWIKfactor(Ac,
          Torque_Vectoring_B->iC, Torque_Vectoring_B->nA, Linv,
          Torque_Vectoring_B->RLinv, Torque_Vectoring_B->D,
          Torque_Vectoring_B->H, Torque_Vectoring_B);
        if (Torque_Vectoring_B->Xnorm0 < 0.0) {
          if (ColdReset) {
            *status = -2;
            exitg3 = 2;
          } else {
            Torque_Vectori_ResetToColdStart(Torque_Vectoring_B->b_iA,
              Torque_Vectoring_B->iC);
            Torque_Vectoring_B->nA = 2;
            *iA_size = 10;
            for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop < 10;
                 Torque_Vectoring_B->kDrop++) {
              iA_data[Torque_Vectoring_B->kDrop] = Torque_Vectoring_B->
                b_iA[Torque_Vectoring_B->kDrop];
            }

            ColdReset = true;
          }
        } else {
          Torque_Vectoring_B->i = (uint8_T)Torque_Vectoring_B->nA;
          for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop <
               Torque_Vectoring_B->i; Torque_Vectoring_B->kDrop++) {
            Torque_Vectoring_B->Rhs[Torque_Vectoring_B->kDrop + 4] =
              b[Torque_Vectoring_B->iC[Torque_Vectoring_B->kDrop] - 1];
            for (z_idx_0_tmp = Torque_Vectoring_B->kDrop + 1; z_idx_0_tmp <=
                 Torque_Vectoring_B->nA; z_idx_0_tmp++) {
              Torque_Vectoring_B->iC_b = ((Torque_Vectoring_B->kDrop << 2) +
                z_idx_0_tmp) - 1;
              Torque_Vectoring_B->U[Torque_Vectoring_B->iC_b] = 0.0;
              for (Torque_Vectoring_B->iSave = 0; Torque_Vectoring_B->iSave <
                   Torque_Vectoring_B->i; Torque_Vectoring_B->iSave++) {
                U_tmp = Torque_Vectoring_B->iSave << 2;
                Torque_Vectoring_B->U[Torque_Vectoring_B->iC_b] +=
                  Torque_Vectoring_B->RLinv[(U_tmp + z_idx_0_tmp) - 1] *
                  Torque_Vectoring_B->RLinv[U_tmp + Torque_Vectoring_B->kDrop];
              }

              Torque_Vectoring_B->U[Torque_Vectoring_B->kDrop + ((z_idx_0_tmp -
                1) << 2)] = Torque_Vectoring_B->U[Torque_Vectoring_B->iC_b];
            }
          }

          for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop < 4;
               Torque_Vectoring_B->kDrop++) {
            Torque_Vectoring_B->Opt[Torque_Vectoring_B->kDrop] =
              ((Torque_Vectoring_B->H[Torque_Vectoring_B->kDrop + 4] *
                Torque_Vectoring_B->Rhs[1] + Torque_Vectoring_B->
                H[Torque_Vectoring_B->kDrop] * Torque_Vectoring_B->Rhs[0]) +
               Torque_Vectoring_B->H[Torque_Vectoring_B->kDrop + 8] *
               Torque_Vectoring_B->Rhs[2]) + Torque_Vectoring_B->
              H[Torque_Vectoring_B->kDrop + 12] * Torque_Vectoring_B->Rhs[3];
            for (z_idx_0_tmp = 0; z_idx_0_tmp < Torque_Vectoring_B->i;
                 z_idx_0_tmp++) {
              Torque_Vectoring_B->Opt[Torque_Vectoring_B->kDrop] +=
                Torque_Vectoring_B->D[(z_idx_0_tmp << 2) +
                Torque_Vectoring_B->kDrop] * Torque_Vectoring_B->Rhs[z_idx_0_tmp
                + 4];
            }
          }

          Torque_Vectoring_B->Xnorm0 = -1.0E-12;
          Torque_Vectoring_B->kDrop = -1;
          for (z_idx_0_tmp = 0; z_idx_0_tmp < Torque_Vectoring_B->i; z_idx_0_tmp
               ++) {
            Torque_Vectoring_B->iSave = z_idx_0_tmp << 2;
            Torque_Vectoring_B->Opt[z_idx_0_tmp + 4] = ((Torque_Vectoring_B->
              D[Torque_Vectoring_B->iSave + 1] * Torque_Vectoring_B->Rhs[1] +
              Torque_Vectoring_B->D[Torque_Vectoring_B->iSave] *
              Torque_Vectoring_B->Rhs[0]) + Torque_Vectoring_B->
              D[Torque_Vectoring_B->iSave + 2] * Torque_Vectoring_B->Rhs[2]) +
              Torque_Vectoring_B->D[Torque_Vectoring_B->iSave + 3] *
              Torque_Vectoring_B->Rhs[3];
            for (Torque_Vectoring_B->iSave = 0; Torque_Vectoring_B->iSave <
                 Torque_Vectoring_B->i; Torque_Vectoring_B->iSave++) {
              Torque_Vectoring_B->Opt[z_idx_0_tmp + 4] += Torque_Vectoring_B->U
                [(Torque_Vectoring_B->iSave << 2) + z_idx_0_tmp] *
                Torque_Vectoring_B->Rhs[Torque_Vectoring_B->iSave + 4];
            }

            Torque_Vectoring_B->cMin = Torque_Vectoring_B->Opt[z_idx_0_tmp + 4];
            lambda[Torque_Vectoring_B->iC[z_idx_0_tmp] - 1] =
              Torque_Vectoring_B->cMin;
            if ((Torque_Vectoring_B->cMin < Torque_Vectoring_B->Xnorm0) &&
                (z_idx_0_tmp + 1 <= Torque_Vectoring_B->nA - 2)) {
              Torque_Vectoring_B->kDrop = z_idx_0_tmp;
              Torque_Vectoring_B->Xnorm0 = Torque_Vectoring_B->cMin;
            }
          }

          if (Torque_Vectoring_B->kDrop + 1 <= 0) {
            DualFeasible = true;
            x[0] = Torque_Vectoring_B->Opt[0];
            x[1] = Torque_Vectoring_B->Opt[1];
            x[2] = Torque_Vectoring_B->Opt[2];
            x[3] = Torque_Vectoring_B->Opt[3];
          } else {
            (*status)++;
            if (*status > 5) {
              Torque_Vectori_ResetToColdStart(Torque_Vectoring_B->b_iA,
                Torque_Vectoring_B->iC);
              Torque_Vectoring_B->nA = 2;
              *iA_size = 10;
              for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop < 10;
                   Torque_Vectoring_B->kDrop++) {
                iA_data[Torque_Vectoring_B->kDrop] = Torque_Vectoring_B->
                  b_iA[Torque_Vectoring_B->kDrop];
              }

              ColdReset = true;
            } else {
              lambda[Torque_Vectoring_B->iC[Torque_Vectoring_B->kDrop] - 1] =
                0.0;
              Torque_Vectoring_DropConstraint(Torque_Vectoring_B->kDrop + 1,
                iA_data, &Torque_Vectoring_B->nA, Torque_Vectoring_B->iC);
            }
          }
        }
      } else {
        if (Torque_Vectoring_B->nA <= 0) {
          memset(&lambda[0], 0, 10U * sizeof(real_T));
          Torque_Vectoring_B->Xnorm0 = f[1];
          Torque_Vectoring_B->cMin = f[0];
          Torque_Vectoring_B->cVal = f[2];
          Torque_Vectoring_B->z_idx_2 = f[3];
          for (Torque_Vectoring_B->i = 0; Torque_Vectoring_B->i < 4;
               Torque_Vectoring_B->i++) {
            x[Torque_Vectoring_B->i] = ((-Hinv[Torque_Vectoring_B->i + 4] *
              Torque_Vectoring_B->Xnorm0 + -Hinv[Torque_Vectoring_B->i] *
              Torque_Vectoring_B->cMin) + -Hinv[Torque_Vectoring_B->i + 8] *
              Torque_Vectoring_B->cVal) + -Hinv[Torque_Vectoring_B->i + 12] *
              Torque_Vectoring_B->z_idx_2;
          }
        }

        exitg3 = 1;
      }
    } while (exitg3 == 0);

    if (exitg3 == 1) {
      guard1 = true;
    }
  } else {
    Torque_Vectoring_B->Xnorm0 = f[1];
    Torque_Vectoring_B->cMin = f[0];
    Torque_Vectoring_B->cVal = f[2];
    Torque_Vectoring_B->z_idx_2 = f[3];
    for (Torque_Vectoring_B->i = 0; Torque_Vectoring_B->i < 4;
         Torque_Vectoring_B->i++) {
      x[Torque_Vectoring_B->i] = ((-Hinv[Torque_Vectoring_B->i + 4] *
        Torque_Vectoring_B->Xnorm0 + -Hinv[Torque_Vectoring_B->i] *
        Torque_Vectoring_B->cMin) + -Hinv[Torque_Vectoring_B->i + 8] *
        Torque_Vectoring_B->cVal) + -Hinv[Torque_Vectoring_B->i + 12] *
        Torque_Vectoring_B->z_idx_2;
    }

    guard1 = true;
  }

  if (guard1) {
    Torque_Vectoring_B->Xnorm0 = Torque_Vectoring_norm(x);
    exitg2 = false;
    while ((!exitg2) && (*status <= 10)) {
      Torque_Vectoring_B->cMin = -0.001;
      Torque_Vectoring_B->i = -1;
      for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop < 8;
           Torque_Vectoring_B->kDrop++) {
        if (!cTolComputed) {
          Torque_Vectoring_B->cVal = fabs(Ac[Torque_Vectoring_B->kDrop] * x[0]);
          Torque_Vectoring_B->z_idx_2 = fabs(Ac[Torque_Vectoring_B->kDrop + 10] *
            x[1]);
          Torque_Vectoring_B->z_idx_3 = fabs(Ac[Torque_Vectoring_B->kDrop + 20] *
            x[2]);
          Torque_Vectoring_B->t1 = fabs(Ac[Torque_Vectoring_B->kDrop + 30] * x[3]);
          if (Torque_Vectoring_B->cVal < Torque_Vectoring_B->z_idx_2) {
            Torque_Vectoring_B->cVal = Torque_Vectoring_B->z_idx_2;
          }

          if (Torque_Vectoring_B->cVal < Torque_Vectoring_B->z_idx_3) {
            Torque_Vectoring_B->cVal = Torque_Vectoring_B->z_idx_3;
          }

          if (Torque_Vectoring_B->cVal < Torque_Vectoring_B->t1) {
            Torque_Vectoring_B->cVal = Torque_Vectoring_B->t1;
          }

          Torque_Vectoring_B->cTol[Torque_Vectoring_B->kDrop] = fmax
            (Torque_Vectoring_B->cTol[Torque_Vectoring_B->kDrop],
             Torque_Vectoring_B->cVal);
        }

        if (!iA_data[Torque_Vectoring_B->kDrop]) {
          Torque_Vectoring_B->cVal = ((((Ac[Torque_Vectoring_B->kDrop + 10] * x
            [1] + Ac[Torque_Vectoring_B->kDrop] * x[0]) + Ac
            [Torque_Vectoring_B->kDrop + 20] * x[2]) + Ac
            [Torque_Vectoring_B->kDrop + 30] * x[3]) - b
            [Torque_Vectoring_B->kDrop]) / Torque_Vectoring_B->
            cTol[Torque_Vectoring_B->kDrop];
          if (Torque_Vectoring_B->cVal < Torque_Vectoring_B->cMin) {
            Torque_Vectoring_B->cMin = Torque_Vectoring_B->cVal;
            Torque_Vectoring_B->i = Torque_Vectoring_B->kDrop;
          }
        }
      }

      cTolComputed = true;
      if (Torque_Vectoring_B->i + 1 <= 0) {
        exitg2 = true;
      } else if (*status == 10) {
        *status = 0;
        exitg2 = true;
      } else {
        do {
          exitg1 = 0;
          if ((Torque_Vectoring_B->i + 1 > 0) && (*status <= 10)) {
            guard2 = false;
            if (Torque_Vectoring_B->nA == 0) {
              Torque_Vectoring_B->cMin = 0.0;
              Torque_Vectoring_B->cVal = 0.0;
              Torque_Vectoring_B->z_idx_2 = 0.0;
              Torque_Vectoring_B->z_idx_3 = 0.0;
              for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop < 4;
                   Torque_Vectoring_B->kDrop++) {
                Torque_Vectoring_B->t1 = Ac[10 * Torque_Vectoring_B->kDrop +
                  Torque_Vectoring_B->i];
                z_idx_0_tmp = Torque_Vectoring_B->kDrop << 2;
                Torque_Vectoring_B->cMin += Hinv[z_idx_0_tmp] *
                  Torque_Vectoring_B->t1;
                Torque_Vectoring_B->cVal += Hinv[z_idx_0_tmp + 1] *
                  Torque_Vectoring_B->t1;
                Torque_Vectoring_B->z_idx_2 += Hinv[z_idx_0_tmp + 2] *
                  Torque_Vectoring_B->t1;
                Torque_Vectoring_B->z_idx_3 += Hinv[z_idx_0_tmp + 3] *
                  Torque_Vectoring_B->t1;
              }

              guard2 = true;
            } else {
              Torque_Vectoring_B->cMin = Torque_Vectoring_KWIKfactor(Ac,
                Torque_Vectoring_B->iC, Torque_Vectoring_B->nA, Linv,
                Torque_Vectoring_B->RLinv, Torque_Vectoring_B->D,
                Torque_Vectoring_B->H, Torque_Vectoring_B);
              if (Torque_Vectoring_B->cMin <= 0.0) {
                *status = -2;
                exitg1 = 1;
              } else {
                for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop <
                     16; Torque_Vectoring_B->kDrop++) {
                  Torque_Vectoring_B->U[Torque_Vectoring_B->kDrop] =
                    -Torque_Vectoring_B->H[Torque_Vectoring_B->kDrop];
                }

                Torque_Vectoring_B->cMin = 0.0;
                Torque_Vectoring_B->cVal = 0.0;
                Torque_Vectoring_B->z_idx_2 = 0.0;
                Torque_Vectoring_B->z_idx_3 = 0.0;
                for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop <
                     4; Torque_Vectoring_B->kDrop++) {
                  Torque_Vectoring_B->t1 = Ac[10 * Torque_Vectoring_B->kDrop +
                    Torque_Vectoring_B->i];
                  z_idx_0_tmp = Torque_Vectoring_B->kDrop << 2;
                  Torque_Vectoring_B->cMin += Torque_Vectoring_B->U[z_idx_0_tmp]
                    * Torque_Vectoring_B->t1;
                  Torque_Vectoring_B->cVal += Torque_Vectoring_B->U[z_idx_0_tmp
                    + 1] * Torque_Vectoring_B->t1;
                  Torque_Vectoring_B->z_idx_2 += Torque_Vectoring_B->
                    U[z_idx_0_tmp + 2] * Torque_Vectoring_B->t1;
                  Torque_Vectoring_B->z_idx_3 += Torque_Vectoring_B->
                    U[z_idx_0_tmp + 3] * Torque_Vectoring_B->t1;
                }

                z_idx_0_tmp = (uint8_T)Torque_Vectoring_B->nA;
                for (Torque_Vectoring_B->kDrop = 0; Torque_Vectoring_B->kDrop <
                     z_idx_0_tmp; Torque_Vectoring_B->kDrop++) {
                  Torque_Vectoring_B->iSave = Torque_Vectoring_B->kDrop << 2;
                  Torque_Vectoring_B->r[Torque_Vectoring_B->kDrop] =
                    ((Torque_Vectoring_B->D[Torque_Vectoring_B->iSave + 1] *
                      Ac[Torque_Vectoring_B->i + 10] + Torque_Vectoring_B->
                      D[Torque_Vectoring_B->iSave] * Ac[Torque_Vectoring_B->i])
                     + Torque_Vectoring_B->D[Torque_Vectoring_B->iSave + 2] *
                     Ac[Torque_Vectoring_B->i + 20]) + Torque_Vectoring_B->
                    D[Torque_Vectoring_B->iSave + 3] * Ac[Torque_Vectoring_B->i
                    + 30];
                }

                guard2 = true;
              }
            }

            if (guard2) {
              Torque_Vectoring_B->kDrop = 0;
              Torque_Vectoring_B->t1 = 0.0;
              DualFeasible = true;
              ColdReset = true;
              if (Torque_Vectoring_B->nA > 2) {
                z_idx_0_tmp = 0;
                exitg4 = false;
                while ((!exitg4) && (z_idx_0_tmp <= (uint8_T)
                                     (Torque_Vectoring_B->nA - 2) - 1)) {
                  if (Torque_Vectoring_B->r[z_idx_0_tmp] >= 1.0E-12) {
                    ColdReset = false;
                    exitg4 = true;
                  } else {
                    z_idx_0_tmp++;
                  }
                }
              }

              if ((Torque_Vectoring_B->nA != 2) && (!ColdReset)) {
                if (Torque_Vectoring_B->nA < -2147483646) {
                  Torque_Vectoring_B->iSave = MIN_int32_T;
                } else {
                  Torque_Vectoring_B->iSave = Torque_Vectoring_B->nA - 2;
                }

                for (z_idx_0_tmp = 0; z_idx_0_tmp < Torque_Vectoring_B->iSave;
                     z_idx_0_tmp++) {
                  Torque_Vectoring_B->rVal = Torque_Vectoring_B->r[z_idx_0_tmp];
                  if (Torque_Vectoring_B->rVal > 1.0E-12) {
                    Torque_Vectoring_B->rVal = lambda[Torque_Vectoring_B->
                      iC[z_idx_0_tmp] - 1] / Torque_Vectoring_B->rVal;
                    if ((Torque_Vectoring_B->kDrop == 0) ||
                        (Torque_Vectoring_B->rVal < Torque_Vectoring_B->rMin)) {
                      Torque_Vectoring_B->rMin = Torque_Vectoring_B->rVal;
                      Torque_Vectoring_B->kDrop = z_idx_0_tmp + 1;
                    }
                  }
                }

                if (Torque_Vectoring_B->kDrop > 0) {
                  Torque_Vectoring_B->t1 = Torque_Vectoring_B->rMin;
                  DualFeasible = false;
                }
              }

              Torque_Vectoring_B->rVal = Ac[Torque_Vectoring_B->i + 10];
              Torque_Vectoring_B->t = Ac[Torque_Vectoring_B->i + 20];
              Torque_Vectoring_B->z_tmp = Ac[Torque_Vectoring_B->i + 30];
              Torque_Vectoring_B->z = ((Torque_Vectoring_B->rVal *
                Torque_Vectoring_B->cVal + Torque_Vectoring_B->cMin *
                Ac[Torque_Vectoring_B->i]) + Torque_Vectoring_B->t *
                Torque_Vectoring_B->z_idx_2) + Torque_Vectoring_B->z_tmp *
                Torque_Vectoring_B->z_idx_3;
              if (Torque_Vectoring_B->z <= 0.0) {
                Torque_Vectoring_B->rVal = 0.0;
                ColdReset = true;
              } else {
                Torque_Vectoring_B->rVal = (b[Torque_Vectoring_B->i] -
                  (((Torque_Vectoring_B->rVal * x[1] + Ac[Torque_Vectoring_B->i]
                     * x[0]) + Torque_Vectoring_B->t * x[2]) +
                   Torque_Vectoring_B->z_tmp * x[3])) / Torque_Vectoring_B->z;
                ColdReset = false;
              }

              if (DualFeasible && ColdReset) {
                *status = -1;
                exitg1 = 1;
              } else {
                if (ColdReset) {
                  Torque_Vectoring_B->t = Torque_Vectoring_B->t1;
                } else if (DualFeasible) {
                  Torque_Vectoring_B->t = Torque_Vectoring_B->rVal;
                } else {
                  Torque_Vectoring_B->t = fmin(Torque_Vectoring_B->t1,
                    Torque_Vectoring_B->rVal);
                }

                Torque_Vectoring_B->iSave = (uint8_T)Torque_Vectoring_B->nA;
                for (z_idx_0_tmp = 0; z_idx_0_tmp < Torque_Vectoring_B->iSave;
                     z_idx_0_tmp++) {
                  Torque_Vectoring_B->iC_b = Torque_Vectoring_B->iC[z_idx_0_tmp];
                  lambda[Torque_Vectoring_B->iC_b - 1] -= Torque_Vectoring_B->t *
                    Torque_Vectoring_B->r[z_idx_0_tmp];
                  if ((Torque_Vectoring_B->iC_b <= 8) &&
                      (lambda[Torque_Vectoring_B->iC_b - 1] < 0.0)) {
                    lambda[Torque_Vectoring_B->iC_b - 1] = 0.0;
                  }
                }

                lambda[Torque_Vectoring_B->i] += Torque_Vectoring_B->t;
                if (fabs(Torque_Vectoring_B->t - Torque_Vectoring_B->t1) <
                    2.2204460492503131E-16) {
                  Torque_Vectoring_DropConstraint(Torque_Vectoring_B->kDrop,
                    iA_data, &Torque_Vectoring_B->nA, Torque_Vectoring_B->iC);
                }

                if (!ColdReset) {
                  x[0] += Torque_Vectoring_B->t * Torque_Vectoring_B->cMin;
                  x[1] += Torque_Vectoring_B->t * Torque_Vectoring_B->cVal;
                  x[2] += Torque_Vectoring_B->t * Torque_Vectoring_B->z_idx_2;
                  x[3] += Torque_Vectoring_B->t * Torque_Vectoring_B->z_idx_3;
                  if (fabs(Torque_Vectoring_B->t - Torque_Vectoring_B->rVal) <
                      2.2204460492503131E-16) {
                    if (Torque_Vectoring_B->nA == 4) {
                      *status = -1;
                      exitg1 = 1;
                    } else {
                      if (Torque_Vectoring_B->nA > 2147483646) {
                        Torque_Vectoring_B->nA = MAX_int32_T;
                      } else {
                        Torque_Vectoring_B->nA++;
                      }

                      Torque_Vectoring_B->iC[Torque_Vectoring_B->nA - 1] =
                        Torque_Vectoring_B->i + 1;
                      z_idx_0_tmp = Torque_Vectoring_B->nA - 1;
                      exitg4 = false;
                      while ((!exitg4) && (z_idx_0_tmp + 1 > 1)) {
                        Torque_Vectoring_B->kDrop = Torque_Vectoring_B->
                          iC[z_idx_0_tmp - 1];
                        if (Torque_Vectoring_B->iC[z_idx_0_tmp] >
                            Torque_Vectoring_B->kDrop) {
                          exitg4 = true;
                        } else {
                          Torque_Vectoring_B->iSave = Torque_Vectoring_B->
                            iC[z_idx_0_tmp];
                          Torque_Vectoring_B->iC[z_idx_0_tmp] =
                            Torque_Vectoring_B->kDrop;
                          Torque_Vectoring_B->iC[z_idx_0_tmp - 1] =
                            Torque_Vectoring_B->iSave;
                          z_idx_0_tmp--;
                        }
                      }

                      iA_data[Torque_Vectoring_B->i] = true;
                      Torque_Vectoring_B->i = -1;
                      (*status)++;
                    }
                  } else {
                    (*status)++;
                  }
                } else {
                  (*status)++;
                }
              }
            }
          } else {
            Torque_Vectoring_B->cMin = Torque_Vectoring_norm(x);
            if (fabs(Torque_Vectoring_B->cMin - Torque_Vectoring_B->Xnorm0) >
                0.001) {
              Torque_Vectoring_B->Xnorm0 = Torque_Vectoring_B->cMin;
              for (Torque_Vectoring_B->i = 0; Torque_Vectoring_B->i < 10;
                   Torque_Vectoring_B->i++) {
                Torque_Vectoring_B->cTol[Torque_Vectoring_B->i] = fmax(fabs
                  (b[Torque_Vectoring_B->i]), 1.0);
              }

              cTolComputed = false;
            }

            exitg1 = 2;
          }
        } while (exitg1 == 0);

        if (exitg1 == 1) {
          exitg2 = true;
        }
      }
    }
  }
}

/* Model step function */
void Torque_Vectoring_step(RT_MODEL_Torque_Vectoring_T *const Torque_Vectoring_M,
  real_T Torque_Vectoring_U_Vx_des, real_T Torque_Vectoring_U_Vx, real_T
  Torque_Vectoring_U_Vy, real_T Torque_Vectoring_U_AngWheel[4], real_T
  Torque_Vectoring_U_r, real_T Torque_Vectoring_U_Fy[4], real_T
  Torque_Vectoring_U_Fz[4], real_T Torque_Vectoring_Y_Tm[4], real_T
  Torque_Vectoring_Y_Fx_opt[4], real_T *Torque_Vectoring_Y_Mx_total, real_T
  *Torque_Vectoring_Y_Fx_total, real_T *Torque_Vectoring_Y_Mzd, real_T
  *Torque_Vectoring_Y_r_des, real_T Torque_Vectoring_Y_Ca[2], real_T
  *Torque_Vectoring_Y_beta)
{
  B_Torque_Vectoring_T *Torque_Vectoring_B = Torque_Vectoring_M->blockIO;
  DW_Torque_Vectoring_T *Torque_Vectoring_DW = Torque_Vectoring_M->dwork;
  static const real_T h[16] = { 1.0E-6, 0.0, 0.0, 0.0, 0.0, 1.0E-6, 0.0, 0.0,
    0.0, 0.0, 1.0E-6, 0.0, 0.0, 0.0, 0.0, 1.0E-6 };

  static const int8_T c_b[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1
  };

  static const int8_T l[32] = { -1, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1, 0, 0,
    0, 0, -1, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 1 };

  boolean_T exitg1;

  /* Sum: '<S5>/Sum1' incorporates:
   *  Inport: '<Root>/Vx'
   *  Inport: '<Root>/Vx_des'
   */
  Torque_Vectoring_B->Sum1 = Torque_Vectoring_U_Vx_des - Torque_Vectoring_U_Vx;

  /* Gain: '<S53>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S45>/Filter'
   *  Gain: '<S43>/Derivative Gain'
   *  Sum: '<S45>/SumD'
   */
  Torque_Vectoring_B->FilterCoefficient = (30.0 * Torque_Vectoring_B->Sum1 -
    Torque_Vectoring_DW->Filter_DSTATE) * 100.0;

  /* MATLAB Function: '<S5>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Vx_des'
   */
  if (Torque_Vectoring_U_Vx_des > 1.0) {
    /* Sum: '<S59>/Sum' incorporates:
     *  DiscreteIntegrator: '<S50>/Integrator'
     *  Gain: '<S55>/Proportional Gain'
     */
    *Torque_Vectoring_Y_Fx_total = (1500.0 * Torque_Vectoring_B->Sum1 +
      Torque_Vectoring_DW->Integrator_DSTATE) +
      Torque_Vectoring_B->FilterCoefficient;

    /* Saturate: '<S5>/Saturation' */
    if (*Torque_Vectoring_Y_Fx_total > 5549.1329479768792) {
      *Torque_Vectoring_Y_Fx_total = 5549.1329479768792;
    } else if (*Torque_Vectoring_Y_Fx_total < 0.0) {
      *Torque_Vectoring_Y_Fx_total = 0.0;
    }

    /* End of Saturate: '<S5>/Saturation' */
  } else {
    *Torque_Vectoring_Y_Fx_total = 0.0;
  }

  /* End of MATLAB Function: '<S5>/MATLAB Function' */

  /* Gain: '<S13>/Gain1' incorporates:
   *  Gain: '<S7>/Gain1'
   *  Inport: '<Root>/Vx'
   */
  Torque_Vectoring_B->TSamp = 0.27777777777777779 * Torque_Vectoring_U_Vx;

  /* Gain: '<S13>/Gain2' incorporates:
   *  Constant: '<S13>/Constant'
   *  Gain: '<S13>/Gain1'
   *  MinMax: '<S13>/Max'
   *  Product: '<S13>/Divide'
   */
  Torque_Vectoring_B->Gain2 = Torque_Vectoring_ConstB.Gain / fmax
    (Torque_Vectoring_B->TSamp, 0.01) * 0.85;

  /* Product: '<S8>/Divide' incorporates:
   *  Constant: '<S8>/Constant'
   *  Inport: '<Root>/AngWheel'
   *  Sum: '<S8>/Add2'
   */
  Torque_Vectoring_B->Divide_n = (Torque_Vectoring_U_AngWheel[0] +
    Torque_Vectoring_U_AngWheel[1]) / 2.0;

  /* Product: '<S8>/Divide1' incorporates:
   *  Constant: '<S8>/Constant1'
   *  Inport: '<Root>/AngWheel'
   *  Sum: '<S8>/Add1'
   */
  Torque_Vectoring_B->Divide1 = (Torque_Vectoring_U_AngWheel[2] +
    Torque_Vectoring_U_AngWheel[3]) / 2.0;

  /* Sum: '<S2>/Add' incorporates:
   *  Inport: '<Root>/Fz'
   *  Lookup_n-D: '<S2>/LUT_Fz2Ca'
   */
  Torque_Vectoring_B->front = look1_binlxpw(Torque_Vectoring_U_Fz[0],
    Torque_Vectoring_ConstP.LUT_Fz2Ca_bp01Data,
    Torque_Vectoring_ConstP.LUT_Fz2Ca_tableData, 7U) + look1_binlxpw
    (Torque_Vectoring_U_Fz[1], Torque_Vectoring_ConstP.LUT_Fz2Ca_bp01Data,
     Torque_Vectoring_ConstP.LUT_Fz2Ca_tableData, 7U);

  /* Sum: '<S2>/Add1' incorporates:
   *  Inport: '<Root>/Fz'
   *  Lookup_n-D: '<S2>/LUT_Fz2Ca'
   */
  Torque_Vectoring_B->rear = look1_binlxpw(Torque_Vectoring_U_Fz[2],
    Torque_Vectoring_ConstP.LUT_Fz2Ca_bp01Data,
    Torque_Vectoring_ConstP.LUT_Fz2Ca_tableData, 7U) + look1_binlxpw
    (Torque_Vectoring_U_Fz[3], Torque_Vectoring_ConstP.LUT_Fz2Ca_bp01Data,
     Torque_Vectoring_ConstP.LUT_Fz2Ca_tableData, 7U);

  /* MATLAB Function: '<S4>/Reference Model' incorporates:
   *  Inport: '<Root>/Vx'
   */
  Torque_Vectoring_B->Vx_ms = Torque_Vectoring_U_Vx / 3.6;
  if (Torque_Vectoring_U_Vx > 3.0) {
    /* Switch: '<S11>/Switch2' incorporates:
     *  SignalConversion generated from: '<S10>/ SFunction '
     */
    *Torque_Vectoring_Y_r_des = Torque_Vectoring_B->Vx_ms /
      (Torque_Vectoring_B->Vx_ms * Torque_Vectoring_B->Vx_ms * 2772.0 * (2.2975 *
        Torque_Vectoring_B->rear - 2.2975 * Torque_Vectoring_B->front) /
       (Torque_Vectoring_B->front * Torque_Vectoring_B->rear * 4.595) + 4.595) *
      (Torque_Vectoring_B->Divide_n - Torque_Vectoring_B->Divide1);
  } else {
    /* Switch: '<S11>/Switch2' */
    *Torque_Vectoring_Y_r_des = 0.0;
  }

  /* Switch: '<S11>/Switch2' incorporates:
   *  Gain: '<S4>/Gain3'
   *  MATLAB Function: '<S4>/Reference Model'
   *  RelationalOperator: '<S11>/LowerRelop1'
   *  RelationalOperator: '<S11>/UpperRelop'
   *  Switch: '<S11>/Switch'
   */
  if (*Torque_Vectoring_Y_r_des > Torque_Vectoring_B->Gain2) {
    /* Switch: '<S11>/Switch2' */
    *Torque_Vectoring_Y_r_des = Torque_Vectoring_B->Gain2;
  } else if (*Torque_Vectoring_Y_r_des < -Torque_Vectoring_B->Gain2) {
    /* Switch: '<S11>/Switch2' incorporates:
     *  Gain: '<S4>/Gain3'
     *  Switch: '<S11>/Switch'
     */
    *Torque_Vectoring_Y_r_des = -Torque_Vectoring_B->Gain2;
  }

  /* End of Switch: '<S11>/Switch2' */

  /* Trigonometry: '<S7>/Atan' incorporates:
   *  Constant: '<S7>/Constant1'
   *  Gain: '<S7>/Gain2'
   *  Inport: '<Root>/Vy'
   *  MinMax: '<S7>/Max'
   *  Product: '<S7>/Divide1'
   */
  *Torque_Vectoring_Y_beta = atan(0.27777777777777779 * Torque_Vectoring_U_Vy /
    fmax(0.01, Torque_Vectoring_B->TSamp));

  /* Sum: '<S70>/Add' incorporates:
   *  Gain: '<S70>/xi'
   *  Inport: '<Root>/r'
   *  Sum: '<S70>/Sum1'
   *  Sum: '<S70>/Sum2'
   */
  Torque_Vectoring_B->Vx_ms = (Torque_Vectoring_U_r - *Torque_Vectoring_Y_r_des)
    + 0.08 * *Torque_Vectoring_Y_beta;

  /* SampleTimeMath: '<S71>/TSamp'
   *
   * About '<S71>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *   */
  Torque_Vectoring_B->TSamp = *Torque_Vectoring_Y_r_des * 500.0;

  /* SampleTimeMath: '<S72>/TSamp'
   *
   * About '<S72>/TSamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *   */
  Torque_Vectoring_B->Gain2 = *Torque_Vectoring_Y_beta * 500.0;

  /* Product: '<S68>/Divide' incorporates:
   *  Constant: '<S68>/Phi'
   */
  Torque_Vectoring_B->vprev = Torque_Vectoring_B->Vx_ms / 0.045;

  /* Saturate: '<S68>/Saturation' */
  if (Torque_Vectoring_B->vprev > 1.0) {
    Torque_Vectoring_B->vprev = 1.0;
  } else if (Torque_Vectoring_B->vprev < -1.0) {
    Torque_Vectoring_B->vprev = -1.0;
  }

  /* Sum: '<S69>/Subtract' incorporates:
   *  Gain: '<S68>/eta'
   *  Gain: '<S68>/lambda'
   *  Gain: '<S69>/xi1'
   *  Saturate: '<S68>/Saturation'
   *  Sum: '<S68>/Subtract'
   *  Sum: '<S69>/Sum3'
   *  Sum: '<S69>/Sum4'
   *  Sum: '<S71>/Diff'
   *  Sum: '<S72>/Diff'
   *  Sum: '<S73>/Diff'
   *  UnitDelay: '<S71>/UD'
   *  UnitDelay: '<S72>/UD'
   *  UnitDelay: '<S73>/UD'
   *
   * Block description for '<S71>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S72>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S73>/Diff':
   *
   *  Add in CPU
   *
   * Block description for '<S71>/UD':
   *
   *  Store in Global RAM
   *
   * Block description for '<S72>/UD':
   *
   *  Store in Global RAM
   *
   * Block description for '<S73>/UD':
   *
   *  Store in Global RAM
   */
  Torque_Vectoring_B->Vx_ms = (((0.0 - 5.5 * Torque_Vectoring_B->Vx_ms) - 2.5 *
    Torque_Vectoring_B->vprev) + (Torque_Vectoring_B->TSamp -
    Torque_Vectoring_DW->UD_DSTATE)) - ((Torque_Vectoring_B->Gain2 -
    Torque_Vectoring_DW->UD_DSTATE_l) - (0.0 - Torque_Vectoring_DW->UD_DSTATE_j))
    * 0.08;

  /* MATLABSystem: '<S69>/Median Filter' */
  if (Torque_Vectoring_DW->obj.pMID.isInitialized != 1) {
    Torque_Vectoring_DW->obj.pMID.isInitialized = 1;
    Torque_Vectoring_DW->obj.pMID.isSetupComplete = true;
    Torque_V_MedianFilter_resetImpl(&Torque_Vectoring_DW->obj.pMID);
  }

  Torque_Vectoring_B->vprev = Torque_Vectoring_DW->obj.pMID.pBuf[(int32_T)
    Torque_Vectoring_DW->obj.pMID.pIdx - 1];
  Torque_Vectoring_DW->obj.pMID.pBuf[(int32_T)Torque_Vectoring_DW->obj.pMID.pIdx
    - 1] = Torque_Vectoring_B->Vx_ms;
  Torque_Vectoring_B->p = Torque_Vectoring_DW->obj.pMID.pPos[(int32_T)
    Torque_Vectoring_DW->obj.pMID.pIdx - 1];
  Torque_Vectoring_DW->obj.pMID.pIdx++;
  if (Torque_Vectoring_DW->obj.pMID.pWinLen + 1.0 ==
      Torque_Vectoring_DW->obj.pMID.pIdx) {
    Torque_Vectoring_DW->obj.pMID.pIdx = 1.0;
  }

  if (Torque_Vectoring_B->p > Torque_Vectoring_DW->obj.pMID.pMidHeap) {
    if (Torque_Vectoring_B->vprev < Torque_Vectoring_B->Vx_ms) {
      Tor_MedianFilter_trickleDownMin(&Torque_Vectoring_DW->obj.pMID,
        (Torque_Vectoring_B->p - Torque_Vectoring_DW->obj.pMID.pMidHeap) * 2.0);
    } else {
      Torque_Vectoring_B->Vx_ms = Torque_Vectoring_B->p -
        Torque_Vectoring_DW->obj.pMID.pMidHeap;
      exitg1 = false;
      while ((!exitg1) && (Torque_Vectoring_B->Vx_ms > 0.0)) {
        Torque_Vectoring_B->vprev = Torque_Vectoring_B->Vx_ms / 2.0;
        Torque_Vectoring_B->p = Torque_Vectoring_B->Vx_ms +
          Torque_Vectoring_DW->obj.pMID.pMidHeap;
        Torque_Vectoring_B->ind2 = trunc(Torque_Vectoring_B->vprev) +
          Torque_Vectoring_DW->obj.pMID.pMidHeap;
        Torque_Vectoring_B->d = Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)
          Torque_Vectoring_B->p - 1];
        Torque_Vectoring_B->d1 = Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)
          Torque_Vectoring_B->ind2 - 1];
        if (Torque_Vectoring_DW->obj.pMID.pBuf[(int32_T)Torque_Vectoring_B->d -
            1] >= Torque_Vectoring_DW->obj.pMID.pBuf[(int32_T)
            Torque_Vectoring_B->d1 - 1]) {
          exitg1 = true;
        } else {
          Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)Torque_Vectoring_B->p - 1]
            = Torque_Vectoring_B->d1;
          Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)Torque_Vectoring_B->ind2
            - 1] = Torque_Vectoring_B->d;
          Torque_Vectoring_DW->obj.pMID.pPos[(int32_T)
            Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)Torque_Vectoring_B->p -
            1] - 1] = Torque_Vectoring_B->p;
          Torque_Vectoring_DW->obj.pMID.pPos[(int32_T)
            Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)
            Torque_Vectoring_B->ind2 - 1] - 1] = Torque_Vectoring_B->ind2;
          Torque_Vectoring_B->Vx_ms = trunc(Torque_Vectoring_B->vprev);
        }
      }

      if (Torque_Vectoring_B->Vx_ms == 0.0) {
        Tor_MedianFilter_trickleDownMax(&Torque_Vectoring_DW->obj.pMID, -1.0);
      }
    }
  } else if (Torque_Vectoring_B->p < Torque_Vectoring_DW->obj.pMID.pMidHeap) {
    if (Torque_Vectoring_B->Vx_ms < Torque_Vectoring_B->vprev) {
      Tor_MedianFilter_trickleDownMax(&Torque_Vectoring_DW->obj.pMID,
        (Torque_Vectoring_B->p - Torque_Vectoring_DW->obj.pMID.pMidHeap) * 2.0);
    } else {
      Torque_Vectoring_B->Vx_ms = Torque_Vectoring_B->p -
        Torque_Vectoring_DW->obj.pMID.pMidHeap;
      exitg1 = false;
      while ((!exitg1) && (Torque_Vectoring_B->Vx_ms < 0.0)) {
        Torque_Vectoring_B->vprev = Torque_Vectoring_B->Vx_ms / 2.0;
        Torque_Vectoring_B->p = trunc(Torque_Vectoring_B->vprev) +
          Torque_Vectoring_DW->obj.pMID.pMidHeap;
        Torque_Vectoring_B->ind2 = Torque_Vectoring_B->Vx_ms +
          Torque_Vectoring_DW->obj.pMID.pMidHeap;
        Torque_Vectoring_B->d = Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)
          Torque_Vectoring_B->p - 1];
        Torque_Vectoring_B->d1 = Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)
          Torque_Vectoring_B->ind2 - 1];
        if (Torque_Vectoring_DW->obj.pMID.pBuf[(int32_T)Torque_Vectoring_B->d -
            1] >= Torque_Vectoring_DW->obj.pMID.pBuf[(int32_T)
            Torque_Vectoring_B->d1 - 1]) {
          exitg1 = true;
        } else {
          Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)Torque_Vectoring_B->p - 1]
            = Torque_Vectoring_B->d1;
          Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)Torque_Vectoring_B->ind2
            - 1] = Torque_Vectoring_B->d;
          Torque_Vectoring_DW->obj.pMID.pPos[(int32_T)
            Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)Torque_Vectoring_B->p -
            1] - 1] = Torque_Vectoring_B->p;
          Torque_Vectoring_DW->obj.pMID.pPos[(int32_T)
            Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)
            Torque_Vectoring_B->ind2 - 1] - 1] = Torque_Vectoring_B->ind2;
          Torque_Vectoring_B->Vx_ms = trunc(Torque_Vectoring_B->vprev);
        }
      }

      if (Torque_Vectoring_B->Vx_ms == 0.0) {
        Tor_MedianFilter_trickleDownMin(&Torque_Vectoring_DW->obj.pMID, 1.0);
      }
    }
  } else {
    if (Torque_Vectoring_DW->obj.pMID.pMaxHeapLength != 0.0) {
      Tor_MedianFilter_trickleDownMax(&Torque_Vectoring_DW->obj.pMID, -1.0);
    }

    if (Torque_Vectoring_DW->obj.pMID.pMinHeapLength > 0.0) {
      Tor_MedianFilter_trickleDownMin(&Torque_Vectoring_DW->obj.pMID, 1.0);
    }
  }

  /* Sum: '<S6>/Subtract' incorporates:
   *  Constant: '<S6>/Iz'
   *  Gain: '<S6>/Gain'
   *  Gain: '<S6>/Gain1'
   *  Inport: '<Root>/Fy'
   *  MATLABSystem: '<S69>/Median Filter'
   *  Product: '<S6>/Product'
   *  Product: '<S6>/Product1'
   *  Product: '<S6>/Product2'
   *  SignalConversion generated from: '<S6>/Cos1'
   *  Sum: '<S6>/Sum1'
   *  Sum: '<S6>/Sum2'
   *  Trigonometry: '<S6>/Cos1'
   * */
  *Torque_Vectoring_Y_Mzd = (Torque_Vectoring_DW->obj.pMID.pBuf[(int32_T)
    Torque_Vectoring_DW->obj.pMID.pHeap[(int32_T)
    Torque_Vectoring_DW->obj.pMID.pMidHeap - 1] - 1] * 4116.0 -
    (Torque_Vectoring_U_Fy[0] + Torque_Vectoring_U_Fy[1]) * 2.2975 * cos
    (Torque_Vectoring_B->Divide_n)) + (Torque_Vectoring_U_Fy[2] +
    Torque_Vectoring_U_Fy[3]) * 2.2975 * cos(Torque_Vectoring_B->Divide1);

  /* MATLAB Function: '<S3>/MATLAB Function1' incorporates:
   *  Constant: '<S3>/Constant1'
   *  Gain: '<S3>/Gain'
   *  Inport: '<Root>/AngWheel'
   *  Inport: '<Root>/Fz'
   *  UnitDelay: '<S3>/Unit Delay'
   */
  if (!Torque_Vectoring_DW->iA0_mem_not_empty) {
    Torque_Vectoring_DW->iA0_mem.size = 8;
    for (Torque_Vectoring_B->jmax = 0; Torque_Vectoring_B->jmax < 8;
         Torque_Vectoring_B->jmax++) {
      Torque_Vectoring_DW->iA0_mem.data[Torque_Vectoring_B->jmax] = false;
    }

    Torque_Vectoring_DW->iA0_mem_not_empty = true;
  }

  Torque_Vectoring_B->Mx_coeff[0] = 2.2975 * sin(Torque_Vectoring_U_AngWheel[0])
    - 0.9 * cos(Torque_Vectoring_U_AngWheel[0]);
  Torque_Vectoring_B->Mx_coeff[1] = 2.2975 * sin(Torque_Vectoring_U_AngWheel[1])
    + 0.9 * cos(Torque_Vectoring_U_AngWheel[1]);
  Torque_Vectoring_B->Mx_coeff[2] = -2.2975 * sin(Torque_Vectoring_U_AngWheel[2])
    - 0.9 * cos(Torque_Vectoring_U_AngWheel[2]);
  Torque_Vectoring_B->Mx_coeff[3] = -2.2975 * sin(Torque_Vectoring_U_AngWheel[3])
    + 0.9 * cos(Torque_Vectoring_U_AngWheel[3]);
  Torque_Vectoring_B->Divide_n = Torque_Vectoring_B->Mx_coeff[0] /
    (Torque_Vectoring_U_Fz[0] + 1.0E-6);
  Torque_Vectoring_B->Divide1 = Torque_Vectoring_B->Divide_n *
    Torque_Vectoring_B->Divide_n;
  Torque_Vectoring_B->Divide_n = Torque_Vectoring_B->Mx_coeff[1] /
    (Torque_Vectoring_U_Fz[1] + 1.0E-6);
  Torque_Vectoring_B->Vx_ms = Torque_Vectoring_B->Divide_n *
    Torque_Vectoring_B->Divide_n;
  Torque_Vectoring_B->Divide_n = Torque_Vectoring_B->Mx_coeff[2] /
    (Torque_Vectoring_U_Fz[2] + 1.0E-6);
  Torque_Vectoring_B->vprev = Torque_Vectoring_B->Divide_n *
    Torque_Vectoring_B->Divide_n;
  Torque_Vectoring_B->Divide_n = Torque_Vectoring_B->Mx_coeff[3] /
    (Torque_Vectoring_U_Fz[3] + 1.0E-6);
  memset(&Torque_Vectoring_B->b[0], 0, sizeof(real_T) << 4U);
  Torque_Vectoring_B->b[0] = Torque_Vectoring_B->Divide1;
  Torque_Vectoring_B->b[5] = Torque_Vectoring_B->Vx_ms;
  Torque_Vectoring_B->b[10] = Torque_Vectoring_B->vprev;
  Torque_Vectoring_B->b[15] = Torque_Vectoring_B->Divide_n *
    Torque_Vectoring_B->Divide_n;
  for (Torque_Vectoring_B->jmax = 0; Torque_Vectoring_B->jmax < 16;
       Torque_Vectoring_B->jmax++) {
    Torque_Vectoring_B->b[Torque_Vectoring_B->jmax] = 2.0 *
      Torque_Vectoring_B->b[Torque_Vectoring_B->jmax] + h
      [Torque_Vectoring_B->jmax];
  }

  Torque_Vectoring_B->Divide_n = Torque_Vectoring_ConstB.Gain1 / 0.346;
  Torque_Vectoring_B->LUT_Fz2Ca[0] = fmax(fmin(Torque_Vectoring_B->Divide_n, 0.6
    * Torque_Vectoring_U_Fz[0]), 0.0);
  Torque_Vectoring_B->LUT_Fz2Ca[1] = fmax(fmin(Torque_Vectoring_B->Divide_n, 0.6
    * Torque_Vectoring_U_Fz[1]), 0.0);
  Torque_Vectoring_B->LUT_Fz2Ca[2] = fmax(fmin(Torque_Vectoring_B->Divide_n, 0.6
    * Torque_Vectoring_U_Fz[2]), 0.0);
  Torque_Vectoring_B->LUT_Fz2Ca[3] = fmax(fmin(Torque_Vectoring_B->Divide_n, 0.6
    * Torque_Vectoring_U_Fz[3]), 0.0);
  Torque_Vectoring_B->jmax = 0;
  Torque_Vectoring_B->d_j = 0;
  exitg1 = false;
  while ((!exitg1) && (Torque_Vectoring_B->d_j < 4)) {
    Torque_Vectoring_B->idxAjj = (Torque_Vectoring_B->d_j << 2) +
      Torque_Vectoring_B->d_j;
    Torque_Vectoring_B->Divide_n = 0.0;
    if (Torque_Vectoring_B->d_j >= 1) {
      for (Torque_Vectoring_B->e_k = 0; Torque_Vectoring_B->e_k <
           Torque_Vectoring_B->d_j; Torque_Vectoring_B->e_k++) {
        Torque_Vectoring_B->Divide1 = Torque_Vectoring_B->b
          [(Torque_Vectoring_B->e_k << 2) + Torque_Vectoring_B->d_j];
        Torque_Vectoring_B->Divide_n += Torque_Vectoring_B->Divide1 *
          Torque_Vectoring_B->Divide1;
      }
    }

    Torque_Vectoring_B->Divide_n = Torque_Vectoring_B->b
      [Torque_Vectoring_B->idxAjj] - Torque_Vectoring_B->Divide_n;
    if (Torque_Vectoring_B->Divide_n > 0.0) {
      Torque_Vectoring_B->Divide_n = sqrt(Torque_Vectoring_B->Divide_n);
      Torque_Vectoring_B->b[Torque_Vectoring_B->idxAjj] =
        Torque_Vectoring_B->Divide_n;
      if (Torque_Vectoring_B->d_j + 1 < 4) {
        if (Torque_Vectoring_B->d_j != 0) {
          Torque_Vectoring_B->e_k = (((Torque_Vectoring_B->d_j - 1) << 2) +
            Torque_Vectoring_B->d_j) + 2;
          for (Torque_Vectoring_B->iac = Torque_Vectoring_B->d_j + 2;
               Torque_Vectoring_B->iac <= Torque_Vectoring_B->e_k;
               Torque_Vectoring_B->iac += 4) {
            Torque_Vectoring_B->b_c_tmp = Torque_Vectoring_B->iac -
              Torque_Vectoring_B->d_j;
            Torque_Vectoring_B->Divide1 = -Torque_Vectoring_B->b
              [(((Torque_Vectoring_B->b_c_tmp - 2) >> 2) << 2) +
              Torque_Vectoring_B->d_j];
            Torque_Vectoring_B->b_c_tmp += 2;
            for (Torque_Vectoring_B->ia = Torque_Vectoring_B->iac;
                 Torque_Vectoring_B->ia <= Torque_Vectoring_B->b_c_tmp;
                 Torque_Vectoring_B->ia++) {
              Torque_Vectoring_B->b_tmp = ((Torque_Vectoring_B->idxAjj +
                Torque_Vectoring_B->ia) - Torque_Vectoring_B->iac) + 1;
              Torque_Vectoring_B->b[Torque_Vectoring_B->b_tmp] +=
                Torque_Vectoring_B->b[Torque_Vectoring_B->ia - 1] *
                Torque_Vectoring_B->Divide1;
            }
          }
        }

        Torque_Vectoring_B->Divide_n = 1.0 / Torque_Vectoring_B->Divide_n;
        Torque_Vectoring_B->e_k = (Torque_Vectoring_B->idxAjj -
          Torque_Vectoring_B->d_j) + 4;
        for (Torque_Vectoring_B->iac = Torque_Vectoring_B->idxAjj + 2;
             Torque_Vectoring_B->iac <= Torque_Vectoring_B->e_k;
             Torque_Vectoring_B->iac++) {
          Torque_Vectoring_B->b[Torque_Vectoring_B->iac - 1] *=
            Torque_Vectoring_B->Divide_n;
        }
      }

      Torque_Vectoring_B->d_j++;
    } else {
      Torque_Vectoring_B->b[Torque_Vectoring_B->idxAjj] =
        Torque_Vectoring_B->Divide_n;
      Torque_Vectoring_B->jmax = Torque_Vectoring_B->d_j + 1;
      exitg1 = true;
    }
  }

  if (Torque_Vectoring_B->jmax == 0) {
    Torque_Vectoring_B->jmax = 5;
  }

  for (Torque_Vectoring_B->d_j = 2; Torque_Vectoring_B->d_j <
       Torque_Vectoring_B->jmax; Torque_Vectoring_B->d_j++) {
    for (Torque_Vectoring_B->idxAjj = 0; Torque_Vectoring_B->idxAjj <=
         Torque_Vectoring_B->d_j - 2; Torque_Vectoring_B->idxAjj++) {
      Torque_Vectoring_B->b[Torque_Vectoring_B->idxAjj +
        ((Torque_Vectoring_B->d_j - 1) << 2)] = 0.0;
    }
  }

  for (Torque_Vectoring_B->jmax = 0; Torque_Vectoring_B->jmax < 4;
       Torque_Vectoring_B->jmax++) {
    Torque_Vectoring_B->d_j = Torque_Vectoring_B->jmax << 2;
    Torque_Vectoring_B->Linv[Torque_Vectoring_B->d_j] = c_b
      [Torque_Vectoring_B->d_j];
    Torque_Vectoring_B->Linv[Torque_Vectoring_B->d_j + 1] =
      c_b[Torque_Vectoring_B->d_j + 1];
    Torque_Vectoring_B->Linv[Torque_Vectoring_B->d_j + 2] =
      c_b[Torque_Vectoring_B->d_j + 2];
    Torque_Vectoring_B->Linv[Torque_Vectoring_B->d_j + 3] =
      c_b[Torque_Vectoring_B->d_j + 3];
  }

  Torque_Vectoring_trisolve(Torque_Vectoring_B->b, Torque_Vectoring_B->Linv);
  Torque_Vectoring_B->iA1_size = Torque_Vectoring_DW->iA0_mem.size + 2;
  Torque_Vectoring_B->d_j = Torque_Vectoring_DW->iA0_mem.size;
  if (Torque_Vectoring_B->d_j - 1 >= 0) {
    memcpy(&Torque_Vectoring_B->iA1_data[0], &Torque_Vectoring_DW->iA0_mem.data
           [0], (uint32_T)Torque_Vectoring_B->d_j * sizeof(boolean_T));
  }

  Torque_Vectoring_B->iA1_data[Torque_Vectoring_DW->iA0_mem.size] = true;
  Torque_Vectoring_B->iA1_data[Torque_Vectoring_DW->iA0_mem.size + 1] = true;
  for (Torque_Vectoring_B->jmax = 0; Torque_Vectoring_B->jmax < 4;
       Torque_Vectoring_B->jmax++) {
    for (Torque_Vectoring_B->idxAjj = 0; Torque_Vectoring_B->idxAjj < 4;
         Torque_Vectoring_B->idxAjj++) {
      Torque_Vectoring_B->d_j = Torque_Vectoring_B->idxAjj << 2;
      Torque_Vectoring_B->e_k = Torque_Vectoring_B->jmax << 2;
      Torque_Vectoring_B->b[Torque_Vectoring_B->idxAjj + Torque_Vectoring_B->e_k]
        = ((Torque_Vectoring_B->Linv[Torque_Vectoring_B->d_j + 1] *
            Torque_Vectoring_B->Linv[Torque_Vectoring_B->e_k + 1] +
            Torque_Vectoring_B->Linv[Torque_Vectoring_B->d_j] *
            Torque_Vectoring_B->Linv[Torque_Vectoring_B->e_k]) +
           Torque_Vectoring_B->Linv[Torque_Vectoring_B->d_j + 2] *
           Torque_Vectoring_B->Linv[Torque_Vectoring_B->e_k + 2]) +
        Torque_Vectoring_B->Linv[Torque_Vectoring_B->d_j + 3] *
        Torque_Vectoring_B->Linv[Torque_Vectoring_B->e_k + 3];
    }

    Torque_Vectoring_B->dv[Torque_Vectoring_B->jmax] = 0.0;
    for (Torque_Vectoring_B->idxAjj = 0; Torque_Vectoring_B->idxAjj < 8;
         Torque_Vectoring_B->idxAjj++) {
      Torque_Vectoring_B->l[Torque_Vectoring_B->idxAjj + 10 *
        Torque_Vectoring_B->jmax] = l[(Torque_Vectoring_B->jmax << 3) +
        Torque_Vectoring_B->idxAjj];
    }

    Torque_Vectoring_B->l[10 * Torque_Vectoring_B->jmax + 8] =
      Torque_Vectoring_B->Mx_coeff[Torque_Vectoring_B->jmax];
    Torque_Vectoring_B->l[10 * Torque_Vectoring_B->jmax + 9] = 1.0;
    Torque_Vectoring_B->Divide_n = Torque_Vectoring_B->
      LUT_Fz2Ca[Torque_Vectoring_B->jmax];
    Torque_Vectoring_B->Fx_max[Torque_Vectoring_B->jmax] =
      -Torque_Vectoring_B->Divide_n;
    Torque_Vectoring_B->Fx_max[Torque_Vectoring_B->jmax + 4] =
      -Torque_Vectoring_B->Divide_n;
  }

  Torque_Vectoring_B->Fx_max[8] = *Torque_Vectoring_Y_Mzd;
  Torque_Vectoring_B->Fx_max[9] = *Torque_Vectoring_Y_Fx_total;
  Torque_Vectoring_qpkwik(Torque_Vectoring_B->Linv, Torque_Vectoring_B->b,
    Torque_Vectoring_B->dv, Torque_Vectoring_B->l, Torque_Vectoring_B->Fx_max,
    Torque_Vectoring_B->iA1_data, &Torque_Vectoring_B->iA1_size,
    Torque_Vectoring_B->LUT_Fz2Ca, Torque_Vectoring_B->lam,
    &Torque_Vectoring_B->jmax, Torque_Vectoring_B);
  if (Torque_Vectoring_B->jmax > 0) {
    Torque_Vectoring_Y_Fx_opt[0] = Torque_Vectoring_B->LUT_Fz2Ca[0];
    Torque_Vectoring_Y_Fx_opt[1] = Torque_Vectoring_B->LUT_Fz2Ca[1];
    Torque_Vectoring_Y_Fx_opt[2] = Torque_Vectoring_B->LUT_Fz2Ca[2];
    Torque_Vectoring_Y_Fx_opt[3] = Torque_Vectoring_B->LUT_Fz2Ca[3];
    Torque_Vectoring_DW->iA0_mem.size = 8;
    for (Torque_Vectoring_B->jmax = 0; Torque_Vectoring_B->jmax < 8;
         Torque_Vectoring_B->jmax++) {
      Torque_Vectoring_DW->iA0_mem.data[Torque_Vectoring_B->jmax] =
        Torque_Vectoring_B->iA1_data[Torque_Vectoring_B->jmax];
    }
  }

  /* Outport: '<Root>/Tm' incorporates:
   *  Constant: '<S3>/Constant'
   *  Gain: '<S3>/Gain'
   *  Product: '<S3>/Divide'
   *  UnitDelay: '<S3>/Unit Delay'
   */
  Torque_Vectoring_Y_Tm[0] = 0.346 * Torque_Vectoring_Y_Fx_opt[0] / 4.0;
  Torque_Vectoring_Y_Tm[1] = 0.346 * Torque_Vectoring_Y_Fx_opt[1] / 4.0;
  Torque_Vectoring_Y_Tm[2] = 0.346 * Torque_Vectoring_Y_Fx_opt[2] / 4.0;
  Torque_Vectoring_Y_Tm[3] = 0.346 * Torque_Vectoring_Y_Fx_opt[3] / 4.0;

  /* Outport: '<Root>/Mx_total' incorporates:
   *  Gain: '<S3>/Gain'
   *  MATLAB Function: '<S3>/MATLAB Function1'
   *  UnitDelay: '<S3>/Unit Delay'
   */
  *Torque_Vectoring_Y_Mx_total = ((Torque_Vectoring_B->Mx_coeff[0] *
    Torque_Vectoring_Y_Fx_opt[0] + Torque_Vectoring_B->Mx_coeff[1] *
    Torque_Vectoring_Y_Fx_opt[1]) + Torque_Vectoring_B->Mx_coeff[2] *
    Torque_Vectoring_Y_Fx_opt[2]) + Torque_Vectoring_B->Mx_coeff[3] *
    Torque_Vectoring_Y_Fx_opt[3];

  /* Outport: '<Root>/Ca' */
  Torque_Vectoring_Y_Ca[0] = Torque_Vectoring_B->front;
  Torque_Vectoring_Y_Ca[1] = Torque_Vectoring_B->rear;

  /* Update for DiscreteIntegrator: '<S50>/Integrator' incorporates:
   *  Gain: '<S47>/Integral Gain'
   */
  Torque_Vectoring_DW->Integrator_DSTATE += 2.0 * Torque_Vectoring_B->Sum1 *
    0.002;

  /* Update for DiscreteIntegrator: '<S45>/Filter' */
  Torque_Vectoring_DW->Filter_DSTATE += 0.002 *
    Torque_Vectoring_B->FilterCoefficient;

  /* Update for UnitDelay: '<S71>/UD'
   *
   * Block description for '<S71>/UD':
   *
   *  Store in Global RAM
   */
  Torque_Vectoring_DW->UD_DSTATE = Torque_Vectoring_B->TSamp;

  /* Update for UnitDelay: '<S72>/UD'
   *
   * Block description for '<S72>/UD':
   *
   *  Store in Global RAM
   */
  Torque_Vectoring_DW->UD_DSTATE_l = Torque_Vectoring_B->Gain2;

  /* Update for UnitDelay: '<S73>/UD'
   *
   * Block description for '<S73>/UD':
   *
   *  Store in Global RAM
   */
  Torque_Vectoring_DW->UD_DSTATE_j = 0.0;
}

/* Model initialize function */
void Torque_Vectoring_initialize(RT_MODEL_Torque_Vectoring_T *const
  Torque_Vectoring_M, real_T *Torque_Vectoring_U_Vx_des, real_T
  *Torque_Vectoring_U_Vx, real_T *Torque_Vectoring_U_Vy, real_T
  Torque_Vectoring_U_AngWheel[4], real_T *Torque_Vectoring_U_r, real_T
  Torque_Vectoring_U_Fy[4], real_T Torque_Vectoring_U_Fz[4], real_T
  Torque_Vectoring_Y_Tm[4], real_T Torque_Vectoring_Y_Fx_opt[4], real_T
  *Torque_Vectoring_Y_Mx_total, real_T *Torque_Vectoring_Y_Fx_total, real_T
  *Torque_Vectoring_Y_Mzd, real_T *Torque_Vectoring_Y_r_des, real_T
  *Torque_Vectoring_Y_beta_des, real_T Torque_Vectoring_Y_Ca[2], real_T
  *Torque_Vectoring_Y_beta)
{
  DW_Torque_Vectoring_T *Torque_Vectoring_DW = Torque_Vectoring_M->dwork;

  /* Registration code */

  /* states (dwork) */
  (void) memset((void *)Torque_Vectoring_DW, 0,
                sizeof(DW_Torque_Vectoring_T));

  /* external inputs */
  (void)memset(&Torque_Vectoring_U_AngWheel[0], 0, sizeof(real_T) << 2U);
  (void)memset(&Torque_Vectoring_U_Fy[0], 0, sizeof(real_T) << 2U);
  (void)memset(&Torque_Vectoring_U_Fz[0], 0, sizeof(real_T) << 2U);
  *Torque_Vectoring_U_Vx_des = 0.0;
  *Torque_Vectoring_U_Vx = 0.0;
  *Torque_Vectoring_U_Vy = 0.0;
  *Torque_Vectoring_U_r = 0.0;

  /* external outputs */
  (void)memset(&Torque_Vectoring_Y_Tm[0], 0, sizeof(real_T) << 2U);
  (void)memset(&Torque_Vectoring_Y_Fx_opt[0], 0, sizeof(real_T) << 2U);
  (void)memset(&Torque_Vectoring_Y_Ca[0], 0, sizeof(real_T) << 1U);
  *Torque_Vectoring_Y_Mx_total = 0.0;
  *Torque_Vectoring_Y_Fx_total = 0.0;
  *Torque_Vectoring_Y_Mzd = 0.0;
  *Torque_Vectoring_Y_r_des = 0.0;
  *Torque_Vectoring_Y_beta_des = 0.0;
  *Torque_Vectoring_Y_beta = 0.0;
  Torque_Vectoring_DW->iA0_mem.size = 0;

  /* SystemInitialize for MATLAB Function: '<S3>/MATLAB Function1' */
  Torque_Vectoring_DW->iA0_mem_not_empty = false;

  /* Start for MATLABSystem: '<S69>/Median Filter' */
  Torque_Vectoring_DW->obj.matlabCodegenIsDeleted = false;
  Torque_Vectoring_DW->obj.isInitialized = 1;
  Torque_Vectoring_DW->obj.NumChannels = 1;
  Torque_Vectoring_DW->obj.pMID.isInitialized = 0;
  Torque_Vectoring_DW->obj.isSetupComplete = true;

  /* InitializeConditions for MATLABSystem: '<S69>/Median Filter' */
  if (Torque_Vectoring_DW->obj.pMID.isInitialized == 1) {
    Torque_V_MedianFilter_resetImpl(&Torque_Vectoring_DW->obj.pMID);
  }

  /* End of InitializeConditions for MATLABSystem: '<S69>/Median Filter' */

  /* ConstCode for Outport: '<Root>/beta_des' incorporates:
   *  Constant: '<S4>/Zero'
   */
  *Torque_Vectoring_Y_beta_des = 0.0;
}

/* Model terminate function */
void Torque_Vectoring_terminate(RT_MODEL_Torque_Vectoring_T *const
  Torque_Vectoring_M)
{
  DW_Torque_Vectoring_T *Torque_Vectoring_DW = Torque_Vectoring_M->dwork;

  /* Terminate for MATLABSystem: '<S69>/Median Filter' */
  if (!Torque_Vectoring_DW->obj.matlabCodegenIsDeleted) {
    Torque_Vectoring_DW->obj.matlabCodegenIsDeleted = true;
    if ((Torque_Vectoring_DW->obj.isInitialized == 1) &&
        Torque_Vectoring_DW->obj.isSetupComplete) {
      Torque_Vectoring_DW->obj.NumChannels = -1;
      if (Torque_Vectoring_DW->obj.pMID.isInitialized == 1) {
        Torque_Vectoring_DW->obj.pMID.isInitialized = 2;
      }
    }
  }

  /* End of Terminate for MATLABSystem: '<S69>/Median Filter' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */

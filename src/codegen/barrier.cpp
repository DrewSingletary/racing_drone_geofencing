//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: barrier.cpp
//
// MATLAB Coder version            : 4.2
// C/C++ source code generated on  : 26-Nov-2021 16:04:37
//

// Include Files
#include <cmath>
#include <string.h>
#include <math.h>
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "barrier.h"
#include <stdio.h>
#include <stdlib.h>

// Type Definitions
typedef struct {
  int lineNo;
  int colNo;
  const char * fName;
  const char * pName;
} rtRunTimeErrorInfo;

// Variable Definitions
static rtRunTimeErrorInfo emlrtRTEI = { 12,// lineNo
  9,                                   // colNo
  "sqrt",                              // fName
  "/Applications/MATLAB_R2019a.app/toolbox/eml/lib/matlab/elfun/sqrt.m"// pName
};

static rtRunTimeErrorInfo b_emlrtRTEI = { 13,// lineNo
  9,                                   // colNo
  "asin",                              // fName
  "/Applications/MATLAB_R2019a.app/toolbox/eml/lib/matlab/elfun/asin.m"// pName
};

static const double dv0[4] = { 0.0, 0.0, 0.0, 1.0 };

// Function Declarations
static void backup_controller(const double x[13], double hover_height, double
  yaw0, double u[4]);
static void normalizeRows(const double matrix[4], double normRowMatrix[4]);
static void quat2eul(double q[4], double eul[3]);
static void quatmultiply(const double q[4], const double r[4], double qout[4]);
static void rtErrorWithMessageID(const int b, const char *c, const
  rtRunTimeErrorInfo *aInfo);
static void rtReportErrorLocation(const char * aFcnName, int aLineNo);
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : const double x[13]
//                double hover_height
//                double yaw0
//                double u[4]
// Return Type  : void
//
static void backup_controller(const double x[13], double hover_height, double
  yaw0, double u[4])
{
  double q[4];
  double tmp[3];
  double ct_idx_0;
  double ct_idx_1;
  double vwdx;
  double vwdy;
  double u0;
  double Rd[9];
  double dv3[4];
  double tempR[9];
  double tempR_tmp;
  double R[9];
  int k;
  int i1;
  double b_Rd[9];
  double b_R[9];
  int R_tmp;
  double b_q[4];
  int i2;
  int i3;
  int i4;
  int i5;
  double dv4[4];
  double dv5[4];
  q[0] = x[3];
  q[1] = x[4];
  q[2] = x[5];
  q[3] = x[6];
  quat2eul(q, tmp);
  ct_idx_0 = std::sin(tmp[0]);
  ct_idx_1 = std::cos(tmp[0]);
  q[0] = std::cos(tmp[0]);
  q[1] = -std::sin(tmp[0]);
  vwdx = 0.0;
  vwdy = 0.0;
  if (x[0] > 0.0) {
    vwdx = -x[0];
    if (!(vwdx < 1.0)) {
      vwdx = 1.0;
    }

    if (!(vwdx > -1.0)) {
      vwdx = -1.0;
    }
  }

  if (x[1] > 49.0) {
    vwdy = -(x[1] - 49.0);
    if (!(vwdy < 1.0)) {
      vwdy = 1.0;
    }

    if (!(vwdy > -1.0)) {
      vwdy = -1.0;
    }
  }

  //      rolld = saturate(-K*vyError,-pi/3,pi/3);
  //      pitchd = saturate(K*vxError,-pi/3,pi/3);
  //      rolld = saturate(-K*vyError,-pi/12,pi/12);
  //      pitchd = saturate(K*vxError,-pi/12,pi/12);
  if ((std::abs(tmp[1]) > 1.5707963267948966) || (std::abs(tmp[2]) >
       1.5707963267948966)) {
    memset(&Rd[0], 0, 9U * sizeof(double));
    Rd[0] = 1.0;
    Rd[4] = 1.0;
    Rd[8] = 1.0;
  } else {
    u0 = 0.15 * ((q[0] * vwdx + ct_idx_0 * vwdy) - (q[0] * x[7] + ct_idx_0 * x[8]));
    if (!(u0 < 1.4959965017094252)) {
      u0 = 1.4959965017094252;
    }

    if (!(u0 > -1.4959965017094252)) {
      u0 = -1.4959965017094252;
    }

    tmp[1] = u0;
    u0 = -0.15 * ((q[1] * vwdx + ct_idx_1 * vwdy) - (q[1] * x[7] + ct_idx_1 * x
      [8]));
    if (!(u0 < 1.4959965017094252)) {
      u0 = 1.4959965017094252;
    }

    if (!(u0 > -1.4959965017094252)) {
      u0 = -1.4959965017094252;
    }

    tmp[2] = u0;
    ct_idx_0 = std::cos(yaw0);
    tmp[0] = std::sin(yaw0);
    ct_idx_1 = std::cos(tmp[1]);
    tmp[1] = std::sin(tmp[1]);
    vwdx = std::cos(tmp[2]);
    tmp[2] = std::sin(tmp[2]);
    Rd[0] = ct_idx_1 * ct_idx_0;
    vwdy = tmp[2] * tmp[1];
    Rd[3] = vwdy * ct_idx_0 - vwdx * tmp[0];
    u0 = vwdx * tmp[1];
    Rd[6] = u0 * ct_idx_0 + tmp[2] * tmp[0];
    Rd[1] = ct_idx_1 * tmp[0];
    Rd[4] = vwdy * tmp[0] + vwdx * ct_idx_0;
    Rd[7] = u0 * tmp[0] - tmp[2] * ct_idx_0;
    Rd[2] = -tmp[1];
    Rd[5] = tmp[2] * ct_idx_1;
    Rd[8] = vwdx * ct_idx_1;
  }

  //      Rd = eye(3);
  normalizeRows(*(double (*)[4])&x[3], dv3);
  vwdx = dv3[3] * dv3[3];
  vwdy = dv3[2] * dv3[2];
  tempR[0] = 1.0 - 2.0 * (vwdy + vwdx);
  u0 = dv3[1] * dv3[2];
  ct_idx_0 = dv3[0] * dv3[3];
  tempR[1] = 2.0 * (u0 - ct_idx_0);
  ct_idx_1 = dv3[1] * dv3[3];
  tempR_tmp = dv3[0] * dv3[2];
  tempR[2] = 2.0 * (ct_idx_1 + tempR_tmp);
  tempR[3] = 2.0 * (u0 + ct_idx_0);
  u0 = dv3[1] * dv3[1];
  tempR[4] = 1.0 - 2.0 * (u0 + vwdx);
  vwdx = dv3[2] * dv3[3];
  ct_idx_0 = dv3[0] * dv3[1];
  tempR[5] = 2.0 * (vwdx - ct_idx_0);
  tempR[6] = 2.0 * (ct_idx_1 - tempR_tmp);
  tempR[7] = 2.0 * (vwdx + ct_idx_0);
  tempR[8] = 1.0 - 2.0 * (u0 + vwdy);
  memcpy(&R[0], &tempR[0], 9U * sizeof(double));
  for (k = 0; k < 3; k++) {
    R[k] = tempR[3 * k];
    R[k + 3] = tempR[3 * k + 1];
    R[k + 6] = tempR[3 * k + 2];
  }

  for (k = 0; k < 3; k++) {
    for (i1 = 0; i1 < 3; i1++) {
      R_tmp = k + 3 * i1;
      b_R[R_tmp] = 0.0;
      i2 = 1 + 3 * k;
      i3 = 1 + 3 * i1;
      i4 = 2 + 3 * k;
      i5 = 2 + 3 * i1;
      b_R[R_tmp] = (R[3 * k] * Rd[3 * i1] + R[i2] * Rd[i3]) + R[i4] * Rd[i5];
      b_Rd[R_tmp] = (Rd[3 * k] * R[3 * i1] + Rd[i2] * R[i3]) + Rd[i4] * R[i5];
    }
  }

  for (k = 0; k < 9; k++) {
    ct_idx_0 = 0.5 * (b_Rd[k] - b_R[k]);
    b_Rd[k] = ct_idx_0;
    tempR[k] = ct_idx_0;
  }

  //  inverse of standard Matlab quatrotate
  ct_idx_0 = ((x[3] * x[3] + x[4] * x[4]) + x[5] * x[5]) + x[6] * x[6];
  b_q[0] = x[3] / ct_idx_0;
  b_q[1] = -x[4] / ct_idx_0;
  b_q[2] = -x[5] / ct_idx_0;
  b_q[3] = -x[6] / ct_idx_0;
  quatmultiply(*(double (*)[4])&x[3], dv0, dv3);
  q[0] = dv3[0];
  q[1] = dv3[1];
  q[2] = dv3[2];
  q[3] = dv3[3];
  quatmultiply(q, b_q, dv4);
  vwdx = dv4[3];
  quatmultiply(*(double (*)[4])&x[3], dv0, dv3);
  q[0] = dv3[0];
  q[1] = dv3[1];
  q[2] = dv3[2];
  q[3] = dv3[3];
  quatmultiply(q, b_q, dv5);
  tmp[2] = dv5[3];
  if ((vwdx < 0.5) && (vwdx >= 0.0)) {
    tmp[2] = 0.5;
  }

  if (tmp[2] < 0.0) {
    tmp[2] = 3.0;
  }

  u0 = 0.1 * (hover_height - x[2]);
  if (!(u0 < 0.3)) {
    u0 = 0.3;
  }

  if (!(u0 > -0.3)) {
    u0 = -0.3;
  }

  ct_idx_0 = 0.05 * (0.0 - x[9]);
  if (!(ct_idx_0 < 0.3)) {
    ct_idx_0 = 0.3;
  }

  if (!(ct_idx_0 > -0.3)) {
    ct_idx_0 = -0.3;
  }

  u0 = (0.32 / tmp[2] + u0) + ct_idx_0;
  if (!(u0 < 1.0)) {
    u0 = 1.0;
  }

  if (!(u0 > 0.0)) {
    u0 = 0.0;
  }

  u[0] = u0;

  //      u(1,1) = hoverThrust + pterm + dterm;
  u[1] = -10.0 * tempR[5];
  u[2] = -10.0 * tempR[6];
  u[3] = -10.0 * tempR[1];
}

//
// Arguments    : const double matrix[4]
//                double normRowMatrix[4]
// Return Type  : void
//
static void normalizeRows(const double matrix[4], double normRowMatrix[4])
{
  double y;
  y = ((matrix[0] * matrix[0] + matrix[1] * matrix[1]) + matrix[2] * matrix[2])
    + matrix[3] * matrix[3];
  if (y < 0.0) {
    rtErrorWithMessageID(4, "sqrt", &emlrtRTEI);
  }

  y = 1.0 / std::sqrt(y);
  normRowMatrix[0] = matrix[0] * y;
  normRowMatrix[1] = matrix[1] * y;
  normRowMatrix[2] = matrix[2] * y;
  normRowMatrix[3] = matrix[3] * y;
}

//
// Arguments    : double q[4]
//                double eul[3]
// Return Type  : void
//
static void quat2eul(double q[4], double eul[3])
{
  double b_q[4];
  double aSinInput;
  b_q[0] = q[0];
  b_q[1] = q[1];
  b_q[2] = q[2];
  b_q[3] = q[3];
  normalizeRows(b_q, q);
  aSinInput = -2.0 * (q[1] * q[3] - q[0] * q[2]);
  if (aSinInput > 1.0) {
    aSinInput = 1.0;
  }

  if (aSinInput < -1.0) {
    aSinInput = -1.0;
  }

  if ((aSinInput < -1.0) || (aSinInput > 1.0)) {
    rtErrorWithMessageID(4, "asin", &b_emlrtRTEI);
  }

  eul[0] = rt_atan2d_snf(2.0 * (q[1] * q[2] + q[0] * q[3]), ((q[0] * q[0] + q[1]
    * q[1]) - q[2] * q[2]) - q[3] * q[3]);
  eul[1] = std::asin(aSinInput);
  eul[2] = rt_atan2d_snf(2.0 * (q[2] * q[3] + q[0] * q[1]), ((q[0] * q[0] - q[1]
    * q[1]) - q[2] * q[2]) + q[3] * q[3]);
}

//
// Arguments    : const double q[4]
//                const double r[4]
//                double qout[4]
// Return Type  : void
//
static void quatmultiply(const double q[4], const double r[4], double qout[4])
{
  qout[0] = ((q[0] * r[0] - q[1] * r[1]) - q[2] * r[2]) - q[3] * r[3];
  qout[1] = (q[0] * r[1] + r[0] * q[1]) + (q[2] * r[3] - q[3] * r[2]);
  qout[2] = (q[0] * r[2] + r[0] * q[2]) + (q[3] * r[1] - q[1] * r[3]);
  qout[3] = (q[0] * r[3] + r[0] * q[3]) + (q[1] * r[2] - q[2] * r[1]);
}

//
// Arguments    : const int b
//                const char *c
//                const rtRunTimeErrorInfo *aInfo
// Return Type  : void
//
static void rtErrorWithMessageID(const int b, const char *c, const
  rtRunTimeErrorInfo *aInfo)
{
  fprintf(stderr,
          "Domain error. To compute complex results from real x, use \'%.*s(complex(x))\'.",
          b, c);
  fprintf(stderr, "\n");
  if (aInfo != NULL) {
    rtReportErrorLocation(aInfo->fName, aInfo->lineNo);
  }

  fflush(stderr);
  abort();
}

//
// Arguments    : const char * aFcnName
//                int aLineNo
// Return Type  : void
//
static void rtReportErrorLocation(const char * aFcnName, int aLineNo)
{
  fprintf(stderr, "Error in %s (line %d)", aFcnName, aLineNo);
  fprintf(stderr, "\n");
}

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  int b_u0;
  int b_u1;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    if (u0 > 0.0) {
      b_u0 = 1;
    } else {
      b_u0 = -1;
    }

    if (u1 > 0.0) {
      b_u1 = 1;
    } else {
      b_u1 = -1;
    }

    y = atan2((double)b_u0, (double)b_u1);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

//
// Arguments    : double x[13]
//                const double u_des[4]
//                double u[4]
//                double h[101]
//                double *lambda
// Return Type  : void
//
void barrier(double x[13], const double u_des[4], double u[4], double h[101],
             double *lambda)
{
  double hover_height;
  double u_tmp[4];
  double tmp[3];
  double x0[13];
  double ub[4];
  int idx;
  boolean_T b0;
  double xDot[13];
  int k;
  boolean_T exitg1;
  int i0;
  double v_pos;
  double q[4];
  double qnrm;
  double b_q[4];
  double dv1[4];
  double dv2[4];
  memset(&h[0], 0, 101U * sizeof(double));

  //      h = -(x(1))^2-(x(2))^2+1^2;
  //      h = -(x(1))^2+10^2;
  //      if x(1) < 0
  //          h = (x(1))^2+10^2;
  //      end
  if ((1.0 - x[0] < 1.0 - x[1]) || rtIsNaN(1.0 - x[1])) {
    h[0] = 1.0 - x[0];
  } else {
    h[0] = 1.0 - x[1];
  }

  hover_height = x[2];
  u_tmp[0] = x[3];
  u_tmp[1] = x[4];
  u_tmp[2] = x[5];
  u_tmp[3] = x[6];
  quat2eul(u_tmp, tmp);
  memcpy(&x0[0], &x[0], 13U * sizeof(double));
  backup_controller(x, x[2], tmp[0], ub);
  for (idx = 0; idx < 100; idx++) {
    backup_controller(x, hover_height, tmp[0], u_tmp);

    // states: x,y,z,qw,qx,qy,qz,vx,vy,vx,ox,oy,oz
    memset(&xDot[0], 0, 13U * sizeof(double));
    xDot[0] = x[7];
    xDot[1] = x[8];
    xDot[2] = x[9];

    //  ox = u(2); oy = u(3); oz = u(4);
    q[0] = x[3];
    q[1] = x[4];
    q[2] = x[5];
    q[3] = x[6];
    xDot[3] = 0.5 * (0.0 * x[3] - ((x[10] * x[4] + x[11] * x[5]) + x[12] * x[6]));
    xDot[4] = 0.5 * ((x[3] * x[10] + 0.0 * x[4]) + (x[5] * x[12] - x[6] * x[11]));
    xDot[5] = 0.5 * ((x[3] * x[11] + 0.0 * x[5]) + (x[6] * x[10] - x[4] * x[12]));
    xDot[6] = 0.5 * ((x[3] * x[12] + 0.0 * x[6]) + (x[4] * x[11] - x[5] * x[10]));
    v_pos = u_tmp[0] * 1639.0 + 172.0;
    v_pos = 2.7508051E-5 * (v_pos * v_pos) - 0.0047313846 * v_pos;

    //  inverse of standard Matlab quatrotate
    qnrm = ((x[3] * x[3] + x[4] * x[4]) + x[5] * x[5]) + x[6] * x[6];
    b_q[0] = x[3] / qnrm;
    b_q[1] = -x[4] / qnrm;
    b_q[2] = -x[5] / qnrm;
    b_q[3] = -x[6] / qnrm;
    quatmultiply(q, dv0, dv1);
    quatmultiply(dv1, b_q, dv2);
    xDot[7] = v_pos * dv2[1];
    xDot[10] = 50.0 * (u_tmp[1] - x[10]);
    xDot[8] = v_pos * dv2[2];
    xDot[11] = 50.0 * (u_tmp[2] - x[11]);
    xDot[9] = v_pos * dv2[3] - 9.81;
    xDot[12] = 50.0 * (u_tmp[3] - x[12]);
    for (i0 = 0; i0 < 13; i0++) {
      x[i0] += xDot[i0] * 0.03;
    }

    //      h = -(x(1))^2-(x(2))^2+1^2;
    //      h = -(x(1))^2+10^2;
    //      if x(1) < 0
    //          h = (x(1))^2+10^2;
    //      end
    if ((1.0 - x[0] < 1.0 - x[1]) || rtIsNaN(1.0 - x[1])) {
      h[1 + idx] = 1.0 - x[0];
    } else {
      h[1 + idx] = 1.0 - x[1];
    }
  }

  b0 = rtIsNaN(h[0]);
  if (!b0) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 102)) {
      if (!rtIsNaN(h[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    hover_height = h[0];
  } else {
    hover_height = h[idx - 1];
    i0 = idx + 1;
    for (k = i0; k < 102; k++) {
      qnrm = h[k - 1];
      if (hover_height > qnrm) {
        hover_height = qnrm;
      }
    }
  }

  v_pos = (x[7] * x[7] + x[8] * x[8]) + x[9] * x[9];
  if (v_pos < 0.0) {
    rtErrorWithMessageID(4, "sqrt", &emlrtRTEI);
  }

  v_pos = std::sqrt(v_pos);
  v_pos = 40.0 * (-v_pos + 1.0);
  if ((!(hover_height < v_pos)) && (!rtIsNaN(v_pos))) {
    hover_height = v_pos;
  }

  if (hover_height < 0.0) {
    hover_height = 0.0;
  }

  if (std::abs(x[0] - 1.0) < std::abs(x[1] - 50.0)) {
    v_pos = std::abs(x0[7]);
    if (!(v_pos < 10.0)) {
      v_pos = 10.0;
    }

    if (!(v_pos > 1.0)) {
      v_pos = 1.0;
    }

    // x barrier
  } else {
    v_pos = std::abs(x0[8]);
    if (!(v_pos < 10.0)) {
      v_pos = 10.0;
    }

    if (!(v_pos > 1.0)) {
      v_pos = 1.0;
    }

    // y barrier
  }

  //      lambda = 1-exp(-3*(min_h^2/(100*v_pos))); %x barrier
  v_pos = std::exp(-3.0 * (hover_height / (2.0 * v_pos)));
  *lambda = 1.0 - v_pos;

  // x barrier
  u[0] = (1.0 - v_pos) * u_des[0] + (1.0 - (1.0 - v_pos)) * ub[0];
  u[1] = (1.0 - v_pos) * u_des[1] + (1.0 - (1.0 - v_pos)) * ub[1];
  u[2] = (1.0 - v_pos) * u_des[2] + (1.0 - (1.0 - v_pos)) * ub[2];
  u[3] = (1.0 - v_pos) * u_des[3] + (1.0 - (1.0 - v_pos)) * ub[3];
  if (!b0) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 102)) {
      if (!rtIsNaN(h[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    v_pos = h[0];
  } else {
    v_pos = h[idx - 1];
    i0 = idx + 1;
    for (k = i0; k < 102; k++) {
      qnrm = h[k - 1];
      if (v_pos > qnrm) {
        v_pos = qnrm;
      }
    }
  }

  if (v_pos < 0.0) {
    u[0] = ub[0];
    u[1] = ub[1];
    u[2] = ub[2];
    u[3] = ub[3];
    *lambda = 0.0;
  }
}

//
// Arguments    : void
// Return Type  : void
//
void barrier_initialize()
{
  rt_InitInfAndNaN(8U);
}

//
// Arguments    : void
// Return Type  : void
//
void barrier_terminate()
{
  // (no terminate code required)
}

//
// File trailer for barrier.cpp
//
// [EOF]
//

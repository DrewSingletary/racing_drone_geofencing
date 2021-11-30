//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: zBodyInWorld.cpp
//
// MATLAB Coder version            : 4.2
// C/C++ source code generated on  : 01-Sep-2021 16:31:59
//

// Include Files
#include "zBodyInWorld.h"

// Function Declarations
static void quatmultiply(const double q[4], const double r[4], double qout[4]);

// Function Definitions

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
// Arguments    : const double in[4]
// Return Type  : double
//
double zBodyInWorld(const double in[4])
{
  double qnrm;
  double q[4];
  static const double dv0[4] = { 0.0, 0.0, 0.0, 1.0 };

  double dv1[4];
  double dv2[4];

  //  inverse of standard Matlab quatrotate
  qnrm = ((in[0] * in[0] + in[1] * in[1]) + in[2] * in[2]) + in[3] * in[3];
  q[0] = in[0] / qnrm;
  q[1] = -in[1] / qnrm;
  q[2] = -in[2] / qnrm;
  q[3] = -in[3] / qnrm;
  quatmultiply(in, dv0, dv1);
  quatmultiply(dv1, q, dv2);
  return dv2[3];
}

//
// Arguments    : void
// Return Type  : void
//
void zBodyInWorld_initialize()
{
}

//
// Arguments    : void
// Return Type  : void
//
void zBodyInWorld_terminate()
{
  // (no terminate code required)
}

//
// File trailer for zBodyInWorld.cpp
//
// [EOF]
//

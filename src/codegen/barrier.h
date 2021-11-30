//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: barrier.h
//
// MATLAB Coder version            : 4.2
// C/C++ source code generated on  : 26-Nov-2021 15:26:14
//
#ifndef BARRIER_H
#define BARRIER_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "barrier_types.h"

// Function Declarations
extern void barrier(double x[13], const double u_des[4], double u[4], double h
                    [101], double *lambda);
extern void barrier_initialize();
extern void barrier_terminate();
extern void quatmultiply(const double q[4], const double r[4], double qout[4]);

#endif

//
// File trailer for barrier.h
//
// [EOF]
//

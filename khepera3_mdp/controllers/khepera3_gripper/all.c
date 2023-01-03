/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: all.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 19-Nov-2022 21:58:16
 */

/* Include Files */
#include "all.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const boolean_T x[16]
 *                boolean_T y[4]
 * Return Type  : void
 */
void all(const boolean_T x[16], boolean_T y[4])
{
  int ix;
  boolean_T exitg1;
  y[0] = true;
  y[1] = true;
  y[2] = true;
  y[3] = true;
  ix = 1;
  exitg1 = false;
  while ((!exitg1) && (ix <= 4)) {
    if (!x[ix - 1]) {
      y[0] = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  ix = 5;
  exitg1 = false;
  while ((!exitg1) && (ix <= 8)) {
    if (!x[ix - 1]) {
      y[1] = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  ix = 9;
  exitg1 = false;
  while ((!exitg1) && (ix <= 12)) {
    if (!x[ix - 1]) {
      y[2] = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
  ix = 13;
  exitg1 = false;
  while ((!exitg1) && (ix <= 16)) {
    if (!x[ix - 1]) {
      y[3] = false;
      exitg1 = true;
    } else {
      ix++;
    }
  }
}

/*
 * Arguments    : const boolean_T x[2]
 * Return Type  : boolean_T
 */
boolean_T b_all(const boolean_T x[2])
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }
  return y;
}

/*
 * File trailer for all.c
 *
 * [EOF]
 */

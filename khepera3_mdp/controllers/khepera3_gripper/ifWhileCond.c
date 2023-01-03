/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: ifWhileCond.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 19-Nov-2022 21:58:16
 */

/* Include Files */
#include "ifWhileCond.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const boolean_T x_data[]
 *                const int x_size[2]
 * Return Type  : boolean_T
 */
boolean_T b_ifWhileCond(const boolean_T x_data[], const int x_size[2])
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = (x_size[1] != 0);
  if (y) {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= x_size[1] - 1)) {
      if (!x_data[k]) {
        y = false;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }
  return y;
}

/*
 * Arguments    : const boolean_T x[4]
 * Return Type  : boolean_T
 */
boolean_T ifWhileCond(const boolean_T x[4])
{
  int k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 4)) {
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
 * File trailer for ifWhileCond.c
 *
 * [EOF]
 */

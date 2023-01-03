/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nnz.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 19-Nov-2022 21:58:16
 */

/* Include Files */
#include "nnz.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const double s[4]
 * Return Type  : int
 */
int intnnz(const double s[4])
{
  int n;
  n = 0;
  if (s[0] != 0.0) {
    n = 1;
  }
  if (s[1] != 0.0) {
    n++;
  }
  if (s[2] != 0.0) {
    n++;
  }
  if (s[3] != 0.0) {
    n++;
  }
  return n;
}

/*
 * File trailer for nnz.c
 *
 * [EOF]
 */

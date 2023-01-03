/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: nonSingletonDim.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 19-Nov-2022 21:58:16
 */

/* Include Files */
#include "nonSingletonDim.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const int x_size[2]
 * Return Type  : int
 */
int nonSingletonDim(const int x_size[2])
{
  int dim;
  if (x_size[0] != 1) {
    dim = 1;
  } else {
    dim = 2;
  }
  return dim;
}

/*
 * File trailer for nonSingletonDim.c
 *
 * [EOF]
 */

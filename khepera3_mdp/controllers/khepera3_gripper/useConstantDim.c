/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: useConstantDim.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 19-Nov-2022 21:58:16
 */

/* Include Files */
#include "useConstantDim.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : double varargin_2_data[]
 *                const int varargin_2_size[2]
 *                int varargin_3
 * Return Type  : void
 */
void useConstantDim(double varargin_2_data[], const int varargin_2_size[2],
                    int varargin_3)
{
  int b_k;
  int i;
  int i1;
  int k;
  signed char subsb_idx_1;
  if (1 == varargin_3) {
    if (varargin_2_size[0] != 0) {
      i = varargin_2_size[0];
      for (k = 0; k < 4; k++) {
        if (0 <= i - 2) {
          subsb_idx_1 = (signed char)(k + 1);
        }
        for (b_k = 0; b_k <= i - 2; b_k++) {
          i1 = ((signed char)((signed char)(b_k + 1) + 1) +
                varargin_2_size[0] * (subsb_idx_1 - 1)) -
               1;
          varargin_2_data[i1] += varargin_2_data[b_k + varargin_2_size[0] * k];
        }
      }
    }
  } else if (varargin_2_size[0] != 0) {
    i = varargin_2_size[0];
    for (k = 0; k < 3; k++) {
      for (b_k = 0; b_k < i; b_k++) {
        i1 = ((signed char)(b_k + 1) + varargin_2_size[0] * (k + 1)) - 1;
        varargin_2_data[i1] += varargin_2_data[b_k + varargin_2_size[0] * k];
      }
    }
  }
}

/*
 * File trailer for useConstantDim.c
 *
 * [EOF]
 */

/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: find.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 19-Nov-2022 21:58:16
 */

/* Include Files */
#include "find.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : const boolean_T x_data[]
 *                const int x_size[2]
 *                int i_data[]
 *                int i_size[2]
 * Return Type  : void
 */
void b_eml_find(const boolean_T x_data[], const int x_size[2], int i_data[],
                int i_size[2])
{
  int idx;
  int ii;
  boolean_T exitg1;
  ii = x_size[1];
  idx = 0;
  i_size[0] = 1;
  i_size[1] = 1;
  exitg1 = false;
  while ((!exitg1) && (ii > 0)) {
    if (x_data[ii - 1]) {
      idx = 1;
      i_data[0] = ii;
      exitg1 = true;
    } else {
      ii--;
    }
  }
  if (idx == 0) {
    i_size[0] = 1;
    i_size[1] = 0;
  }
}

/*
 * Arguments    : const boolean_T x[4]
 *                int i_data[]
 *                int i_size[2]
 * Return Type  : void
 */
void eml_find(const boolean_T x[4], int i_data[], int i_size[2])
{
  int idx;
  int ii;
  boolean_T exitg1;
  idx = 0;
  i_size[0] = 1;
  ii = 0;
  exitg1 = false;
  while ((!exitg1) && (ii < 4)) {
    if (x[ii]) {
      idx++;
      i_data[idx - 1] = ii + 1;
      if (idx >= 4) {
        exitg1 = true;
      } else {
        ii++;
      }
    } else {
      ii++;
    }
  }
  if (1 > idx) {
    i_size[1] = 0;
  } else {
    i_size[1] = idx;
  }
}

/*
 * File trailer for find.c
 *
 * [EOF]
 */

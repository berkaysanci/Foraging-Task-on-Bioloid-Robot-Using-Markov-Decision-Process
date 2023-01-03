/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: mdp_run_run_initialize.c
 *
 * MATLAB Coder version            : 5.3
 * C/C++ source code generated on  : 19-Nov-2022 21:58:16
 */

/* Include Files */
#include "mdp_run_run_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "mdp_run_run_data.h"
#include "rt_nonfinite.h"

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : void
 */
void mdp_run_run_initialize(void)
{
  rt_InitInfAndNaN();
  c_eml_rand_mt19937ar_stateful_i();
  isInitialized_mdp_run_run = true;
}

/*
 * File trailer for mdp_run_run_initialize.c
 *
 * [EOF]
 */

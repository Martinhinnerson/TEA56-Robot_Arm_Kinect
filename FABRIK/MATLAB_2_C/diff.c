/*
 * File: diff.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 25-Apr-2016 17:23:47
 */

/* Include files */
#include "rt_nonfinite.h"
#include "FABRIK.h"
#include "main.h"
#include "my_intersection.h"
#include "n_rules.h"
#include "diff.h"

/* Function Definitions */

/*
 * Arguments    : const double x[4]
 *                double y[2]
 * Return Type  : void
 */
void diff(const double x[4], double y[2])
{
  int ixStart;
  int iyStart;
  int r;
  ixStart = 1;
  iyStart = 0;
  for (r = 0; r < 2; r++) {
    y[iyStart] = x[ixStart] - x[ixStart - 1];
    ixStart += 2;
    iyStart++;
  }
}

/*
 * File trailer for diff.c
 *
 * [EOF]
 */

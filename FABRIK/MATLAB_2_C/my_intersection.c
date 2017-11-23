/*
 * File: my_intersection.c
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
 *                const double y[4]
 *                double *xi
 *                double *yi
 * Return Type  : void
 */
void my_intersection(const double x[4], const double y[4], double *xi, double
                     *yi)
{
  double dx[2];
  double dy[2];
  double ua;
  diff(x, dx);

  /* # Take the differences down each column */
  diff(y, dy);

  /* # Precompute the denominator */
  ua = (dx[1] * (y[0] - y[2]) - dy[1] * (x[0] - x[2])) / (dx[0] * dy[1] - dy[0] *
    dx[1]);
  *xi = x[0] + ua * dx[0];
  *yi = y[0] + ua * dy[0];
}

/*
 * File trailer for my_intersection.c
 *
 * [EOF]
 */

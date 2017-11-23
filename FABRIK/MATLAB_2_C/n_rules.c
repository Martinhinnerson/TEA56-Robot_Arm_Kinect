/*
 * File: n_rules.c
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
#include "norm.h"

/* Function Definitions */

/*
 * Arguments    : const double p[2]
 *                const double p2[2]
 *                const double t[2]
 *                const double theta[2]
 *                double new_pos[2]
 * Return Type  : void
 */
void n_rules(const double p[2], const double p2[2], const double t[2], const
             double theta[2], double new_pos[2])
{
  double L1[2];
  double L2[2];
  int ix;
  double c;
  int iy;
  int k;
  double a[3];
  double b[3];
  double L4_idx_2;
  boolean_T guard1 = false;
  double dv2[4];
  double dv3[2];
  double b_p[4];
  double c_p[4];
  for (ix = 0; ix < 2; ix++) {
    new_pos[ix] = t[ix];
    L1[ix] = p[ix] - p2[ix];
    L2[ix] = t[ix] - p[ix];
  }

  c = 0.0;
  ix = 0;
  iy = 0;
  for (k = 0; k < 2; k++) {
    c += L2[ix] * L1[iy];
    ix++;
    iy++;

    /*  O =  ProjectPoint([p;p2],t)'; */
    /*  S = pdist([p;O],'euclidean'); */
    /*  L3 = t - O; */
    a[k] = L1[k];
    b[k] = L2[k];
  }

  L4_idx_2 = a[0] * b[1] - a[1] * b[0];
  guard1 = false;
  if (L4_idx_2 > 0.0) {
    L4_idx_2 = theta[0];
    guard1 = true;
  } else if (L4_idx_2 < 0.0) {
    L4_idx_2 = -theta[1];
    guard1 = true;
  } else {
    for (ix = 0; ix < 2; ix++) {
      new_pos[ix] = p[ix] + L1[ix];
    }
  }

  if (guard1) {
    if (acos(c / (norm(L2) * norm(L1))) < fabs(L4_idx_2)) {
    } else {
      dv2[0] = cos(L4_idx_2);
      dv2[2] = -sin(L4_idx_2);
      dv2[1] = sin(L4_idx_2);
      dv2[3] = cos(L4_idx_2);
      for (ix = 0; ix < 2; ix++) {
        dv3[ix] = 0.0;
        for (iy = 0; iy < 2; iy++) {
          dv3[ix] += dv2[ix + (iy << 1)] * L1[iy];
        }

        L2[ix] = dv3[ix] + p[ix];
      }

      for (ix = 0; ix < 2; ix++) {
        L1[ix] += t[ix];
      }

      b_p[0] = p[0];
      b_p[2] = t[0];
      b_p[1] = L2[0];
      b_p[3] = L1[0];
      c_p[0] = p[1];
      c_p[2] = t[1];
      c_p[1] = L2[1];
      c_p[3] = L1[1];
      my_intersection(b_p, c_p, &new_pos[0], &new_pos[1]);
    }
  }
}

/*
 * File trailer for n_rules.c
 *
 * [EOF]
 */

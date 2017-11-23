/*
 * File: FABRIK.c
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
 * Arguments    : double p[8]
 *                const double t[2]
 *                const double allowed_theta_f[8]
 *                double p_new[8]
 * Return Type  : void
 */
void FABRIK(double p[8], const double t[2], const double allowed_theta_f[8],
            double p_new[8])
{
  boolean_T b0;
  double d[3];
  int i;
  double b_p[2];
  int i0;
  double last_dif;
  double b[2];
  double dif;
  int i1;
  int i2;
  int i3;
  double dv0[2];
  double c_p[2];
  double b_allowed_theta_f[2];
  double dv1[2];
  double d_p[2];
  b0 = false;
  if ((t[1] < 0.0) || (norm(t) < 3.0)) {
    memcpy(&p_new[0], &p[0], sizeof(double) << 3);
  } else {
    for (i = 0; i < 3; i++) {
      for (i0 = 0; i0 < 2; i0++) {
        b_p[i0] = p[i + (i0 << 2)] - p[(i + (i0 << 2)) + 1];
      }

      d[i] = norm(b_p);
    }

    /* total_d = sum(d); */
    /* plot_arm(p,t,total_d); */
    for (i0 = 0; i0 < 2; i0++) {
      b_p[i0] = p[i0 << 2] - t[i0];
    }

    /* Om armen är utanför räckvidden, kör bara en iteration av forward reaching */
    last_dif = d[0];
    for (i = 0; i < 2; i++) {
      last_dif += d[i + 1];
    }

    if (norm(b_p) > last_dif) {
      for (i = 0; i < 3; i++) {
        for (i0 = 0; i0 < 2; i0++) {
          b_p[i0] = t[i0] - p[i + (i0 << 2)];
        }

        last_dif = d[i] / norm(b_p);
        for (i0 = 0; i0 < 2; i0++) {
          p[(i + (i0 << 2)) + 1] = (1.0 - last_dif) * p[i + (i0 << 2)] +
            last_dif * t[i0];
        }
      }
    } else {
      /* plot_arm(p,t,total_d); */
      last_dif = 30.0;
      for (i0 = 0; i0 < 2; i0++) {
        b[i0] = p[i0 << 2];
        b_p[i0] = p[3 + (i0 << 2)] - t[i0];
      }

      dif = norm(b_p);
      while ((dif > 0.01) && (last_dif - dif > 1.0E-6)) {
        /* plot_arm(p,t,total_d); */
        /* plot_arm(p,t,total_d); */
        for (i0 = 0; i0 < 2; i0++) {
          p[3 + (i0 << 2)] = t[i0];
          b_p[i0] = p[2 + (i0 << 2)] - p[3 + (i0 << 2)];
        }

        last_dif = d[2] / norm(b_p);
        for (i0 = 0; i0 < 2; i0++) {
          p[2 + (i0 << 2)] = (1.0 - last_dif) * p[3 + (i0 << 2)] + last_dif * p
            [2 + (i0 << 2)];
        }

        /* plot_arm(p,t,total_d); */
        for (i = 0; i < 2; i++) {
          /* p(i2,:) = follow_the_rules(p(i2+1,:),p(i2+2,:),p(i2,:),allowed_theta_b(i2+1,:)); */
          i0 = -i;
          i1 = -i;
          for (i2 = 0; i2 < 2; i2++) {
            b_p[i2] = p[(i0 + (i2 << 2)) + 1] - p[(i1 + (i2 << 2)) + 2];
          }

          last_dif = d[1 - i] / norm(b_p);
          i0 = -i;
          i1 = -i;
          i2 = -i;
          for (i3 = 0; i3 < 2; i3++) {
            p[(i0 + (i3 << 2)) + 1] = (1.0 - last_dif) * p[(i1 + (i3 << 2)) + 2]
              + last_dif * p[(i2 + (i3 << 2)) + 1];
          }

          /* plot_arm(p,t,total_d); */
        }

        /* plot_arm(p,t,total_d); */
        for (i0 = 0; i0 < 2; i0++) {
          p[i0 << 2] = b[i0];
          b_p[i0] = p[i0 << 2];
        }

        if (!b0) {
          for (i0 = 0; i0 < 2; i0++) {
            dv0[i0] = 0.0 - (double)i0;
          }

          b0 = true;
        }

        for (i0 = 0; i0 < 2; i0++) {
          c_p[i0] = p[1 + (i0 << 2)];
          b_allowed_theta_f[i0] = allowed_theta_f[i0 << 2];
        }

        n_rules(b_p, dv0, c_p, b_allowed_theta_f, dv1);
        for (i0 = 0; i0 < 2; i0++) {
          p[1 + (i0 << 2)] = dv1[i0];
          b_p[i0] = p[i0 << 2] - p[1 + (i0 << 2)];
        }

        last_dif = d[0] / norm(b_p);
        for (i0 = 0; i0 < 2; i0++) {
          p[1 + (i0 << 2)] = (1.0 - last_dif) * p[i0 << 2] + last_dif * p[1 +
            (i0 << 2)];
        }

        /*          plot_arm(p,t,total_d); */
        for (i = 0; i < 2; i++) {
          for (i0 = 0; i0 < 2; i0++) {
            b_p[i0] = p[(i + (i0 << 2)) + 1];
            c_p[i0] = p[i + (i0 << 2)];
            d_p[i0] = p[(i + (i0 << 2)) + 2];
            b_allowed_theta_f[i0] = allowed_theta_f[(i + (i0 << 2)) + 1];
          }

          n_rules(b_p, c_p, d_p, b_allowed_theta_f, dv1);
          for (i0 = 0; i0 < 2; i0++) {
            p[(i + (i0 << 2)) + 2] = dv1[i0];
            b_p[i0] = p[(i + (i0 << 2)) + 1] - p[(i + (i0 << 2)) + 2];
          }

          last_dif = d[i + 1] / norm(b_p);
          for (i0 = 0; i0 < 2; i0++) {
            p[(i + (i0 << 2)) + 2] = (1.0 - last_dif) * p[(i + (i0 << 2)) + 1] +
              last_dif * p[(i + (i0 << 2)) + 2];
          }

          /*              plot_arm(p,t,total_d); */
        }

        last_dif = dif;
        for (i0 = 0; i0 < 2; i0++) {
          b_p[i0] = p[3 + (i0 << 2)] - t[i0];
        }

        dif = norm(b_p);

        /* plot_arm(p,t,total_d); */
      }
    }

    /* plot_arm(p,t,total_d); */
    memcpy(&p_new[0], &p[0], sizeof(double) << 3);
  }
}

/*
 * File trailer for FABRIK.c
 *
 * [EOF]
 */

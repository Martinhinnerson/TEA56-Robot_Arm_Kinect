/*
 * File: main.c
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
#include <stdio.h>

/* Function Definitions */

/*
 * Arguments    : void
 * Return Type  : void
 */
void main(void)
{
double pos[8] = {0,0,1,1,2,2,3,3};
double t[2] = {4,3};
double theta[8] = {1.6,1.6,1.6,1.6,1.6,1.6,1.6,1.6};
double new_pos[8];

FABRIK(pos,t,theta,new_pos);

printf(new_pos);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */

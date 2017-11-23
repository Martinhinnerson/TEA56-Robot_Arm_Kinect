/* 
 * File: _coder_FABRIK_api.h 
 *  
 * MATLAB Coder version            : 2.6 
 * C/C++ source code generated on  : 25-Apr-2016 17:23:47 
 */

#ifndef ___CODER_FABRIK_API_H__
#define ___CODER_FABRIK_API_H__
/* Include files */
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"

/* Function Declarations */
extern void FABRIK_initialize(emlrtContext *aContext);
extern void FABRIK_terminate(void);
extern void FABRIK_atexit(void);
extern void FABRIK_api(const mxArray *prhs[3], const mxArray *plhs[1]);
extern void FABRIK(double p[8], double t[2], double allowed_theta_f[8], double p_new[8]);
extern void main_api(void);
extern void main(void);
extern void my_intersection_api(const mxArray *prhs[2], const mxArray *plhs[2]);
extern void my_intersection(double x[4], double y[4], double *xi, double *yi);
extern void n_rules_api(const mxArray *prhs[4], const mxArray *plhs[1]);
extern void n_rules(double p[2], double p2[2], double t[2], double theta[2], double new_pos[2]);
extern void FABRIK_xil_terminate(void);

#endif
/* 
 * File trailer for _coder_FABRIK_api.h 
 *  
 * [EOF] 
 */

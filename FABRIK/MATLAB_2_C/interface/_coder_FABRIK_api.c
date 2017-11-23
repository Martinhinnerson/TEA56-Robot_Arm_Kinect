/*
 * File: _coder_FABRIK_api.c
 *
 * MATLAB Coder version            : 2.6
 * C/C++ source code generated on  : 25-Apr-2016 17:23:47
 */

/* Include files */
#include "_coder_FABRIK_api.h"

/* Function Declarations */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *p, const
  char *identifier))[8];
static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[8];
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *t, const
  char *identifier))[2];
static double (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[2];
static const mxArray *emlrt_marshallOut(const double u[8]);
static double (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x, const
  char *identifier))[4];
static double (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId))[4];
static const mxArray *b_emlrt_marshallOut(const double u);
static const mxArray *c_emlrt_marshallOut(const double u[2]);
static double (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8];
static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2];
static double (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4];

/* Function Definitions */

/*
 * Arguments    : emlrtContext *aContext
 * Return Type  : void
 */
void FABRIK_initialize(emlrtContext *aContext)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, aContext, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void FABRIK_terminate(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void FABRIK_atexit(void)
{
  emlrtStack st = { NULL, NULL, NULL };

  emlrtCreateRootTLS(&emlrtRootTLSGlobal, &emlrtContextGlobal, NULL, 1);
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  FABRIK_xil_terminate();
}

/*
 * Arguments    : const mxArray *prhs[3]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void FABRIK_api(const mxArray *prhs[3], const mxArray *plhs[1])
{
  double (*p_new)[8];
  double (*p)[8];
  double (*t)[2];
  double (*allowed_theta_f)[8];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  p_new = (double (*)[8])mxMalloc(sizeof(double [8]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);

  /* Marshall function inputs */
  p = emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "p");
  t = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "t");
  allowed_theta_f = emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "allowed_theta_f");

  /* Invoke the target function */
  FABRIK(*p, *t, *allowed_theta_f, *p_new);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*p_new);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *p
 *                const char *identifier
 * Return Type  : double (*)[8]
 */
static double (*emlrt_marshallIn(const emlrtStack *sp, const mxArray *p, const
  char *identifier))[8]
{
  double (*y)[8];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = b_emlrt_marshallIn(sp, emlrtAlias(p), &thisId);
  emlrtDestroyArray(&p);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[8]
 */
  static double (*b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[8]
{
  double (*y)[8];
  y = g_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *t
 *                const char *identifier
 * Return Type  : double (*)[2]
 */
static double (*c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *t, const
  char *identifier))[2]
{
  double (*y)[2];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = d_emlrt_marshallIn(sp, emlrtAlias(t), &thisId);
  emlrtDestroyArray(&t);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[2]
 */
  static double (*d_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[2]
{
  double (*y)[2];
  y = h_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const double u[8]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const double u[8])
{
  const mxArray *y;
  static const int iv0[2] = { 0, 0 };

  const mxArray *m0;
  static const int iv1[2] = { 4, 2 };

  y = NULL;
  m0 = emlrtCreateNumericArray(2, iv0, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m0, (void *)u);
  emlrtSetDimensions((mxArray *)m0, iv1, 2);
  emlrtAssign(&y, m0);
  return y;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_api(void)
{
  /* Invoke the target function */
  main();
}

/*
 * Arguments    : const mxArray *prhs[2]
 *                const mxArray *plhs[2]
 * Return Type  : void
 */
void my_intersection_api(const mxArray *prhs[2], const mxArray *plhs[2])
{
  double (*x)[4];
  double (*y)[4];
  double yi;
  double xi;
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);

  /* Marshall function inputs */
  x = e_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "x");
  y = e_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "y");

  /* Invoke the target function */
  my_intersection(*x, *y, &xi, &yi);

  /* Marshall function outputs */
  plhs[0] = b_emlrt_marshallOut(xi);
  plhs[1] = b_emlrt_marshallOut(yi);
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *x
 *                const char *identifier
 * Return Type  : double (*)[4]
 */
static double (*e_emlrt_marshallIn(const emlrtStack *sp, const mxArray *x, const
  char *identifier))[4]
{
  double (*y)[4];
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = identifier;
  thisId.fParent = NULL;
  y = f_emlrt_marshallIn(sp, emlrtAlias(x), &thisId);
  emlrtDestroyArray(&x);
  return y;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : double (*)[4]
 */
  static double (*f_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u,
  const emlrtMsgIdentifier *parentId))[4]
{
  double (*y)[4];
  y = i_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const double u
 * Return Type  : const mxArray *
 */
static const mxArray *b_emlrt_marshallOut(const double u)
{
  const mxArray *y;
  const mxArray *m1;
  y = NULL;
  m1 = emlrtCreateDoubleScalar(u);
  emlrtAssign(&y, m1);
  return y;
}

/*
 * Arguments    : const mxArray *prhs[4]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void n_rules_api(const mxArray *prhs[4], const mxArray *plhs[1])
{
  double (*new_pos)[2];
  double (*p)[2];
  double (*p2)[2];
  double (*t)[2];
  double (*theta)[2];
  emlrtStack st = { NULL, NULL, NULL };

  st.tls = emlrtRootTLSGlobal;
  new_pos = (double (*)[2])mxMalloc(sizeof(double [2]));
  prhs[0] = emlrtProtectR2012b(prhs[0], 0, false, -1);
  prhs[1] = emlrtProtectR2012b(prhs[1], 1, false, -1);
  prhs[2] = emlrtProtectR2012b(prhs[2], 2, false, -1);
  prhs[3] = emlrtProtectR2012b(prhs[3], 3, false, -1);

  /* Marshall function inputs */
  p = c_emlrt_marshallIn(&st, emlrtAlias(prhs[0]), "p");
  p2 = c_emlrt_marshallIn(&st, emlrtAlias(prhs[1]), "p2");
  t = c_emlrt_marshallIn(&st, emlrtAlias(prhs[2]), "t");
  theta = c_emlrt_marshallIn(&st, emlrtAlias(prhs[3]), "theta");

  /* Invoke the target function */
  n_rules(*p, *p2, *t, *theta, *new_pos);

  /* Marshall function outputs */
  plhs[0] = c_emlrt_marshallOut(*new_pos);
}

/*
 * Arguments    : const double u[2]
 * Return Type  : const mxArray *
 */
static const mxArray *c_emlrt_marshallOut(const double u[2])
{
  const mxArray *y;
  static const int iv2[2] = { 0, 0 };

  const mxArray *m2;
  static const int iv3[2] = { 1, 2 };

  y = NULL;
  m2 = emlrtCreateNumericArray(2, iv2, mxDOUBLE_CLASS, mxREAL);
  mxSetData((mxArray *)m2, (void *)u);
  emlrtSetDimensions((mxArray *)m2, iv3, 2);
  emlrtAssign(&y, m2);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[8]
 */
static double (*g_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[8]
{
  double (*ret)[8];
  int iv4[2];
  int i0;
  for (i0 = 0; i0 < 2; i0++) {
    iv4[i0] = 4 + -2 * i0;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv4);
  ret = (double (*)[8])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[2]
 */
  static double (*h_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[2]
{
  double (*ret)[2];
  int iv5[2];
  int i1;
  for (i1 = 0; i1 < 2; i1++) {
    iv5[i1] = 1 + i1;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv5);
  ret = (double (*)[2])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : double (*)[4]
 */
static double (*i_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src,
  const emlrtMsgIdentifier *msgId))[4]
{
  double (*ret)[4];
  int iv6[2];
  int i;
  for (i = 0; i < 2; i++) {
    iv6[i] = 2;
  }

  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 2U, iv6);
  ret = (double (*)[4])mxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}
/*
 * File trailer for _coder_FABRIK_api.c
 *
 * [EOF]
 */

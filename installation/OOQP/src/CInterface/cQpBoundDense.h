/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef CQPBOUNDDENSE
#define CQPBOUNDDENSE

#include "cQpBound.h"

#ifdef __cplusplus
extern "C" {
#endif

  void qpboundsolvede( double    c[],  int  nx,       double Q[],
		       double xlow[],  char ixlow[], 
		       double xupp[],  char ixupp[],
		       double    x[],  double gamma[],     double phi[],
		       int * ierr );

  void QpBoundDenseGondzioSetup( double    c[],   int  nx,       double Q[],
			     double xlow[],   char ixlow[], 
			     double xupp[],   char ixupp[],
			     QpBoundContext * ctx,
			     int * ierr );

#ifdef __cplusplus
};
#endif

#endif

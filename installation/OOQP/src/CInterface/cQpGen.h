/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef CQPGEN
#define CQPGEN

#include "OoqpMonitorData.h"

/** A data structure for holding the various bits needed to solve
 *  a general QP. This structure is part of the CInterface
 *  to using QpGen */
typedef struct {
  void * factory;
  void * prob;
  void * solver;
} QpGenContext;

#ifdef __cplusplus
extern "C" {
#endif
  void QpGenFinish ( QpGenContext * ctx,
		     double   x[],  double gamma[],     double phi[],
		     double   y[], 
		     double   z[],  double lambda[],    double pi[],
		     double *objectiveValue,
		     int * status_code );

  void QpGenCleanup( QpGenContext * ctx );

  void QpGenAddMonitor( QpGenContext * ctx, DoItCFunc cmon,
			void * mctx );

  void QpGenMonitorSelf( QpGenContext * ctx );
#ifdef __cplusplus
}
#endif

#endif

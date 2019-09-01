/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef CQPBOUND
#define CQPBOUND

#include "OoqpMonitorData.h"

/** A data structure for holding the various bits needed to solve
 *  a simple bound QP. This structure is part of the CInterface
 *  to using QpBound */
typedef struct {
  void * factory;
  void * prob;
  void * solver;
} QpBoundContext;

#ifdef __cplusplus
extern "C" {
#endif
  void QpBoundDoSolve( double   x[],  double gamma[],     double phi[],
		       QpBoundContext * ctx, int * ierr );

  void QpBoundCleanup( QpBoundContext * ctx );

  void QpBoundAddMonitor( QpBoundContext * ctx, DoItCFunc cmon,
			  void * mctx );
#ifdef __cplusplus
};
#endif

#endif

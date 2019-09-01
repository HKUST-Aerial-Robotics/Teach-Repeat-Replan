/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cQpBoundDense.h"
#include "QpBoundDense.h"
#include "QpBoundData.h"
#include "GondzioSolver.h"

extern "C"
void QpBoundDenseGondzioSetup( double    c[],   int  nx,       double Q[],
			       double xlow[],   char ixlow[], 
			       double xupp[],   char ixupp[],
			       QpBoundContext * ctx,
			       int * ierr )
{
  if( nx <= 0 ) {
    *ierr = 1;
    return;
  }
 
  ctx->factory = 0;
  ctx->prob    = 0;
  ctx->solver  = 0;
  try {
    QpBoundDense * factory = new QpBoundDense( nx );
    ctx->factory               = (void *) factory;
    
    QpBoundData * prob = factory->makeData( 0.0, c,  Q, xlow, ixlow, xupp, ixupp );
    ctx->prob = (void *) prob;
    
    GondzioSolver * solver = new GondzioSolver( factory, prob );
    ctx->solver     = (void *) solver;
    
    *ierr = 0;
  }
  catch( ... ) {
    QpBoundCleanup( ctx );
    *ierr = 1;
  }
}

extern "C"
void qpboundsolvede( double    c[],  int  nx,       double Q[],
		     double xlow[],  char ixlow[], 
		     double xupp[],  char ixupp[],
		     double    x[],  double gamma[],     double phi[],
		     int * ierr )
{
  QpBoundContext  ctx;
  QpBoundDenseGondzioSetup( c, nx, Q, xlow, ixlow, xupp, ixupp,
			    &ctx, ierr );
  if( *ierr == 0 ) {
    QpBoundDoSolve( x, gamma, phi, &ctx, ierr );
  }
  QpBoundCleanup( &ctx );
}


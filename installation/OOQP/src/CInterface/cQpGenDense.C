/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cQpGenDense.h"
#include <cstdlib>
#include "QpGenDense.h"
#include "GondzioSolver.h"
#include "QpGenData.h"
#include "cBounds.h"

extern int gOoqpPrintLevel;

extern "C"
void newQpGenDense( double ** c,      int nx,    double ** Q,
		    double ** xlow,              char   ** ixlow,
		    double ** xupp,              char   ** ixupp,
		    double ** A,      int my,    double ** b,
		    double ** C,      int mz,
		    double ** clow,              char   ** iclow,
		    double ** cupp,              char   ** icupp,
		    int    *  ierr )
{
  *c    = NULL; *Q = NULL;
  *xlow = NULL; *ixlow = NULL; *xupp = NULL; *ixupp = NULL;
  *A    = NULL; *b     = NULL;
  *C    = NULL; 
  *clow = NULL; *iclow = NULL; *cupp = NULL; *icupp = NULL;

  *c    = (double *) malloc( nx * sizeof( double ) );
  if( *c != NULL ) {
    *ierr = 0;
  } else {
    *ierr = 1;
  }
  if( 0 == *ierr ) newBounds( xlow, nx, ixlow, xupp, ixupp, ierr );
  if( 0 == *ierr ) newBounds( clow, mz, iclow, cupp, icupp, ierr );
  if( 0 == *ierr ) {
    *Q = (double *) malloc( nx * nx * sizeof( double ) );
    if( *Q == NULL ) *ierr = 1;
  }
  if( 0 == *ierr && my > 0 ) {
    *A = (double *) malloc( nx * my * sizeof(double) );
    *b = (double *) malloc( my * sizeof( double ) );
    if( *A == NULL || *b == NULL ) *ierr = 1;
  }
  if( 0 == *ierr && mz > 0 ) {
    *C = (double *) malloc( nx * mz * sizeof( double ) );
    if( *C == NULL ) *ierr = 1;
  }

  if( *ierr != 0 ) {
    freeQpGenDense( c,  Q,    xlow,   ixlow,  xupp,  ixupp,
		    A,  b,
		    C,  clow, iclow,  cupp,   icupp );
  }
}

extern "C"
void freeQpGenDense( double ** c,      double ** Q,
		     double ** xlow,   char   ** ixlow,
		     double ** xupp,   char   ** ixupp,
		     double ** A,      double ** b,
		     double ** C,
		     double ** clow,   char ** iclow,
		     double ** cupp,   char ** icupp )
{
  if( NULL != *c ) free( *c ); *c = NULL;
  if( NULL != *Q ) free( *Q ); *Q = NULL;
  freeBounds( xlow, ixlow, xupp, ixupp );
  if( NULL != *A ) free( *A ); *A = NULL;
  if( NULL != *b ) free( *b ); *b = NULL;
  if( NULL != *C ) free( *C ); *C = NULL;
  freeBounds( clow, iclow, cupp, icupp );
}

extern "C" 
void QpGenDenseGondzioSetup( double    c[],   int  nx,       double Q[],
			     double xlow[],   char ixlow[], 
			     double xupp[],   char ixupp[],
			     double    A[],   int  my,       double   bA[],
			     double    C[],   int  mz,   
			     double clow[],   char iclow[],
			     double cupp[],   char icupp[],
			     QpGenContext * ctx,
			     int * ierr )
{
  if( nx <= 0 || my < 0 || mz < 0 ||
      ctx == NULL ) {
    *ierr = 1;
    return;
  }

  ctx->factory = 0;
  ctx->prob    = 0;
  ctx->solver  = 0;
  try {
    QpGenDense * factory =
      new QpGenDense( nx, my, mz );
    ctx->factory = (void *) factory;

    Data * prob = factory->makeData( c,  Q, xlow, ixlow, xupp, ixupp,
				     A,  bA,
				     C,  clow,  iclow,  cupp, icupp);
    ctx->prob = (void *) prob;

    GondzioSolver * solver = new GondzioSolver( factory, prob );
    ctx->solver = (void *) solver;

    *ierr = 0;
  }
  catch( ... ) {
    QpGenCleanup( ctx );
    *ierr = 1;
  }
}


//////////////////
extern "C" 
void qpsolvede( double    c[],  int  nx,       double Q[],
		double xlow[],  char ixlow[], 
		double xupp[],  char ixupp[],
		double    A[],  int  my,       double   bA[],
		double    C[],  int  mz,
		double clow[],  char iclow[],
		double cupp[],  char icupp[],
		double    x[],  double gamma[],     double phi[],
		double    y[],
		double    z[],  double lambda[],  double pi[],
		double   *objectiveValue,
		int print_level,
		int * ierr )
{
  QpGenContext   ctx;
  QpGenDenseGondzioSetup( c,  nx,  Q,     xlow,   ixlow,  xupp,  ixupp,
			  A,  my,  bA,
			  C,  mz,  clow,  iclow,  cupp,   icupp,
			  &ctx, ierr );
  if( *ierr == 0 ) {
    Solver * solver = (Solver *) ctx.solver;
    gOoqpPrintLevel = print_level;
    solver->monitorSelf();

    QpGenFinish( &ctx, x, gamma, phi, y, z, lambda, pi, objectiveValue, ierr );
  }
  QpGenCleanup( &ctx );
}





/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cQpGenSparse.h"
#include "cBounds.h"
#include <cstdlib>

#include "QpGenSparseMa27.h"
#include "GondzioSolver.h"

extern int gOoqpPrintLevel;

extern "C"
void newQpGenSparse( double ** c,      int nx,
		     int    ** irowQ,  int nnzQ,  int  ** jcolQ,  double ** dQ,
		     double ** xlow,              char ** ixlow,
		     double ** xupp,              char ** ixupp,
		     int    ** irowA,  int nnzA,  int  ** jcolA,  double ** dA,
		     double ** b,      int my,
		     int    ** irowC,  int nnzC,  int  ** jcolC,  double ** dC,
		     double ** clow,   int mz,    char ** iclow,
		     double ** cupp,              char ** icupp,
		     int    *  ierr )
{
  *c      = NULL;  *b      = NULL;
  *irowQ  = NULL;  *jcolQ  = NULL;  *dQ   = NULL;
  *irowA  = NULL;  *jcolA  = NULL;  *dA   = NULL;
  *irowC  = NULL;  *jcolC  = NULL;  *dC   = NULL;

  *xlow   = NULL;  *ixlow  = NULL;  *xupp = NULL;  *ixupp = NULL;
  *clow   = NULL;  *iclow  = NULL;  *cupp = NULL;  *icupp = NULL;

  *ierr = 0;

  *c     = (double *) malloc( nx * sizeof( double ) );
  if( NULL == *c ) *ierr = -1;
  if( *ierr == 0 ) newBounds( xlow, nx, ixlow, xupp, ixupp, ierr );

  if( *ierr == 0 && nnzQ > 0 ) newTriple( irowQ, nnzQ, jcolQ, dQ, ierr );
  if( *ierr == 0 && nnzA > 0 ) newTriple( irowA, nnzA, jcolA, dA, ierr );
  if( *ierr == 0 && nnzC > 0 ) newTriple( irowC, nnzC, jcolC, dC, ierr );

  if( *ierr == 0 && my > 0 ) {
    *b = (double *) malloc( my * sizeof( double ) );
    if( NULL == *b ) *ierr = -1;
  }
  if( *ierr == 0 && mz > 0 ) {
    newBounds( clow, mz, iclow, cupp, icupp, ierr );
  }
  
  // Clean up if we have failed
  if( *ierr != 0 ) {
    freeQpGenSparse( c,     irowQ,  jcolQ,  dQ,
		     xlow,  ixlow,  xupp,   ixupp,
		     irowA, jcolA,  dA,     b,
		     irowC, jcolC,  dC,
		     clow,  iclow,  cupp,   icupp );
  }
}

void freeQpGenSparse( double ** c,      
		      int    ** irowQ,  int  ** jcolQ,  double ** dQ,
		      double ** xlow,   char ** ixlow,
		      double ** xupp,   char ** ixupp,
		      int    ** irowA,  int  ** jcolA,  double ** dA,
		      double ** b,
		      int    ** irowC,  int  ** jcolC,  double ** dC,
		      double ** clow,   char ** iclow,
		      double ** cupp,   char ** icupp )
{
  if( NULL != *c ) free( *c );          *c    = NULL;
  freeBounds( xlow, ixlow, xupp, ixupp );

  freeTriple( irowQ, jcolQ, dQ );
  freeTriple( irowA, jcolA, dA );
  freeTriple( irowC, jcolC, dC );
  
  if( NULL != *b ) free( *b );           *b   = NULL;
  freeBounds( clow, iclow, cupp, icupp );
}

void newTriple( int ** irow, int nnz, int ** jcol, double ** M, int * ierr )
{
  *irow = NULL; *jcol = NULL; *M = NULL;
  *ierr = 0;

  if( nnz > 0 ) {
    *irow = (int *)    malloc( nnz * sizeof( int    ) );
    *jcol = (int *)    malloc( nnz * sizeof( int    ) );
    *M =    (double *) malloc( nnz * sizeof( double ) );

    if( NULL == *irow || NULL == *jcol || NULL == *M ) {
      freeTriple( irow, jcol, M );
      *ierr = -1;
    }
  }
}


void freeTriple( int ** irow, int ** jcol, double ** M )
{
  if( NULL != *irow ) free( *irow ); 
  if( NULL != *jcol ) free( *jcol );
  if( NULL != *M )    free( *M );
  *irow = NULL; *jcol = NULL; *M = NULL;
}


void makehb( int irow[], int nnz, int krow[], int m, int * ierr )
{
  int i = 0, k = 0;
  krow[0] = 0;

  while( i < m && k < nnz ) {
    if( irow[k] == i ) {
      /* Element k is in row i */
      k++;
    } else if ( irow[k] > i ) {
      /* Element k belongs to a row greater than i */
      i++;
      krow[i] = k;
    } else { /* irow[k] < i which means the elts weren't sorted */
      *ierr = -1;
      return;
    }
  }
  /* Should have run out of elements in irow */
  if( k != nnz ) {
    *ierr = -1;
    return;
  }
  /* Fill in the remaining rows of krow */
  for( i++; i < m + 1; i++ ) {
    krow[i] = nnz;
  }

  *ierr = 0;
}


extern "C" 
void QpGenHbGondzioSetup( double    c[],  int  nx,
			  int   krowQ[],  int  jcolQ[],  double dQ[],
			  double xlow[],  char ixlow[], 
			  double xupp[],  char ixupp[],
			  int   krowA[],  int  my,       int  jcolA[],
			  double   dA[],
			  double   bA[],
			  int   krowC[],  int  mz,       int  jcolC[],
			  double dC[],
			  double clow[],  char iclow[],
			  double cupp[],  char icupp[],
			  QpGenContext * ctx,
			  int * ierr )
{
  int nnzQ = 0, nnzA = 0, nnzC = 0;
  nnzQ = krowQ[nx];
  if( my > 0 ) nnzA = krowA[my];
  if( mz > 0 ) nnzC = krowC[mz];
    
  if( nx <= 0 || my < 0 || mz < 0 || nnzQ < 0 || nnzA < 0 || nnzC < 0 ||
      ctx == NULL ) {
    *ierr = -1;
    return;
  }

  ctx->factory = 0;
  ctx->prob    = 0;
  ctx->solver  = 0;
  try {
    QpGenSparseSeq * factory =
      new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );
    ctx->factory = (void *) factory;

    Data * prob = factory->makeData( c,     krowQ,  jcolQ,  dQ,
				     xlow,  ixlow,  xupp,   ixupp,
				     krowA, jcolA,  dA,     bA,
				     krowC, jcolC,  dC,
				     clow,  iclow,  cupp, icupp);
    ctx->prob = (void *) prob;

    GondzioSolver * solver = new GondzioSolver( factory, prob );
    //solver->monitorSelf();
    ctx->solver = (void *) solver;

    *ierr = 0;
  }
  catch( ... ) {
    QpGenCleanup( ctx );
    *ierr = -1;
  }
}


extern "C" 
void qpsolvehb( double    c[],  int  nx,
		int   krowQ[],  int  jcolQ[],  double dQ[],
	        double xlow[],  char ixlow[], 
		double xupp[],  char ixupp[],
		int   krowA[],  int  my,       int  jcolA[],  double dA[],
		double   bA[],
		int   krowC[],  int  mz,       int  jcolC[],  double dC[],
		double clow[],  char iclow[],
		double cupp[],  char icupp[],
		double    x[],  double gamma[],     double phi[],
		double    y[],
		double    z[],  double lambda[],  double pi[],
		double * objectiveValue,
		int print_level, int * status_code )
{
  QpGenContext ctx;

  QpGenHbGondzioSetup( c,      nx,     krowQ,  jcolQ,  dQ,
		       xlow,   ixlow,  xupp,   ixupp,
		       krowA,  my,     jcolA,  dA,     bA,
		       krowC,  mz,     jcolC,  dC,
		       clow,   iclow,  cupp,   icupp,  &ctx,
		       status_code );
  if( *status_code == 0 ) {
    Solver * solver = (Solver *) ctx.solver;
    gOoqpPrintLevel = print_level;
    solver->monitorSelf();

    QpGenFinish( &ctx, x, gamma, phi, y, z, lambda, pi, objectiveValue,
		 status_code );
  }
  
  QpGenCleanup( &ctx );
} 

//////////////////
extern "C" 
void qpsolvesp( double    c[],  int  nx,
	      int   irowQ[],  int  nnzQ,     int  jcolQ[],  double dQ[],
	      double xlow[],  char ixlow[], 
	      double xupp[],  char ixupp[],
	      int   irowA[],  int  nnzA,     int  jcolA[],  double dA[],
	      double   bA[],  int  my,
	      int   irowC[],  int  nnzC,     int  jcolC[],  double dC[],
	      double clow[],  int  mz,       char iclow[],
	      double cupp[],  char icupp[],
	      double    x[],  double gamma[],     double phi[],
	      double    y[],
	      double    z[],  double lambda[],  double pi[],
	      double *objectiveValue,
	      int print_level, int * status_code )
{
  *status_code = 0;
  int *krowQ = 0, *krowA = 0, *krowC = 0;
  try {
    krowQ = new int[nx + 1];
    krowA = new int[my + 1];
    krowC = new int[mz + 1];
  }
  catch( ... ) {
    *status_code = -1;
    
  }
  if( *status_code == 0 ) {
    makehb( irowQ, nnzQ, krowQ, nx, status_code );
    if( *status_code == 0 ) makehb( irowA, nnzA, krowA, my, status_code );
    if( *status_code == 0 ) makehb( irowC, nnzC, krowC, mz, status_code );

    if( *status_code == 0 ) {
      qpsolvehb( c,      nx,     krowQ,   jcolQ,  dQ,
		 xlow,   ixlow,  xupp,    ixupp,
		 krowA,  my,     jcolA,   dA,     bA,
		 krowC,  mz,     jcolC,   dC,
		 clow,   iclow,  cupp,    icupp,
		 x,      gamma,  phi,
		 y,      z,      lambda,  pi,
		 objectiveValue,
		 print_level, status_code );
    }
  }

  delete [] krowQ;
  delete [] krowA;
  delete [] krowC;
  
}

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <stdio.h>
#include <stdlib.h>

#include "cMpsReader.h"
#include "cQpGenSparse.h"

#define PRINT_LEVEL 0

int main()
{
  char filename[] = "../sample-qps/Example.qps";
  int nx, my, mz;
  int nnzC, nnzA, nnzQ;

  double *c, *dQ, *xlow, *xupp;
  int    *irowQ, *jcolQ;
  char   *ixlow, *ixupp;
  double *dA, *b, *dC, *clow, *cupp;
  int    *irowA, *jcolA, *irowC, *jcolC;
  char   *iclow, *icupp;

  double objconst, objval;
  double *x = 0, *gamma = 0, *phi = 0;
  double *y = 0, *z = 0,     *lambda = 0 , *pi = 0;

  int ierr;
  void * mps;

  mps = newCMpsReader( filename, &ierr );
  if( 0 != ierr ) {
    fprintf( stderr, "Cannot read file %s\n", filename );
    return 1;
  }

  cMpsReaderGetSizes( mps, &nx, &my, &mz );
  
  cMpsReaderGetNNZ( mps, &nnzQ, &nnzA, &nnzC );

  newQpGenSparse( &c, nx, &irowQ, nnzQ, &jcolQ, &dQ,
		  &xlow, &ixlow, &xupp, &ixupp,
		  &irowA, nnzA, &jcolA, &dA,
		  &b, my,
		  &irowC, nnzC, &jcolC, &dC,
		  &clow, mz, &iclow, &cupp, &icupp,
		  &ierr );
  if( 0 != ierr ) {
    fprintf( stderr, "Cannot allocate enough memory to start solving"
	     " the problem.\n" );
    return 1;
  }


  cMpsReaderReadQpGen( mps,  &objconst, c,  irowQ,  jcolQ,  dQ,
		       xlow,  ixlow,  xupp,  ixupp,
		       irowA,  jcolA,  dA,  b,
		       irowC,  jcolC,  dC,
		       clow,  iclow,  cupp,  icupp,  &ierr );
  if( 0 != ierr ) {
    fprintf( stderr, "Cannot read file %s\n", filename );
    return 1;
  }
  
  x     = (double *) calloc( nx, sizeof( double ) );
  gamma = (double *) calloc( nx, sizeof( double ) );
  phi   = (double *) calloc( nx, sizeof( double ) );
  if( my > 0 ) y = (double *) calloc( my, sizeof( double ) );
  if( mz > 0 ) {
    z      = (double *) calloc( mz, sizeof( double ) );
    lambda = (double *) calloc( mz, sizeof( double ) );
    pi     = (double *) calloc( mz, sizeof( double ) );
  }
  qpsolvesp( c, nx, irowQ, nnzQ, jcolQ, dQ, xlow, ixlow, xupp, ixupp,
	     irowA, nnzA, jcolA, dA, b, my,
	     irowC, nnzC, jcolC, dC,
	     clow,  mz,   iclow, cupp, icupp,
	     x, gamma, phi,
	     y, 
	     z, lambda, pi, &objval, PRINT_LEVEL, &ierr );

  if( ierr != 0 ) {
    fprintf( stderr, "Couldn't solve it.\n" );
    return 1;
  } else {
    int i;

    printf(" Final Objective Value: %g\n\n", objval + objconst);
    printf( "Solution:...\n" );
    for( i = 0; i < nx; i++ ) {
      printf( "x[%2d] = %g\n", i, x[i] );
    }
    return 0;
  }
  free( x ); free(gamma); free(phi);
  if( NULL != y ) free(y);
  if( NULL != lambda ) free(lambda);
  if( NULL != pi ) free(pi);

 
  freeQpGenSparse( &c, &irowQ, &jcolQ, &dQ,
		   &xlow,  &ixlow, &xupp,  &ixupp,
		   &irowA, &jcolA, &dA, &b,
		   &irowC, &jcolC, &dC, &clow,  &iclow, &cupp,  &icupp );
  
  freeCMpsReader( &mps );

  return 0;
}

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/* Mike Gertz 2-Aug-98 */
#ifndef MEXUTILITY
#define MEXUTILITY

#ifndef mex_h
#include "mex.h"
#endif


void dble2int ( int n, double * da, int * ia );
void int2dble( int n, int * ia, double * da );

void assertScalar( const mxArray * mex, char name[8] );
void assertRowDim( const mxArray * mex, int dim,  char name[8] );
void assertColDim( const mxArray * mex, int dim,  char name[8] );
void assertString( const mxArray * mex, int maxl, char name[8] );
void assertSparse( const mxArray * mex, char name[8] );
void mx2char ( int rows, int cols, int lendp, mxChar * dp, char string[] );
void jc2cols ( int n, int * jc, int * cols );
void int2integer( int n, int * i, int * ii );
int iequal( int * a, int * b, int n );
void assertDense( const mxArray * mex, char name[8] );
int overlaySparse( int m, int n, int nza, int ira[], int jca[], double a[],
					int nzb, int irb[], int jcb[], double b[] );

#endif

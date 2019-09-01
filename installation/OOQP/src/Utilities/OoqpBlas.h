/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef OOQPBLAS_H
#define OOQPBLAS_H

/**
 * Blas routines used in OOQP 
 */

extern "C"
void dsymv_ ( char * uplo, int * n, double * alpha,
	      double * A, int * lda, 
	      double * x, int * incx,
	      double * beta, double * y, int * incy );

extern "C"
void dgemv_ ( char * trans, int * m, int * n, 
	      double * alpha, double * a, int * lda,
	      double * x, int * incx,
              double * beta, double * y, int * incy );

extern "C" 
void daxpy_ ( int * n, double * alpha, double x[], int * incx,
	      double y[], int * incy );

extern "C"
double ddot_ ( int * n, double dx[], int * incx, double dy[], int * incy );

extern "C"
double dscal_( int * n, double * da, double dx[], int * incx );

#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "DeSymPSDSolver.h"
#include <cassert>

#include "DenseStorage.h"
#include "DenseSymMatrix.h"
#include "SimpleVector.h"

// declarations for LAPACK functions used to factor/solve:

// dsytrf_() factors a symmetric indefinite matrix A, see LAPACK 
// documentation for more details.
extern "C" void dpotrf_( char * uplo, int * n,
			 double * A, int * lda, int * info );
 
extern "C" void dpotrs_( char * uplo, int * n, int * nrhs,
			 double * A, int * lda,
			 double * b, int * ldb, int * info ) ;

// dsytrs_() solves the system Ax = b using the factor obtained by dsytrf_().
extern "C" void dsytrs_(char *uplo, 
			int *n, 
			int *nrhs, 
			double A[], 
			int *lda, 
			int ipiv[], 
			double b[], 
			int *ldb,
			int *info);
  
DeSymPSDSolver::DeSymPSDSolver( DenseSymMatrix * dsm )
{
  mStorage = DenseStorageHandle( dsm->getStorage() );
}

void DeSymPSDSolver::matrixChanged()
{
  char fortranUplo = 'U';
  int info;

  int n = mStorage->n;

  dpotrf_( &fortranUplo, &n, &mStorage->M[0][0], &n, &info );
}

void DeSymPSDSolver::solve ( OoqpVector& x_in )
{
  char fortranUplo = 'U';
  int info;
  int one = 1;

  int n = mStorage->n;
  SimpleVector & x = dynamic_cast<SimpleVector &>(x_in);
  
  dpotrs_( &fortranUplo, &n, &one,	&mStorage->M[0][0],	&n,
	   &x[0], &n, &info);
}

void DeSymPSDSolver::diagonalChanged( int /* idiag */, int /*extent*/ )
{
  this->matrixChanged();
}

DeSymPSDSolver::~DeSymPSDSolver()
{
}

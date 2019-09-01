/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "DeSymIndefSolver.h"
#include "SimpleVector.h"
#include <cassert>

#include "DenseSymMatrix.h"

// declarations for LAPACK functions used to factor/solve:

// dsytrf_() factors a symmetric indefinite matrix A, see LAPACK 
// documentation for more details.
extern "C" void dsytrf_(char *uplo, 
			int *n, 
			double A[], 
			int *lda, 
			int ipiv[], 
			double work[],
			int *lwork, 
			int *info);

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
  
DeSymIndefSolver::DeSymIndefSolver( DenseSymMatrix * dm )
{
  mStorage = DenseStorageHandle( dm->getStorage() );

  int size = mStorage->n;
  ipiv = new int[size];
  int lwork = 10*size*size;
  work = SimpleVectorHandle(new SimpleVector(lwork));
}

void DeSymIndefSolver::matrixChanged()
{
  char fortranUplo = 'U';
  int info;

  int n = mStorage->n;
  int lwork = work->length();

  dsytrf_( &fortranUplo, &n, &mStorage->M[0][0], &n,
	   ipiv, work->elements(), &lwork, &info );
}

void DeSymIndefSolver::solve ( OoqpVector& v )
{
  char fortranUplo = 'U';
  int info;
  int one = 1;

  int n = mStorage->n;
  
  SimpleVector &  sv = dynamic_cast<SimpleVector &>(v);
  dsytrs_( &fortranUplo, &n, &one,	&mStorage->M[0][0],	&n,
	   ipiv, &sv[0],	&n,	&info);
}

void DeSymIndefSolver::diagonalChanged( int /* idiag */, int /* extent */ )
{
  this->matrixChanged();
}

DeSymIndefSolver::~DeSymIndefSolver()
{
  delete [] ipiv;
}

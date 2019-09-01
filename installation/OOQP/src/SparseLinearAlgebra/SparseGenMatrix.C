/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "SparseGenMatrix.h"
#include "SparseStorage.h"
#include <cassert>
#include "SimpleVector.h"

#include "DoubleMatrixTypes.h"

int SparseGenMatrix::isKindOf( int type )
{
  return type == kSparseGenMatrix || type == kGenMatrix;
}


SparseGenMatrix::SparseGenMatrix( int rows, int cols, int nnz )
{
  mStorage = SparseStorageHandle( new SparseStorage( rows, cols, nnz ) );
}


SparseGenMatrix::SparseGenMatrix( int rows, int cols, int nnz,
					  int krowM[], int jcolM[],
					  double M[] )
{
  mStorage = SparseStorageHandle( new SparseStorage( rows, cols,
							 nnz, krowM,
							 jcolM, M ) );
}


void SparseGenMatrix::atPutDense( int row, int col, double * A, int lda,
				      int rowExtent, int colExtent )
{
  mStorage->atPutDense( row, col, A, lda, rowExtent, colExtent );
}


void SparseGenMatrix::fromGetDense( int row, int col, double * A, int lda,
					int rowExtent, int colExtent )
{
  mStorage->fromGetDense( row, col, A, lda, rowExtent, colExtent );
}
  

void SparseGenMatrix::fromGetSpRow( int row, int col,
				    double A[], int lenA,
				    int jcolA[], int& nnz,
				    int colExtent, int& info )
{
  mStorage->fromGetSpRow( row, col, A, lenA, jcolA, nnz,
			  colExtent, info );
}


void SparseGenMatrix::putSparseTriple( int irow[], int len,
					   int jcol[], double A[], 
					   int& info )
{
  mStorage->putSparseTriple( irow, len, jcol, A, info );
}


void SparseGenMatrix::writeToStream(ostream& out) const
{
  mStorage->writeToStream( out );
}


void SparseGenMatrix::randomize( double alpha, double beta, double * seed )
{
  mStorage->randomize( alpha, beta, seed );
}


void SparseGenMatrix::getDiagonal( OoqpVector& vec )
{
  mStorage->getDiagonal( vec );
}


void SparseGenMatrix::setToDiagonal( OoqpVector& vec )
{
  mStorage->setToDiagonal( vec );
}


void SparseGenMatrix::atPutSpRow( int row, double A[],
				      int lenA, int jcolA[], int& info )
{
  mStorage->atPutSpRow( row, A, lenA, jcolA, info );
}


int SparseGenMatrix::numberOfNonZeros()
{
  return mStorage->numberOfNonZeros();
}


void SparseGenMatrix::symmetrize( int& info ) 
{
  mStorage->symmetrize( info );
}


void SparseGenMatrix::getSize( int& m, int& n )
{
  m = mStorage->m;
  n = mStorage->n;
}


void SparseGenMatrix::atPutSubmatrix( int destRow, int destCol,
					  DoubleMatrix& M,
					  int srcRow, int srcCol,
					  int rowExtent, int colExtent )
{
  int i, k;
  int info, nnz;

  int *    ja = new int[colExtent];
  double * a  = new double[colExtent];

  nnz = 0;
  for ( i = 0; i < rowExtent; i++ ) {
    M.fromGetSpRow( srcRow + i, srcCol, a, colExtent, ja,
		     nnz, colExtent, info );
    for( k = 0; k < nnz; k++ ) {
      ja[k] += (destCol - srcCol);
    }
    mStorage->atPutSpRow( destRow + i, a, nnz, ja, info );
  }

  delete [] ja;
  delete [] a;
}


void SparseGenMatrix::mult ( double beta,  OoqpVector& y_in,
				 double alpha, OoqpVector& x_in )
{
  SimpleVector & x = dynamic_cast<SimpleVector &>(x_in);
  SimpleVector & y = dynamic_cast<SimpleVector &>(y_in);
  
  assert( x.n == mStorage->n && y.n == mStorage->m );

  double *xv = 0, *yv = 0;

  if( x.n > 0 ) xv = &x[0];
  if( y.n > 0 ) yv = &y[0];

  mStorage->mult( beta, yv, 1, alpha, xv, 1 );
}


void SparseGenMatrix::transMult ( double beta,   OoqpVector& y_in,
				      double alpha,  OoqpVector& x_in )
{
  SimpleVector & x = dynamic_cast<SimpleVector &>(x_in);
  SimpleVector & y = dynamic_cast<SimpleVector &>(y_in);
  
  assert( x.n == mStorage->m && y.n == mStorage->n );

  double *xv = 0, *yv = 0;

  if( x.n > 0 ) xv = &x[0];
  if( y.n > 0 ) yv = &y[0];

  mStorage->transMult( beta, yv, 1, alpha, xv, 1 );
}




double SparseGenMatrix::abmaxnorm()
{
  return mStorage->abmaxnorm();
}


void SparseGenMatrix::atPutDiagonal( int idiag, OoqpVector& vvec )
{
  SimpleVector & v = dynamic_cast<SimpleVector &>(vvec);

  mStorage->atPutDiagonal( idiag, v.elements(), 1, v.n );
}


void SparseGenMatrix::fromGetDiagonal( int idiag, OoqpVector& vvec )
{
  mStorage->fromGetDiagonal( idiag, vvec );
}

void SparseGenMatrix::ColumnScale( OoqpVector& vec )
{
  mStorage->ColumnScale( vec );
}

void SparseGenMatrix::scalarMult( double num )
{
  mStorage->scalarMult( num );
}

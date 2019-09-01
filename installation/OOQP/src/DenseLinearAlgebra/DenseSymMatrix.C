/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <cmath>
#include <cassert>

#include "DenseSymMatrix.h"
#include "DeSymPSDSolver.h"
#include "DeSymIndefSolver.h"
#include "OoqpBlas.h"
#include "SimpleVector.h"

#include "DoubleMatrixTypes.h"

int DenseSymMatrix::isKindOf( int matrixType )
{
  return matrixType == kDenseSymMatrix || matrixType == kSymMatrix;
}


DenseSymMatrix::DenseSymMatrix( int size )
{ 
  mStorage = DenseStorageHandle( new DenseStorage( size, size ) );
}


DenseSymMatrix::DenseSymMatrix( double Q[], int size )
{
  mStorage = DenseStorageHandle( new DenseStorage( Q, size, size ) );
}


void DenseSymMatrix::symAtPutZeros( int row, int col,
				     int rowExtent, int colExtent )
{
  mStorage->atPutZeros( row, col, rowExtent, colExtent );
}


void DenseSymMatrix::putSparseTriple( int irow[], int len,
					  int jcol[], double A[], 
					  int& info )
{
  mStorage->putSparseTriple( irow, len, jcol, A, info );
}


void DenseSymMatrix::atAddOuterProductOf( int row, int col, double alpha,
					      double * x, int incx, int nx )
{
  mStorage->atAddOuterProductOf( row, col, alpha, x, incx, nx );
}


void DenseSymMatrix::getDiagonal( OoqpVector& vec )
{
  mStorage->getDiagonal( vec );
}


void DenseSymMatrix::setToDiagonal( OoqpVector& vec )
{
  mStorage->setToDiagonal( vec );
}


void DenseSymMatrix::fromGetSpRow( int row, int col,
				       double A[], int lenA,
				       int jcolA[], int& nnz,
				       int colExtent, int& info )
{
  if( col + colExtent < row + 1 ) {
    mStorage->fromGetSpRow( row, col, A, lenA, jcolA, nnz, colExtent, info );
  } else {
    if( col <= row ) {
      mStorage->fromGetSpRow( row, col, A, lenA, jcolA,
			      nnz, row - col + 1, info );
    }
  }
}


void DenseSymMatrix::getSize( int& m, int& n )
{
  m = mStorage->m;
  n = mStorage->n;
}


int DenseSymMatrix::size()
{ 
   return mStorage->m;
}


void DenseSymMatrix::symAtPutSubmatrix( int destRow, int destCol,
					    DoubleMatrix& Mat,
					    int srcRow, int srcCol,
					    int rowExtent, int colExtent )
{
  int m = mStorage->m, n = mStorage->n;
  double ** M = mStorage->M;

  assert( destRow >= 0 && destRow + rowExtent <= m );
  assert( destCol >= 0 && destCol + colExtent <= n );

  // If assertions are turned off, clip to the actual size of this matrix
  destRow = ( destRow >= 0 ) ? destRow : 0;
  destCol = ( destCol >= 0 ) ? destCol : 0;
  rowExtent = ( destRow + rowExtent <= m ) ?  rowExtent : m - destRow;
  colExtent = ( destCol + colExtent <= n ) ?  colExtent : n - destCol;

  Mat.fromGetDense( srcRow, srcCol, &M[destRow][destCol], n,
		     rowExtent, colExtent );
  //    int i, j;
  //    for( i = 0; i < m; i++ ) {
  //  	for( j = i + 1; j < n; j++ ) {
  //  	  M[i][j] = 0.0;
  //  	}
  //    }
}


void DenseSymMatrix::mult ( double beta,  double y[], int incy,
				double alpha, double x[], int incx )
{
  char fortranUplo = 'U';
  int n = mStorage->n;
  
  dsymv_( &fortranUplo, &n, &alpha, &mStorage->M[0][0], &n,
	  x, &incx, &beta, y, &incy );
}


void DenseSymMatrix::mult ( double beta,  OoqpVector& y_in,
				double alpha, OoqpVector& x_in )
{
  char fortranUplo = 'U';
  int n = mStorage->n;
  SimpleVector & y = (SimpleVector &) y_in;
  SimpleVector & x = (SimpleVector &) x_in;
  int incx = 1, incy = 1;
  
  if( n != 0 ) {
    dsymv_( &fortranUplo, &n, &alpha, &mStorage->M[0][0], &n,
	    &x[0], &incx, &beta, &y[0], &incy );
  } 
}


void DenseSymMatrix::transMult ( double beta,  OoqpVector& y,
				     double alpha, OoqpVector& x )
{
  // We're symmetric silly
  this->mult( beta, y, alpha, x );
}


void DenseSymMatrix::transMult ( double beta,  double y[], int incy,
				     double alpha, double x[], int incx )
{
  // We're symmetric silly
  this->mult( beta, y, incy, alpha, x, incx );
}


double DenseSymMatrix::abmaxnorm()
{
  double norm = 0;
  
  int i, j;
  double ** M = mStorage->M;
  int m = mStorage->m;
  double eltNorm;

  for ( i = 0; i < m; i++ ) {
    for ( j = 0; j <= i; j++ ) {
      eltNorm = fabs( M[i][j] );
      if ( eltNorm > norm ) norm = eltNorm;
    }
  }
  return norm;
}


void DenseSymMatrix::writeToStream(ostream& out) const
{
  int i, j;
  int n = mStorage->n;
  double ** M = mStorage->M;

  for( i = 0; i < n; i++ ) {
    for( j = 0; j <= i && j < n - 1; j++ ) {
      out << M[i][j] << "   ";
    }
    for(      ; j < n - 1; j++ ) {
      out << M[j][i] << "   ";
    }
    if ( j < n )     out << M[j][i];
    if ( i < n - 1 ) out << endl;
  }
}


void DenseSymMatrix:: randomizePSD(double * seed)
{  
  int n = mStorage->n;
  double ** M = mStorage->M;
  double drand(double*);
  int i, j, k;

  mStorage->atPutZeros( 0, 0, n, n );
  for(i=0;i<n;i++) {
    for(j=0;j<=i;j++) {
      M[i][j] = drand(seed);
    }
  }

  for( i = n-1; i >= 0; i-- ) {
    for( j = i; j >= 0; j--) {
      M[i][j] = M[i][j] * M[j][j];
      for( k = j - 1; k >= 0; k-- ) {
	M[i][j] += M[i][k]*M[j][k];
      }
    }
  }
}


void DenseSymMatrix::fromGetDense( int row, int col, double * A,
				       int lda,
				       int rowExtent, int colExtent )
{
  int m = mStorage->m, n = mStorage->n;
  double ** M = mStorage->M;

  int i, j;
  assert( row >= 0 && row + rowExtent <= m );
  assert( col >= 0 && col + colExtent <= n );

  // If assertions are turned off, clip to the actual size of this matrix
  row = ( row >= 0 ) ? row : 0;
  col = ( col >= 0 ) ? col : 0;
  rowExtent = ( row + rowExtent <= m ) ?  rowExtent : m - row;
  colExtent = ( col + colExtent <= n ) ?  colExtent : n - col;
  
  for ( i = row; i < row + rowExtent; i++ ) {
    for ( j = col; j <= i && j < col + colExtent; j++ ) {
      A[(i - row)*lda + j - col] = M[i][j];
    }
    for( ; j < col + colExtent; j++ ) {
      A[(i - row)*lda + j - col] = M[j][i];
    }
  }
}


void DenseSymMatrix::atPutDiagonal( int idiag,
					OoqpVector& v )
{
  mStorage->atPutDiagonal( idiag, v );
}


void DenseSymMatrix::fromGetDiagonal( int idiag,
					  OoqpVector& v )
{
  mStorage->fromGetDiagonal( idiag, v );
}


void DenseSymMatrix::symAtPutSpRow( int row, double A[], int lenA,
					int jcolA[], int& info )
{
  // Lower triangular put
  int lA = lenA;
  while( lA > 0 && jcolA[lA - 1] > row ) lA--;
  if( lA > 0 ) {
    mStorage->atPutSpRow( row, A, lA, jcolA, info );
  } else {
    info = 0;
  }
}

void DenseSymMatrix::symAtPutDense( int row, int col, double * A, int lda,
				     int rowExtent, int colExtent )
{
  mStorage->atPutDense( row, col, A, lda, rowExtent, colExtent );
}

void DenseSymMatrix::SymmetricScale( OoqpVector& vec )
{
  mStorage->SymmetricScale( vec );
}

void DenseSymMatrix::scalarMult( double num )
{
  mStorage->scalarMult( num );
}

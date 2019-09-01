/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <cassert>
#include <cmath>

#include "DenseGenMatrix.h"
#include "DenseSymMatrix.h"
#include "OoqpBlas.h"
#include "SimpleVector.h"

#include "DoubleMatrixTypes.h"

int DenseGenMatrix::isKindOf( int type )
{
  return type == kDenseGenMatrix || type == kGenMatrix;
}


DenseGenMatrix::DenseGenMatrix( int size )
{
  mStorage = DenseStorageHandle( new DenseStorage( size, size ) );
}


DenseGenMatrix::DenseGenMatrix( double A[], int m, int n )
{
  mStorage = DenseStorageHandle( new DenseStorage( A, m, n ) );
}


DenseGenMatrix::DenseGenMatrix( int m, int n  )
{
  mStorage = DenseStorageHandle( new DenseStorage( m, n ) );
}


// Delegate these methods to the storage
void DenseGenMatrix::atPutDense( int row, int col, double * A, int lda,
				     int rowExtent, int colExtent )
{
  mStorage->atPutDense( row, col, A, lda, rowExtent, colExtent );
}


void DenseGenMatrix::atPutZeros( int row, int col,
				     int rowExtent, int colExtent )
{
  mStorage->atPutZeros( row, col, rowExtent, colExtent );
}


void DenseGenMatrix::putSparseTriple( int irow[], int len,
					  int jcol[], double A[], 
					  int& info )
{
  mStorage->putSparseTriple( irow, len, jcol, A, info );
}


void DenseGenMatrix::fromGetSpRow( int row, int col,
				       double A[], int lenA,
				       int jcolA[], int& nnz,
				       int colExtent, int& info )
{
  mStorage->fromGetSpRow( row, col, A, lenA, jcolA, nnz, colExtent, info );
}


void DenseGenMatrix::atPutSpRow( int  row, double * A,
				     int lenA , int * jcolA,
				     int& info )
{
  mStorage->atPutSpRow( row, A, lenA, jcolA, info );
}


void DenseGenMatrix::getDiagonal( OoqpVector& vec )
{
  mStorage->getDiagonal( vec );
}


void DenseGenMatrix::setToDiagonal( OoqpVector& vec )
{
  mStorage->setToDiagonal( vec );
}

void DenseGenMatrix::getSize( int& m, int& n )
{
  m = mStorage->m;
  n = mStorage->n;
}


void DenseGenMatrix::atPutSubmatrix( int destRow, int destCol,
					 DoubleMatrix & Mat,
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
}


void DenseGenMatrix::mult ( double beta,  double y[], int incy,
				double alpha, double x[], int incx )
{
  char fortranTrans = 'T';
  int n = mStorage->n, m = mStorage->m;
  
  dgemv_( &fortranTrans, &n, &m, &alpha, &mStorage->M[0][0], &n,
	  x, &incx, &beta, y, &incy );
}


void DenseGenMatrix::mult ( double beta,  OoqpVector& y_in,
				double alpha, OoqpVector& x_in )
{
  char fortranTrans = 'T';
  int n = mStorage->n, m = mStorage->m;
  double ** M = mStorage->M;
  int incx = 1, incy = 1;

  SimpleVector & x = dynamic_cast<SimpleVector &>(x_in);
  SimpleVector & y = dynamic_cast<SimpleVector &>(y_in);

  if( n != 0 && m != 0 ) {
    dgemv_( &fortranTrans, &n, &m, &alpha, &M[0][0], &n,
	    &x[0], &incx, &beta, &y[0], &incy );
  } else {
    if( m != 0 ) y.scale( beta );
  }
}


void DenseGenMatrix::transMult ( double beta,  double y[], int incy,
				     double alpha, double x[], int incx )
{
  char fortranTrans = 'N';
  int n = mStorage->n, m = mStorage->m; 
  double ** M = mStorage->M;
 
  dgemv_( &fortranTrans, &n, &m, &alpha, &M[0][0], &n,
	  x, &incx, &beta, y, &incy );
}


void DenseGenMatrix::transMult ( double beta,  OoqpVector& y_in,
				     double alpha, OoqpVector& x_in )
{
  char fortranTrans = 'N';
  int n = mStorage->n, m = mStorage->m;
  double ** M = mStorage->M;
  SimpleVector & x = dynamic_cast<SimpleVector &>(x_in);
  SimpleVector & y = dynamic_cast<SimpleVector &>(y_in);
  int incx = 1, incy = 1;
  
  if( m != 0 && n != 0 ) {
    dgemv_( &fortranTrans, &n, &m, &alpha, &M[0][0], &n,
	    &x[0], &incx, &beta, &y[0], &incy );
  } else {
    if( n != 0 ) {
	y.scale( beta );
    }
  }
}


double DenseGenMatrix::abmaxnorm()
{
  double norm = 0;
  
  int i, j;
  double ** M = mStorage->M;
  int m = mStorage->m, n = mStorage->n;
  double eltNorm;

  for ( i = 0; i < m; i++ ) {
    for ( j = 0; j < n; j++ ) {
      eltNorm = fabs( M[i][j] );
      if ( eltNorm > norm ) norm = eltNorm;
    }
  }
  return norm;
}


void DenseGenMatrix::writeToStream(ostream& out) const
{
  int i, j;
  int m = mStorage->m, n = mStorage->n;
  double ** M = mStorage->M;

  for( i = 0; i < m; i++ ) {
    for( j = 0; j < n - 1; j++ ) {
      out << M[i][j] << "\t";
    }
    if ( j < n )     out << M[i][j];
    if ( i < m - 1 ) out << endl;
  }
}


void DenseGenMatrix::randomize( double alpha, double beta, double * seed )
{
  int m = mStorage->m, n = mStorage->n;
  double ** M = mStorage->M;
  double drand(double *);
  int i, j;

  double scale = beta - alpha;
  double shift = alpha/scale;

  for( i = 0; i < m; i++ ) {
    for( j = 0; j < n; j++ ) {
      M[i][j] = scale * (drand(seed) + shift);
    }
  }
}


void DenseGenMatrix::fromGetDense( int row, int col, double * A,
				       int lda,
				       int rowExtent, int colExtent )
{
  int m = mStorage->m, n = mStorage->n;

  assert( row >= 0 && row + rowExtent <= m );
  assert( col >= 0 && col + colExtent <= n );

  // If assertions are turned off, clip to the actual size of this matrix
  row = ( row >= 0 ) ? row : 0;
  col = ( col >= 0 ) ? col : 0;
  int lrow = ( row + rowExtent <= m ) ?  rowExtent : m - row;
  int lcol = ( col + colExtent <= n ) ?  colExtent : n - col;
  
  mStorage->fromGetDense( row, col, A, lda, lrow, lcol );
}


void DenseGenMatrix::atPutDiagonal( int idiag, OoqpVector& v )
{
  mStorage->atPutDiagonal( idiag, v );
}



void DenseGenMatrix::fromGetDiagonal( int idiag, OoqpVector& v )
{
  mStorage->fromGetDiagonal( idiag, v );
}


void DenseGenMatrix::getRow ( int rowIndex, OoqpVector& v_in)
{
  assert (rowIndex >= 0 && rowIndex <= mStorage->m);
  SimpleVector & v = dynamic_cast<SimpleVector &>(v_in);

  mStorage->fromGetDense(rowIndex, 0, &v[0], 1, 1, mStorage->n);
}

void DenseGenMatrix::ColumnScale( OoqpVector& vec )
{
  mStorage->ColumnScale( vec );
}

void DenseGenMatrix::scalarMult( double num )
{
  mStorage->scalarMult( num );
}


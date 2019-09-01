/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <cassert>
#include "OoqpBlas.h"
#include "DenseStorage.h"
#include "DoubleMatrix.h"
#include "OoqpVector.h"
#include "SimpleVector.h"

extern "C" void dsyr_( char * uplo, int * n,
		       double * alpha, double * x,
		       int * incx, double * a,
		       int * lda );

int DenseStorageInstances = 0;

void DenseStorage::fromGetDiagonal( int idiag, OoqpVector& vec )
{ 
  int k;
  int extent = vec.length();
  
  assert( idiag + extent <= n );
  assert( idiag + extent <= m );

  SimpleVector &  sv = (SimpleVector &) vec;

  for ( k = idiag; k < idiag + extent; k++ ) {
    sv[k] = M[k][k];
  }
}

void DenseStorage::getDiagonal( OoqpVector& vec )
{ 
  this->fromGetDiagonal( 0, vec );
}


void DenseStorage::setToDiagonal( OoqpVector& vec )
{ 
  int i,k;

  int extent = vec.length();

  assert( extent <= n );
  assert( extent <= m );

  SimpleVector &  sv = (SimpleVector &) vec;
  for( i = 0; i < m; i++ ) {
    for( k = 0; k < n; k++ ) {
      M[i][k] = 0.0;
    }
  }

  for ( k = 0; k < extent; k++ ) {
    M[k][k] = sv[k];
  }
}


void DenseStorage::getSize( int& m_, int& n_ )
{
  m_ = m;
  n_ = n;
}

DenseStorage::DenseStorage( int min, int nin )
{
  DenseStorageInstances++;
  m = min;
  n = nin;

  int mbar = (m > 0 ) ? m : 1; // We always allocate one row.
  try {
    neverDeleteElts = 0;

    M    = new double*[mbar];
    if( m > 0 ) {
      M[0] = new double[m*n];
    } else {
      M[0] = 0;
    }
    int i;
    for( i = 1; i < m; i++ ) M[i] = M[0] + i * n; 
  } catch ( ... ) {
    cerr << "Out of memory in DenseStorage::DenseStorage(int, int)\n";
    throw;
  }
}
 
DenseStorage::DenseStorage( double A[], int min, int nin )
{
  DenseStorageInstances++;
  m = min;
  n = nin;

  M = new double*[m];
  int i;
  for( i = 0; i < m; i++ ) {
    M[i] = A + i * n;
  } 

  neverDeleteElts = 1;
}

void DenseStorage::fromGetSpRow( int row, int col,
				     double A[], int lenA,
				     int jcolA[], int& nnz,
				     int colExtent, int& info )
{
  assert( col >= 0 && col + colExtent <= n );
  assert( row >= 0 && row < m );

  int j, k;
  k = 0; info = 0;

  for( j = col; j < col + colExtent; j++ ) {
    if ( M[row][j] != 0.0 ) { 
      if( k < lenA ) {
	// Add the element to A
	A[k]     = M[row][j];
	jcolA[k] = j;
	k++;
      } else {
	// Count the number of additional elements needed in A
	info++;
      }
    }
  }
  nnz = k;
}

void DenseStorage::atPutSpRow( int row, double A[], int lenA, int jcolA[],
				   int& info )
{
  info = 0;
  int k;
  for ( k = 0; k < lenA; k++ ) {
    M[ row ][ jcolA[k] ] = A[k];
  }
}


void DenseStorage::putSparseTriple( int irow[], int len,
					int jcol[], double A[], 
					int& info )
{
  int i, k;

  for( i = 0; i < m; i++ ) {
    for( k = 0; k < n; k++ ) {
      M[i][k] = 0.0;
    }
  }

  for ( k = 0; k < len; k++ ) {
    assert( irow[k] >= 0 && irow[k] < m );
    assert( jcol[k] >= 0 && jcol[k] < n );
    M[ irow[k] ][ jcol[k] ] = A[k];
  }
  info = 0;
}


void DenseStorage::fromGetDense( int row, int col, double * A,
				     int lda,
				     int rowExtent, int colExtent )
{
  int i;
  assert( row >= 0 && row + rowExtent <= m );
  assert( col >= 0 && col + colExtent <= n );

  for ( i = 0; i < rowExtent; i++ ) {
    memcpy( &A[i*lda], &M[i + row][col], colExtent * sizeof( double ) );
  }
}


DenseStorage::~DenseStorage()
{
  DenseStorageInstances--;
  if( !neverDeleteElts ) {
    delete [] M[0];
  }
  delete [] M;
}

void DenseStorage::atPutZeros( int row, int col,
				   int rowExtent, int colExtent )
{
  int i, j;
  assert( row >= 0 && row + rowExtent <= m );
  assert( col >= 0 && col + colExtent <= n );
  
  // If assertions are turned off, clip to the actual size of this matrix
  row = ( row >= 0 ) ? row : 0;
  col = ( col >= 0 ) ? col : 0;
  int mrow = ( row + rowExtent <= m ) ? row + rowExtent : m;
  int ncol = ( col + colExtent <= n ) ? col + colExtent : n;

  for ( i = row; i < mrow; i++ ) {
    for ( j = col; j < ncol; j++ ) {
      M[i][j] = 0.0;
    }
  }
}

void DenseStorage::atPutDense( int row, int col, double * A, int lda,
				   int rowExtent, int colExtent )
{
  int i;
  assert( row >= 0 && row + rowExtent <= m );
  assert( col >= 0 && col + colExtent <= n );

  for ( i = 0; i < rowExtent; i++ ) {
    memcpy( &M[i + row][col], &A[i*lda], colExtent * sizeof( double ) );
  }
}
  
void DenseStorage::atAddOuterProductOf(int row, int col, double alpha,
					   double * x, int incx, int nx )
{
  assert( row >= 0 && row + nx <= m );
  assert( col >= 0 && col + nx <= n );
  
  // If assertions are turned off, clip to the actual size of this matrix
  row = ( row >= 0 ) ? row : 0;
  col = ( col >= 0 ) ? col : 0;
  
  nx = ( row + nx <= m ) ? nx : m - row;
  nx = ( col + nx <= n ) ? nx : n - col;

  char fortranUplo = 'U';
  
  dsyr_( &fortranUplo, &nx, &alpha, x, &incx,
	 &M[row][col], &n );
}


void DenseStorage::addToDiagonalAt( double alpha, double x[], int incx,
					int idiag, int extent )
{
  assert( idiag + extent <= n );
  assert( idiag + extent <= m );
  
  // If assertions are off, clip to the actual size of this matrix
  if ( idiag + extent < n ) extent = n - idiag;
  if ( idiag + extent < m ) extent = m - idiag;

  int incy =  n + 1;
  daxpy_( &extent, &alpha, x, &incx,
	  &M[idiag][idiag], &incy );
  
}


void DenseStorage::atPutDiagonal( int idiag,
				      OoqpVector& vvec )
{
  SimpleVector & v = (SimpleVector &) vvec;
  
  this->atPutDiagonal( idiag, &v[0], 1, v.n  );
}

void DenseStorage::atPutDiagonal( int idiag,
				      double x[], int incx, int extent )
{
  int i;
  for( i = 0; i < extent; i++ ) {
    M[i + idiag][i + idiag] = x[i*incx];
  }
}

void DenseStorage::ColumnScale( OoqpVector& scale_in )
{
  SimpleVector & scale = dynamic_cast<SimpleVector &>(scale_in);
  int extent = scale.length();

  assert( extent == n );
  
  int i,j;

  for ( i = 0; i < m; i++ ) {
    // Loop over all rows in the dense matrix
    for( j = 0; j < n; j++ ) {
      // Loop over the elements of the dense row
      M[i][j] = M[i][j] * scale[j];
    } // End loop over the elements of the dense row
  } // End loop over all rows in the dense matrix

}

void DenseStorage::scalarMult( double num )
{
  int i,j;

  for ( i = 0; i < m; i++ ) {
    // Loop over all rows in the dense matrix
    for( j = 0; j < n; j++ ) {
      // Loop over the elements of the dense row
      M[i][j] = M[i][j] * num;
    } // End loop over the elements of the dense row
  } // End loop over all rows in the dense matrix
}


void DenseStorage::SymmetricScale( OoqpVector& scale_in )
{
  SimpleVector & scale = dynamic_cast<SimpleVector &>(scale_in);
  int extent = scale.length();

  assert( extent == n );
  assert( extent == m );

  int i, j;

  for ( i = 0; i < m; i++ ) {
    // Loop over all rows in the dense matrix
    for( j = 0; j < n; j++ ) {
      // Loop over the elements of the dense row
      M[i][j] = M[i][j] * scale[i] * scale[j];
    } // End loop over the elements of the dense row
  } // End loop over all rows in the dense matrix

}

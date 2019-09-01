/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "PetscSparseStorage.h"
#include <cassert>
#include "OoqpVector.h"
#include "SimpleVector.h"
#include "PetscVector.h"

PetscSparseStorage::PetscSparseStorage( int lm, int ln,
					int m, int n, int /* nnz */ )
{
  int ierr;

  ierr = MatCreate( PETSC_COMM_WORLD, &M );  assert( ierr  == 0);
  ierr = MatSetFromOptions( M ); assert(ierr == 0);
  ierr = MatSetSizes( M, lm, ln, m, n); assert(ierr == 0);


  preserveMat = 0;
}

PetscSparseStorage::PetscSparseStorage( Mat M_ )
{
  M = M_;

  preserveMat = 1;
}

void PetscSparseStorage::getDiagonal( OoqpVector& vec_in )
{
  PetscVector & vec = dynamic_cast<PetscVector &>(vec_in);

  int ierr;
  ierr = MatGetDiagonal(M, vec.pv); assert( ierr  == 0);
}
 
void PetscSparseStorage::setToDiagonal( OoqpVector& vec_in )
{
  int ierr;
  PetscVector & vec = dynamic_cast<PetscVector &>(vec_in);

  ierr = MatZeroEntries( M ); assert( ierr  == 0);
  ierr = MatDiagonalSet( M, vec.pv, INSERT_VALUES );
}

void PetscSparseStorage::atPutDense( int row, int col, double * A, int lda,
				     int rowExtent, int colExtent )
{
  int k, ierr;
#ifndef NDEBUG
  {
    int m, n;
    this->getSize( m, n );
    assert( row >= 0 && row + rowExtent <= m );
    assert( col >= 0 && col + colExtent <= n );
  }
#endif

  int * jcol = new int[colExtent];
  
  for ( k = 0; k < colExtent; k++ ) {
    jcol[k] = k + col;
  }
  if ( lda == colExtent ) {
    int * irow = new int[rowExtent];

    for ( k = 0; k < rowExtent; k++ ) {
      irow[k] = k + row;
    }
    ierr = 
      MatSetValues( M, rowExtent, irow, colExtent, jcol, A, INSERT_VALUES );
    assert(ierr == 0);
    delete [] irow;
  } else {
    int i;
    for( i = row; i < row + rowExtent; i++ ) {
      ierr = 
	MatSetValues( M, 1, &i, colExtent, jcol,
		      &A[(i - row)*lda], INSERT_VALUES );
      assert(ierr == 0);
    }
  }
  delete [] jcol;

  ierr = MatAssemblyBegin( M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);
  ierr = MatAssemblyEnd( M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);  
}

void PetscSparseStorage::fromGetDense( int row, int col,
				       double * A, int lda,
				       int rowExtent, int colExtent )
{
  int ierr;
  int k;

#ifndef NDEBUG
  {
    int m, n;
    this->getSize( m, n );
    assert( row >= 0 && row + rowExtent <= m );
    assert( col >= 0 && col + colExtent <= n );
  }
#endif

  int * jcol = new int[colExtent];

  for ( k = 0; k < colExtent; k++ ) {
    jcol[k] = k + col;
  }
  
  if( lda == colExtent ) {
    int * irow = new int[rowExtent];
    for ( k = 0; k < rowExtent; k++ ) {
      irow[k] = k + row;
    }
    ierr = MatGetValues( M, rowExtent, irow, colExtent, jcol, A );
    assert(ierr == 0);

    delete [] irow;
  } else {
    int i;
    for( i = row; i < row + rowExtent; i++ ) {
      ierr = MatGetValues( M, 1, &i, colExtent, jcol,
			   &A[(i - row)*lda] );
      assert(ierr == 0);
    }
  }
  delete [] jcol;
}

void PetscSparseStorage::fromGetSpCol( int row, int col,
				       double A[], int lenA,
				       int irowA[], int& nnz,
				       int rowExtent, int& info )
{
  double * B      = new double[rowExtent];
  int    * irowB  = new int[rowExtent];
  int ierr;
  
  int k;
  for ( k =0; k < rowExtent; k++ ) {
    irowB[k] = k + row;
  }
  ierr = MatGetValues( M, rowExtent, irowB, 1, &col, B ); assert(ierr == 0);
  info = 0; nnz = 0;
  for( k = 0; k < rowExtent; k++ ) {
    if ( B[k] != 0.0 ) {
      if ( nnz < lenA ) {
	irowA[nnz] = irowB[k];
	A[nnz]     = B[k];
	nnz++;
      } else {
	info++;
      }
    }
  }

  delete [] irowB;
  delete [] B;
}

void PetscSparseStorage::fromGetSpRow( int row, int col,
				       double A[], int /*lenA */,
				       int jcolA[], int& nnz,
				       int colExtent, int& info )
{
  const double * B;
  const int    * jcolB;
  int k, ierr; 

  int m, n;
  this->getSize(m,n);
  ierr = MatGetRow(M, row, &nnz, &jcolB, &B ); assert( ierr == 0 );
  if( col == 0 && colExtent == n ) {
    // We want the whole row, just do a simple copy
    for( k = 0; k < nnz; k++ ) {
      A[k]     = B[k];
      jcolA[k] = jcolB[k];
    }
  } else { // Copy only those in range
    int i = 0;
    for( k = 0; k < nnz; k++ ) {
      if( jcolB[k] >= col && jcolB[k] < col + colExtent ) {
	A[i]     = B[k];
	jcolA[i] = jcolB[k];
	i++;
      }
    }
  }
  ierr = MatRestoreRow( M, row, &nnz, &jcolB, &B ); assert( ierr == 0 );
  info = 0;
}

void PetscSparseStorage::fatPutSpCol( int col, double A[],
				      int lenA, int irowA[],
				      int& info )
{
  int ierr;

  ierr = MatSetValues( M, lenA, irowA, 1, &col, A, INSERT_VALUES ); 
  assert(ierr == 0);
  info = 0;
}

void PetscSparseStorage::atPutSpCol( int col, double A[],
				     int lenA, int irowA[],
				     int& info )
{
  int ierr;

  ierr = 
    MatSetValues( M, lenA, irowA, 1, &col, A, INSERT_VALUES ); 
  assert(ierr == 0);
  ierr = MatAssemblyBegin( M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);
  ierr = MatAssemblyEnd( M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);

  info = 0;
}

void PetscSparseStorage::fatPutSpRow( int row, double A[],
				      int lenA, int jcolA[],
				      int& info )
{
  info = 
    MatSetValues( M, 1, &row, lenA, jcolA, A, INSERT_VALUES ); 
  assert(info == 0);

}

void PetscSparseStorage::atPutSpRow( int row, double A[],
				     int lenA, int jcolA[],
				     int& info )
{
  int ierr;

  ierr = 
    MatSetValues( M, 1, &row, lenA, jcolA, A, INSERT_VALUES ); 
  assert(ierr == 0);
  ierr = MatAssemblyBegin( M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);
  ierr = MatAssemblyEnd( M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);  

  info = 0;
}


void PetscSparseStorage::putSparseTriple( int irow[], int len,
					  int jcol[], double A[], 
					  int& info )
{
  int k, ierr;
  for( k = 0; k < len; k++ ) {
    ierr = MatSetValue(M, irow[k] , jcol[k], A[k], INSERT_VALUES );
    assert( ierr == 0 );
    //      if( (k + 1) % 1024 == 0 ) {
    //        ierr = MatAssemblyBegin( M, MAT_FINAL_ASSEMBLY  ); assert( ierr == 0 );
    //        ierr = MatAssemblyEnd( M, MAT_FINAL_ASSEMBLY  ); assert( ierr == 0 );
    //      }
  }

  ierr = MatAssemblyBegin( M, MAT_FINAL_ASSEMBLY ); assert( ierr == 0 );
  ierr = MatAssemblyEnd( M, MAT_FINAL_ASSEMBLY ); assert( ierr == 0 );

  info = 0;
}

PetscSparseStorage::~PetscSparseStorage()
{
  int ierr;

  if( !preserveMat ) {
    ierr = MatDestroy( M ); assert( ierr  == 0);
  }
}

void PetscSparseStorage::getSize( int& m, int& n ) 
{
  MatGetSize( M, &m, &n );
}

void PetscSparseStorage::atPutZeros( int row, int col,
				     int rowExtent, int colExtent )
{
  int m, n;
  this->getSize( m, n );
  if( 0 == row && 0 == col &&
      m == rowExtent && n == colExtent ) {
    int ierr;
    ierr = MatZeroEntries( M ); assert( ierr  == 0);
  } else {
    double * zeros = new double[colExtent];
    double * A     = new double[colExtent];
    int    * jcol  = new int   [colExtent];
	
    int i, nnz, info;
    for ( i = 0; i < colExtent; i++ ) {
      zeros[i] = 0.0;
    }

    for( i = row; i < row + rowExtent; i++ ) {
      // Both calls will always succeed.
      this->fromGetSpRow( i, col, A, colExtent, jcol, nnz,
			  colExtent, info );
      this->atPutSpRow( i, zeros, nnz, jcol, info );
    }

    delete [] jcol;
    delete [] A;
    delete [] zeros;
  }
}

void PetscSparseStorage::genmult ( double beta,  PetscVector& py,
				   double alpha, PetscVector& px,
				   int trans )
{
  int ierr;
  int my, nx;
  if( trans ) {
    this->getSize( nx, my );
  } else {
    this->getSize( my, nx );
  }

  if( alpha == 0.0 ) {
    py.scale( beta );
  } else {
    if ( alpha != 1.0 || beta != 1.0 ) {
      py.scale( beta/alpha );
    }

    if( trans ) {
      ierr = MatMultTransposeAdd(M, px.pv, py.pv, py.pv ); assert( ierr  == 0);
    } else {
      ierr = MatMultAdd(M, px.pv, py.pv, py.pv ); assert( ierr  == 0);
    }
    if( alpha != 1.0 ) {
      py.scale(alpha);
    }
  }
}

void PetscSparseStorage::atPutDiagonal( int idiag,
					OoqpVector& vec_in )
{
  int ierr;
  PetscVector & vec = dynamic_cast<PetscVector &>(vec_in);

  int low, high;
  ierr =  VecGetOwnershipRange( vec.pv, &low, &high ); assert( ierr  == 0);
  double * a; 
  ierr = VecGetArray( vec.pv, &a ); assert( ierr  == 0);
  int extent = vec.getLocalSize();
  int k;
  for( k = 0; k < extent; k++ ) {
    //if ( a[k] != 0 ) {
    ierr = MatSetValue( M, k + low + idiag,
			k + low + idiag, a[k], INSERT_VALUES );
    assert( ierr == 0 );
    //}
  }
  ierr = VecRestoreArray( vec.pv, &a ); assert( ierr  == 0);

  ierr = MatAssemblyBegin( M, MAT_FINAL_ASSEMBLY ); assert( ierr  == 0);
  ierr = MatAssemblyEnd  ( M, MAT_FINAL_ASSEMBLY ); assert( ierr  == 0);
}

void PetscSparseStorage::fromGetDiagonal( int idiag,
					  OoqpVector& vec_in )
{
  int ierr;
  PetscVector & vec = dynamic_cast<PetscVector &>(vec_in);

  int low, high;
  ierr =  VecGetOwnershipRange( vec.pv, &low, &high ); assert( ierr  == 0);
  double * a; 
  ierr = VecGetArray( vec.pv, &a ); assert( ierr  == 0);
  int extent = vec.getLocalSize();
  int k;

  int kdiag;
  for( k = 0; k < extent; k++ ) {
    kdiag = k + low + idiag;
    ierr = MatGetValues(M, 1 ,&kdiag, 1, &kdiag, &a[k]); assert(ierr == 0);
  }
  
  ierr = VecRestoreArray( vec.pv, &a ); assert( ierr  == 0);

}

void PetscSparseStorage::writeToStream(ostream& out) const
{
  int ierr;
  int low, high;
  ierr = MatGetOwnershipRange( M, &low, &high ); assert(ierr == 0);
  int row;
  for( row = low; row < high; row++ ) {
    const PetscScalar * a;
    const PetscInt * ja;
    int   nnz;

    ierr = MatGetRow( M, row, &nnz, &ja, &a ); assert(ierr == 0);
    int k;
    for( k = 0; k < nnz; k++ ) {
      out << row << "    " << ja[k] << "   " << a[k] << endl;
    }
    ierr = MatRestoreRow( M, row, &nnz, &ja, &a ); assert(ierr == 0);
  }
}



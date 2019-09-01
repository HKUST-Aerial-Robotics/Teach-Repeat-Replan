/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "PetscSpSymMatrix.h"
#include "PetscSparseStorage.h"
#include <cassert>
#include "PetscVector.h"
#include "DoubleMatrixTypes.h"

int PetscSpSymMatrix::isKindOf( int type )
{
  return type == kPetscSpSymMatrix || type == kSymMatrix ||
    type == kPetscSpMatrix;
}

PetscSpSymMatrix::PetscSpSymMatrix( int size, int nnz )
{
  mStorage =
    PetscSparseStorageHandle( new PetscSparseStorage( PETSC_DECIDE,
							  PETSC_DECIDE,
							  size, size, nnz) );
}

PetscSpSymMatrix::PetscSpSymMatrix( int lm, int ln, int size, int nnz )
{
  mStorage =
    PetscSparseStorageHandle( new PetscSparseStorage( lm, ln,
							  size, size, nnz) );
}

PetscSpSymMatrix::PetscSpSymMatrix( Mat M )
{
  mStorage = PetscSparseStorageHandle( new PetscSparseStorage( M ) );
}


void PetscSpSymMatrix::getDiagonal( OoqpVector& vec )
{
  mStorage->getDiagonal( vec );
}

void PetscSpSymMatrix::setToDiagonal( OoqpVector& vec )
{
  mStorage->setToDiagonal( vec );
}

void PetscSpSymMatrix::atPutDense( int /* row */, int /* col */,
				   double * /* A */, int /* lda */,
				   int /* rowExtent */,
				   int /* colExtent */ )
{
  assert( "Not implemented" && 0 );
}

void PetscSpSymMatrix::fromGetDense( int row, int col,
				     double * A, int lda,
				     int rowExtent,
				     int colExtent )
{
  mStorage->fromGetDense( row, col, A, lda, rowExtent, colExtent );
}

void PetscSpSymMatrix::randomizePSD(double * /* seed */)
{
  assert( "Not implemented" && 0 );
}
  
void PetscSpSymMatrix::symAtPutSpRow( int row, double A[],
					  int lenA, int jcolA[],
					  int& info )
{
  int ierr;

  this->fsymAtPutSpRow( row, A, lenA, jcolA, info );

  ierr = MatAssemblyBegin( mStorage->M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);
  ierr = MatAssemblyEnd(   mStorage->M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);  
}

void PetscSpSymMatrix::fsymAtPutSpRow( int row, double A[],
					   int lenA, int jcolA[],
					   int& info )
{
  int lA = lenA;
  
  // Find the part of A in the lower triangle
  while ( lA > 0 && jcolA[lA - 1] > row ) lA--;

  
  // We are symmetric
  mStorage->fatPutSpRow( row, &A[0], lA, &jcolA[0], info );
  if ( info == 0 ) {
    if( lA > 0 && jcolA[lA - 1] == row ) lA--;
    mStorage->fatPutSpCol( row, &A[0], lA, &jcolA[0], info );
  }
}

void PetscSpSymMatrix::fromGetSpRow( int row, int col,
					 double A[], int lenA,
					 int jcolA[], int& nnz,
					 int colExtent, int& info )
{
  mStorage->fromGetSpRow( row, col, A, lenA, jcolA, nnz, colExtent, info );
}


void PetscSpSymMatrix::symAtPutSubmatrix( int destRow, int destCol,
					      DoubleMatrix& M,
					      int srcRow, int srcCol,
					      int rowExtent, int colExtent )
{
  int m, n;
  mStorage->getSize( m, n );
  if ( M.isKindOf( kPetscSpMatrix ) &&
       srcRow == 0 && srcCol == 0 && 
       destRow == 0  && destCol == 0 &&
       rowExtent == m && colExtent == n ) {
    // We can do a quick copy
    PetscSpSymMatrix & mb = (PetscSpSymMatrix &) M;
    PetscSparseStorageHandle ms( mb.getStorage() );
    int ierr; 
    ierr = MatCopy( ms->M, mStorage->M,  DIFFERENT_NONZERO_PATTERN );
    assert(ierr == 0);
  } else {
    int ierr, first, lastp1;
    ierr = MatGetOwnershipRange( mStorage->M, &first, &lastp1 ); assert(ierr == 0);
    if( srcRow < first ) {
      destRow    += first - srcRow;
      rowExtent  -= first - srcRow;
      srcRow      = first;
    }
    if( srcRow + rowExtent > lastp1 ) rowExtent = lastp1 - srcRow; 
	
    int i, k;
    int info, nnz;
	
    if( rowExtent > 0 && colExtent > 0 ) {
      int *    ja = new int[colExtent];
      double * a  = new double[colExtent];
	  
      nnz = 0;
      for ( i = 0; i < rowExtent; i++ ) {
	M.fromGetSpRow( srcRow + i, srcCol, a, colExtent, ja,
			 nnz, colExtent, info );
	for( k = 0; k < nnz; k++ ) {
	  ja[k] += (destCol - srcCol);
	}
	this->fsymAtPutSpRow( destRow + i, a, nnz, ja, info );
      }
	  
      delete [] ja;
      delete [] a;
    }
    ierr = MatAssemblyBegin( mStorage->M, MAT_FINAL_ASSEMBLY );
    assert(ierr == 0);
    ierr = MatAssemblyEnd(   mStorage->M, MAT_FINAL_ASSEMBLY );
    assert(ierr == 0);
  }
}

void PetscSpSymMatrix::putSparseTriple( int irow[], int len,
					    int jcol[], double A[], 
					    int& info )
{
  int k, ierr;
  for( k = 0; k < len; k++ ) {
    ierr = MatSetValue(mStorage->M, irow[k] , jcol[k], A[k], INSERT_VALUES );
    assert( ierr == 0 );
    ierr = MatSetValue(mStorage->M, jcol[k] , irow[k], A[k], INSERT_VALUES );
    assert( ierr == 0 );
//      if( (k + 1) % 1024 == 0 ) {
//        ierr = MatAssemblyBegin( mStorage->M, MAT_FINAL_ASSEMBLY  );
//        assert( ierr == 0 );
//        ierr = MatAssemblyEnd( mStorage->M, MAT_FINAL_ASSEMBLY  );
//        assert( ierr == 0 );
//      }
  }
  //if( k % 1024 != 0 ) {
    ierr = MatAssemblyBegin( mStorage->M, MAT_FINAL_ASSEMBLY );
    assert( ierr == 0 );
    ierr = MatAssemblyEnd( mStorage->M, MAT_FINAL_ASSEMBLY );
    assert( ierr == 0 );
    //}
  info = 0;
}

// Pass these to storage
void PetscSpSymMatrix::getSize( int& m, int& n )
{
  mStorage->getSize( m, n );
}

int PetscSpSymMatrix::size()
{
  int m, n;
  mStorage->getSize( m, n );

  return m;
}

void PetscSpSymMatrix::atPutZeros( int row, int col,
				       int rowExtent, int colExtent )
{
  mStorage->atPutZeros( row, col, rowExtent, colExtent );
}
  
void PetscSpSymMatrix::transMult ( double beta,  OoqpVector& y_in,
				       double alpha, OoqpVector& x_in )
{
  // We are symmetric
  PetscVector & x = dynamic_cast<PetscVector &>(x_in);
  PetscVector & y = dynamic_cast<PetscVector &>(y_in);

  const int notTrans = 0; // This is a symmetric matrix
  mStorage->genmult( beta, y,  alpha, x, notTrans );
}

void PetscSpSymMatrix::mult ( double beta,  OoqpVector& y_in,
				  double alpha, OoqpVector& x_in )
{
  PetscVector & x = dynamic_cast<PetscVector &>(x_in);
  PetscVector & y = dynamic_cast<PetscVector &>(y_in);

  const int notTrans = 0;

  mStorage->genmult( beta, y, alpha, x, notTrans );
}

double PetscSpSymMatrix::abmaxnorm()
{ 
  double norm;

  int ierr = MatNorm(mStorage->M, NORM_INFINITY, &norm); assert(ierr == 0);

  return norm;
}

void PetscSpSymMatrix::writeToStream(ostream& out) const
{
  mStorage->writeToStream( out );
}


void PetscSpSymMatrix::atPutDiagonal( int idiag, OoqpVector& v )
{
  mStorage->atPutDiagonal( idiag, v );
}

void PetscSpSymMatrix::fromGetDiagonal( int idiag, OoqpVector& v )
{
  mStorage->fromGetDiagonal( idiag, v );
}


void PetscSpSymMatrix::SymmetricScale( OoqpVector& vec )
{
  mStorage->SymmetricScale( vec );
}

void PetscSpSymMatrix::ColumnScale( OoqpVector& vec )
{
  mStorage->ColumnScale( vec );
}

void PetscSpSymMatrix::RowScale( OoqpVector& vec )
{
  mStorage->RowScale( vec );
}

void PetscSpSymMatrix::scalarMult( double num )
{
  mStorage->scalarMult( num );
}

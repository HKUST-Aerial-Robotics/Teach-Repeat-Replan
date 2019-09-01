/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/*
 * Thanks to Quan H. Nguyen for help porting to newer versions of Petsc.
 */

#include "PetscSpGenMatrix.h"
#include "PetscSparseStorage.h"
#include <cassert>
#include "SimpleVector.h"
#include "PetscVector.h"

#include "DoubleMatrixTypes.h"

PetscSpGenMatrix::PetscSpGenMatrix( int m, int n, int nnz )
{
  mStorage =
    PetscSparseStorageHandle( new PetscSparseStorage( PETSC_DECIDE,
							  PETSC_DECIDE,
							  m, n, nnz)  );
}

PetscSpGenMatrix::PetscSpGenMatrix( Mat M )
{
  mStorage =
    PetscSparseStorageHandle( new PetscSparseStorage( M ) );
}

int PetscSpGenMatrix::isKindOf( int type )
{
  return type == kPetscSpGenMatrix || type == kPetscSpMatrix;
}

void PetscSpGenMatrix::getDiagonal( OoqpVector& vec )
{
  mStorage->getDiagonal( vec );
}

void PetscSpGenMatrix::setToDiagonal( OoqpVector& vec )
{
  mStorage->setToDiagonal( vec );
}

void PetscSpGenMatrix::atPutDense( int  row,     int  col,
				   double *  A , int  lda,
				   int  rowExtent ,
				   int  colExtent  )
{
  mStorage->atPutDense( row, col, A, lda, rowExtent, colExtent );
}

void PetscSpGenMatrix::fromGetDense( int  row,    int  col,
				     double *  A, int  lda,
				     int  rowExtent,
				     int  colExtent  )
{
  mStorage->fromGetDense( row, col, A, lda, rowExtent, colExtent );
}

void PetscSpGenMatrix::putSparseTriple( int irow[], int len,
					    int jcol[], double A[], 
					    int& info )
{
  mStorage->putSparseTriple( irow, len, jcol, A, info );
}

void PetscSpGenMatrix::fromGetSpRow( int row, int col,
					 double A[], int lenA,
					 int jcolA[], int& nnz,
					 int colExtent, int& info )
{
  mStorage->fromGetSpRow( row, col, &A[0], lenA,
			  &jcolA[0], nnz, colExtent, info );
}

void PetscSpGenMatrix::atPutSpRow( int col, double A[],
				       int lenA, int jcolA[], int& info )
{
  mStorage->atPutSpRow( col, &A[0], lenA, &jcolA[0], info );
}

void PetscSpGenMatrix::getSize( int& m, int& n )
{
  mStorage->getSize( m, n );
}

void PetscSpGenMatrix::atPutSubmatrix( int destRow, int destCol,
					   DoubleMatrix& M,
					   int srcRow, int srcCol,
					   int rowExtent, int colExtent )
{
  int i, k;
  int info, nnz;

  int *    ja = new int[colExtent];
  double * a =  new double[colExtent];

  nnz = 0;
  for ( i = 0; i < rowExtent; i++ ) {
    M.fromGetSpRow( srcRow + i, srcCol, a, colExtent, ja,
		     nnz, colExtent, info );
    for( k = 0; k < nnz; k++ ) {
      ja[k] += (destCol - srcCol);
    }
    mStorage->fatPutSpRow( destRow + i, a, nnz, ja, info );
  }
  int ierr;
  ierr = MatAssemblyBegin( mStorage->M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);
  ierr = MatAssemblyEnd( mStorage->M, MAT_FINAL_ASSEMBLY ); assert(ierr == 0);  

  delete [] ja;
  delete [] a;
}

void PetscSpGenMatrix::mult ( double beta,  OoqpVector& y_in,
				  double alpha, OoqpVector& x_in )
{
  const int notTrans = 0;
  PetscVector & y = dynamic_cast<PetscVector &>(y_in);
  PetscVector & x = dynamic_cast<PetscVector &>(x_in);
  
  mStorage->genmult( beta, y, alpha, x, notTrans );
}


void PetscSpGenMatrix::transMult ( double beta,  OoqpVector& y_in,
				       double alpha, OoqpVector& x_in )
{
  PetscVector & y = dynamic_cast<PetscVector &>(y_in);
  PetscVector & x = dynamic_cast<PetscVector &>(x_in);


  const int trans = 1;
  mStorage->genmult( beta, y, alpha, x, trans );
}




double PetscSpGenMatrix::abmaxnorm()
{
  double norm;

  int ierr = MatNorm(mStorage->M, NORM_INFINITY, &norm); assert(ierr == 0);

  return norm;
}


void PetscSpGenMatrix::writeToStream(ostream& out) const
{
  mStorage->writeToStream( out );
}


void PetscSpGenMatrix::randomize(double /* alpha */, double /* beta */,
					double * /* seed */)
{
  assert( "Not implemented" && 0 );
}


void PetscSpGenMatrix::atPutDiagonal( int idiag, OoqpVector& v )
{
  mStorage->atPutDiagonal( idiag, v );
}

void PetscSpGenMatrix::fromGetDiagonal( int idiag, OoqpVector& v )
{
  mStorage->fromGetDiagonal( idiag, v );
}

void PetscSpGenMatrix::SymmetricScale( OoqpVector& vec )
{
  mStorage->SymmetricScale( vec );
}

void PetscSpGenMatrix::ColumnScale( OoqpVector& vec )
{
  mStorage->ColumnScale( vec );
}

void PetscSpGenMatrix::RowScale( OoqpVector& vec )
{
  mStorage->RowScale( vec );
}

void PetscSpGenMatrix::scalarMult( double num )
{
  mStorage->scalarMult( num );
}

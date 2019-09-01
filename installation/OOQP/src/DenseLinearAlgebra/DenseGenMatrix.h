/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DENSEGENMATRIX_H
#define DENSEGENMATRIX_H

#include "DoubleMatrix.h"
#include "DenseStorage.h"
#include "DenseGenMatrixHandle.h"

class DoubleLinearSolver;

/** A class of dense, non-symmetric, possibly non-square, matrices.
 *  @ingroup DenseLinearAlgebra
 */
class DenseGenMatrix : public GenMatrix {
public:
  DenseStorageHandle mStorage;

  DenseGenMatrix( int size );
  DenseGenMatrix( int m, int n );
  DenseGenMatrix( double A[], int m, int n );
  
  virtual int isKindOf( int matType );

  virtual void getSize( int& m, int& n );

  virtual void atPutDense( int row, int col, double * A, int lda,
			   int rowExtent, int colExtent );

  /** Fill a region of this matrix with zeros. 
   *  
   *  The region starts at (row, col) and extends rowExtent rows 
   *  and colExtent columns.
   */
  virtual void atPutZeros( int row, int col,
			   int rowExtent, int colExtent );


  virtual void getDiagonal( OoqpVector& vec );
  virtual void setToDiagonal( OoqpVector& vec );

  virtual void atPutSubmatrix( int destRow, int destCol,
			       DoubleMatrix& M,
			       int srcRow, int srcCol,
			       int rowExtent, int colExtent );
  virtual void atPutSpRow( int row, double A[], int lenA, int jcolA[],
			   int& info );

  virtual void putSparseTriple( int irow[], int len, int jcol[], double A[], 
				int& info );

  virtual void mult ( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x );
  virtual void mult ( double beta,  double y[], int incy,
		      double alpha, double x[], int incx );

  virtual void transMult ( double beta,  OoqpVector& y,
			   double alpha, OoqpVector& x );
  virtual void transMult ( double beta,  double y[], int incy,
			   double alpha, double x[], int incx );

  virtual void fromGetDense( int row, int col, double * A, int lda,
			     int rowExtent, int colExtent );

  virtual void fromGetSpRow( int row, int col,
			     double A[], int lenA, int jcolA[], int& nnz,
			     int rowExtent, int& info );

  virtual void ColumnScale( OoqpVector& vec );
  virtual void scalarMult( double num);

  virtual double abmaxnorm();
  virtual void writeToStream(ostream& out) const;
  virtual void randomize( double alpha, double beta, double * seed );

  virtual void atPutDiagonal( int idiag, OoqpVector& v );
  virtual void fromGetDiagonal( int idiag, OoqpVector& v );
  /** Get a row of this matrix. */
  virtual void getRow ( int rowIndex, OoqpVector& v_in);

  double * operator[]( int index ) { return mStorage->M[index]; }

  const double * operator[]( int index ) const
  { return mStorage->M[index]; }

  /** Return a pointer to the first element in the matrix */
  double * elements() { return mStorage->M[0]; };
  /** Return mMat, an    */
  double **Mat() { return mStorage->M; };

  DenseStorage& storage() { return *mStorage; }
  DenseStorage *getStorage() { return mStorage.ptr(); }
};

#endif

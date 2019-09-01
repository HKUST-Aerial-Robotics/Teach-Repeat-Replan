/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DENSESYMMATRIX_H
#define DENSESYMMATRIX_H

#include "DenseStorage.h"
#include "DoubleMatrix.h"
#include "DenseSymMatrixHandle.h"

/** A class representing dense, symmetric matrices
 * @ingroup DenseLinearAlgebra
 */
class DenseSymMatrix : public SymMatrix {
public:
  DenseStorageHandle mStorage;
  
  DenseSymMatrix( int size );
  DenseSymMatrix( double Q[], int size );

  virtual int isKindOf( int matrixType );

  virtual void mult ( double beta,  double y[], int incy,
		      double alpha, double x[], int incx );
  virtual void mult ( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x);

  virtual void transMult ( double beta,  double y[], int incy,
			   double alpha, double x[], int incx );
  virtual void transMult ( double beta,  OoqpVector& y,
			   double alpha, OoqpVector& x );

  virtual void getSize( int& m, int& n );

  virtual double abmaxnorm();
  virtual void writeToStream(ostream& out) const;
  virtual void randomizePSD(double * seed);

  virtual void fromGetDense( int row, int col, double * A, int lda,
			     int rowExtent, int colExtent );
  
  virtual void fromGetSpRow( int row, int col,
			     double A[], int lenA, int jcolA[], int& nnz,
			     int rowExtent, int& info );

  virtual void SymmetricScale ( OoqpVector& vec );
  virtual void scalarMult( double num);

  virtual void symAtPutSpRow( int col, double A[], int lenA, int irowA[],
			      int& info );
  
  /** Insert the dense array symmetrically (the part that winds up
   *  in the lower triangle of this matrix is significant.)
   */
  virtual void symAtPutDense( int row, int col, double * A, int lda,
				     int rowExtent, int colExtent );
  /** Put a block of zeros into this matrix symmetrically (the part that
   *  winds up in the lower triangle of this matrix is significant.) */
  virtual void symAtPutZeros( int row, int col,
  			   int rowExtent, int colExtent );

  virtual void putSparseTriple( int irow[], int len, int jcol[], double A[], 
				int& info );

  virtual void atAddOuterProductOf( int row, int col, double alpha,
				    double * x, int incx, int nx );

  virtual void symAtPutSubmatrix( int destRow, int destCol,
				  DoubleMatrix& M,
				  int srcRow, int srcCol,
				  int rowExtent, int colExtent );

  virtual void getDiagonal( OoqpVector& vec );
  virtual void setToDiagonal( OoqpVector& vec );

  virtual void atPutDiagonal( int idiag, OoqpVector& v );
  virtual void fromGetDiagonal( int idiag, OoqpVector& v );

  static DenseSymMatrix * randomPSD( int n, double * seed );
 
  double * operator[]( int index ) { return mStorage->M[index]; }

   const double * operator[]( int index ) const
  { return mStorage->M[index]; }
  
  /** Return a pointer to the first element in the matrix */
  double * elements() { return mStorage->M[0]; };
  /** Return mMat, an    */
  double **Mat() { return mStorage->M; };
  
  virtual int size();

  DenseStorage& storage() { return *mStorage; }
  DenseStorage *  getStorage() { return mStorage.ptr(); }
};


#endif

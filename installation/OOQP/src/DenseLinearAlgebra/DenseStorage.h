/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DENSEDOUBLEMATRIX_H
#define DENSEDOUBLEMATRIX_H

#include "DoubleMatrix.h"
#include "DenseStorageHandle.h"

extern int DenseStorageInstances;

/** A class for manupulating the storage of dense matrices.
 *  @ingroup DenseLinearAlgebra
 */
class DenseStorage : public DoubleStorage {
protected:
  int neverDeleteElts;
public:
  int m;
  int n;
  double ** M;

  DenseStorage( int m, int n );
  DenseStorage( double A[], int m, int n );

  virtual ~DenseStorage();

  virtual void getSize( int& m, int& n );



  virtual void getDiagonal( OoqpVector& vec );
  virtual void setToDiagonal( OoqpVector& vec );

  virtual void atPutDense( int row, int col, double * A, int lda,
			   int rowExtent, int colExtent );

  virtual void fromGetDense( int row, int col, double * A, int lda,
			     int rowExtent, int colExtent );
  
  virtual void atPutZeros( int row, int col,
			   int rowExtent, int colExtent );

  virtual void atAddOuterProductOf( int row, int col, double alpha,
				    double * x, int incx, int nx );


  virtual void addToDiagonalAt( double alpha, double x[], int incx,
				int idiag, int extent );
  virtual void fromGetSpRow( int row, int col,
			     double A[], int lenA, int irowA[], int& nnz,
			     int rowExtent, int& info );

  virtual void ColumnScale( OoqpVector& vec );
  virtual void SymmetricScale( OoqpVector& vec );
  virtual void scalarMult( double num);

  virtual void atPutSpRow( int col, double A[], int lenA, int irowA[],
			   int& info );

  virtual void putSparseTriple( int irow[], int len, int jcol[], double A[], 
				int& info );

  virtual void atPutDiagonal(   int idiag, OoqpVector& v );
  virtual void fromGetDiagonal( int idiag, OoqpVector& v );
  virtual void atPutDiagonal( int idiag, double x[], int incx, int extent );
};
  
#endif

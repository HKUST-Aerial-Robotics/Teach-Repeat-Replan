/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SPARSESTORAGE_H
#define SPARSESTORAGE_H

#include "DoubleMatrix.h"
#include "SparseStorageHandle.h"
#include "OoqpVectorHandle.h"

#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;

/** A class for managing the matrix elements used by sparse matrices.
 *  @ingroup SparseLinearAlgebra
 */
class SparseStorage : public DoubleStorage {
protected:
  int neverDeleteElts;
  
public:
  static int instances;

  int m;
  int n;
  int len;
  int * jcolM;
  int * krowM;
  double * M;

  SparseStorage( int m_, int n_, int len_ );
  SparseStorage( int m_, int n_, int len_,
		     int * krowM_, int * jcolM_,
		     double * M_ );

  void shiftRows( int row, int shift, int& info );
  virtual void getSize( int& m, int& n );
  int rows() { return m; }
  int cols() { return n; }

  int length() { return len; };
  int numberOfNonZeros() {	return krowM[m]; };
  virtual void fromGetDense( int row, int col, double * A, int lda,
			     int rowExtent, int colExtent );
  virtual void atPutDense( int row, int col, double * A, int lda,
			   int rowExtent, int colExtent );

  virtual void putSparseTriple( int irow[], int len, int jcol[], double A[], 
				int& info );

  virtual void getDiagonal( OoqpVector& vec );
  virtual void setToDiagonal( OoqpVector& vec );

  virtual void ColumnScale( OoqpVector& vec );
  virtual void SymmetricScale( OoqpVector& vec );
  virtual void scalarMult( double num);

  virtual void atPutSpRow( int col, double A[], int lenA, int irowA[],
			   int& info );

  virtual void fromGetSpRow( int row, int col,
			     double A[], int lenA, int irowA[], int& nnz,
			     int rowExtent, int& info );

  virtual void randomize( double alpha, double beta, double * seed );

  virtual void getTransposePat( int row, int col, int rowExtent, int colExtent,
				int kpat[], int krowM[], int jcolM[] );
  virtual void getFromPat( double data[], int n, int kpat[] );
  virtual void mult( double beta,  double y[], int incy,
		     double alpha, double x[], int incx );

  virtual void transMult ( double beta,  double y[], int incy,
			   double alpha, double x[], int incx );

  virtual void atPutDiagonal( int idiag, OoqpVector& v );
  virtual void fromGetDiagonal( int idiag, OoqpVector& v );

  virtual void atPutDiagonal( int idiag,
			      double x[], int incx, int extent );

  virtual void writeToStream(ostream& out) const;

  virtual void symmetrize( int& info);
  virtual double abmaxnorm();

  virtual ~SparseStorage();
};

#endif

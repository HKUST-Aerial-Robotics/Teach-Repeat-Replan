/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */
 
/*
 * Edited by: Quan H. Nguyen 
 */
#ifndef PETSCSPARSESTORAGE_H
#define PETSCSPARSESTORAGE_H

#include "petscmat.h"
#include "DoubleMatrix.h"
#include "OoqpVectorHandle.h"
#include "PetscVectorHandle.h"

class PetscSparseStorage : public DoubleStorage {
protected:
  int preserveMat;
public:
  Mat M;

  PetscSparseStorage( int lm, int ln, int m, int n, int nnz );
  PetscSparseStorage( Mat M_ );

  virtual void atPutDense( int row, int col, double * A, int lda,
			   int rowExtent, int colExtent );
  virtual void fromGetDense( int row, int col, double * A, int lda,
			     int rowExtent, int colExtent );
  virtual void fromGetSpCol( int row, int col,
			     double A[], int lenA, int irowA[], int& nnz,
			     int rowExtent, int& info );
  virtual void fromGetSpRow( int row, int col,
			     double A[], int lenA, int jcolA[], int& nnz,
			     int colExtent, int& info );
  virtual void atPutSpCol( int col, double A[], int lenA, int irowA[],
			   int& info );
  virtual void atPutSpRow( int col, double A[], int lenA, int irowA[],
			   int& info );
  
  virtual void putSparseTriple( int irow[], int len, int jcol[], double A[], 
				int& info );

  virtual void fatPutSpCol( int col, double A[], int lenA, int irowA[],
			    int& info );
  virtual void fatPutSpRow( int col, double A[], int lenA, int irowA[],
			    int& info );
  virtual void atPutZeros( int row, int col, int rowExtent, int colExtent );

  virtual void getSize( int& m, int& n );

  virtual void getDiagonal( OoqpVector& vec );
  virtual void setToDiagonal( OoqpVector& vec );

  virtual void genmult ( double beta,  PetscVector& y,
			 double alpha, PetscVector& x,
			 int trans );

  virtual void atPutDiagonal( int idiag, OoqpVector& v );
  virtual void fromGetDiagonal( int idiag, OoqpVector& v );
  virtual void writeToStream(ostream& out) const;

  virtual void SymmetricScale ( OoqpVector& vec ){};
  virtual void ColumnScale ( OoqpVector& vec ){};
  virtual void RowScale ( OoqpVector& vec ){};
  virtual void scalarMult( double num){};
  virtual ~PetscSparseStorage();
};

typedef SmartPointer<PetscSparseStorage> PetscSparseStorageHandle;

#endif

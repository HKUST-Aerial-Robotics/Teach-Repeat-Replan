/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCSPSYMMATRIX_H
#define PETSCSPSYMMATRIX_H

#include "DoubleMatrix.h"
#include "PetscSparseStorage.h"
#include "petscmat.h"

class DenseSymMatrix;

class PetscSpSymMatrix : public SymMatrix {
protected:
  PetscSparseStorageHandle mStorage;
public:
  PetscSpSymMatrix( int size, int nnz );
  PetscSpSymMatrix( int lm, int ln, int size, int nnz );
  PetscSpSymMatrix( Mat M );

  PetscSparseStorage * getStorage() { return mStorage.ptr(); };
  PetscSparseStorage& storage() { return *mStorage; }

  virtual int isKindOf( int type );
  virtual void atPutDense( int row, int col, double * A, int lda,
			   int rowExtent, int colExtent );
  virtual void fromGetDense( int row, int col, double * A, int lda,
			     int rowExtent, int colExtent );

  virtual void symAtPutSpRow( int row, double A[], int lenA, int jcolA[],
			      int& info );

  virtual void fsymAtPutSpRow( int row, double A[], int lenA, int jcolA[],
			       int& info );

  virtual void getSize( int& m, int& n );

  virtual int size();

  virtual void symAtPutSubmatrix( int destRow, int destCol,
				  DoubleMatrix& M,
				  int srcRow, int srcCol,
				  int rowExtent, int colExtent );

  virtual void fromGetSpRow( int row, int col,
                             double A[], int lenA, int irowA[], int& nnz,
                             int rowExtent, int& info );

  virtual void atPutZeros( int row, int col,
			   int rowExtent, int colExtent );
  virtual void mult ( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x );
  virtual void transMult ( double beta,  OoqpVector& y,
			   double alpha, OoqpVector& x );
  
  virtual double abmaxnorm();
  
  virtual void writeToStream(ostream& out) const;

  virtual void randomizePSD(double * seed);
  
  virtual void getDiagonal( OoqpVector& vec );
  virtual void setToDiagonal( OoqpVector& vec );
  virtual void atPutDiagonal( int idiag, OoqpVector& v );
  virtual void fromGetDiagonal( int idiag, OoqpVector& x );

  virtual void putSparseTriple( int irow[], int len, int jcol[], double A[], 
				int& info );

  virtual void SymmetricScale ( OoqpVector& vec );
  virtual void ColumnScale ( OoqpVector& vec );
  virtual void RowScale ( OoqpVector& vec );
  virtual void scalarMult( double num);
  
  
  virtual ~PetscSpSymMatrix() {};
};

typedef SmartPointer<PetscSpSymMatrix> PetscSpSymMatrixHandle;

#endif

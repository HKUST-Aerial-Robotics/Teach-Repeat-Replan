/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCSPGENMATRIXBODY 
#define PETSCSPGENMATRIXBODY

#include "DoubleMatrix.h"
#include "PetscSpGenMatrixHandle.h"
#include "PetscSparseStorageHandle.h"
#include "petscmat.h"


class DenseSymMatrix;

class PetscSpGenMatrix : public GenMatrix {
protected:
  PetscSparseStorageHandle mStorage;
public:
  PetscSparseStorage * getStorage() { return mStorage.ptr(); };
  PetscSparseStorage& storage() { return *mStorage; }
  PetscSpGenMatrix( int m, int n, int nnz );
  PetscSpGenMatrix( Mat M );

  virtual int isKindOf( int type );

  virtual void atPutDense( int row, int col, double * A, int lda,
			   int rowExtent, int colExtent );
  virtual void fromGetDense( int row, int col, double * A, int lda,
			     int rowExtent, int colExtent );
  virtual void fromGetSpRow( int row, int col,
			     double A[], int lenA, int jcolA[], int& nnz,
			     int colExtent, int& info );
  virtual void atPutSpRow( int row, double A[], int lenA, int jcolA[],
			   int& info );

  virtual void getSize( int& m, int& n );

  virtual void atPutSubmatrix( int destRow, int destCol, DoubleMatrix& M,
			       int srcRow, int srcCol,
			       int rowExtent, int colExtent );

  virtual void mult ( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x );

  virtual void transMult ( double beta,  OoqpVector& y,
			   double alpha, OoqpVector& x );

  virtual void getDiagonal( OoqpVector& vec );
  virtual void setToDiagonal( OoqpVector& vec );

  virtual void atPutDiagonal( int idiag, OoqpVector& v );
  virtual void fromGetDiagonal( int idiag, OoqpVector& v );

  virtual double abmaxnorm();

  virtual void writeToStream(ostream& out) const;

  virtual void randomize(double alpha, double beta, double * seed);

  virtual void putSparseTriple( int irow[], int len, int jcol[], double A[], 
				int& info );

  virtual void SymmetricScale ( OoqpVector& vec );
  virtual void ColumnScale ( OoqpVector& vec );
  virtual void RowScale ( OoqpVector& vec );
  virtual void scalarMult( double num);
  
  virtual ~PetscSpGenMatrix() {};
};


#endif


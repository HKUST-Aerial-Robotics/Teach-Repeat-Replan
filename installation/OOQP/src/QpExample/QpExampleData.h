/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPEXAMPLEDATA_H
#define QPEXAMPLEDATA_H

#include "Data.h"

#include "DoubleMatrixHandle.h"
#include "DenseSymMatrixHandle.h"
#include "OoqpVectorHandle.h"


#include "OoqpVector.h"
#include "DoubleMatrix.h"
class LinearAlgebraPackage;

/**
 * Data for the example QP formulation.
 * 
 * @ingroup QpExample
 */


class QpExampleData : public Data
{
protected:
  QpExampleData() {};

  LinearAlgebraPackage * la;

  /** number of x components */
  int nx;

  /** number of equality constraints */
  int my;

  /** number of inequality constraints */
  int mz;

  /** Hessian of objective */
  SymMatrix * Q;

  /** matrix of equality consttaints */
  GenMatrix * A;

  /** matrix of inequality constraints */
  GenMatrix * C;
 
  /** linear term in the objective */
  OoqpVector * c; 

  /** right-hand side of equality constraints */
  OoqpVector * b;

  /** right-hand side of inequality constraints */
  OoqpVector * d;
public:

  QpExampleData(LinearAlgebraPackage * la, int nx_in, int my_in, int mz_in);
  virtual ~QpExampleData();

  virtual double datanorm();

  virtual void datainput() { };
  virtual void datarandom();
  virtual void print();

  /** put Hessian Q into coefficient matrix of linear system to be
   * solved at each interior-point iteration */
  void putQIntoAt( SymMatrix& M, int row, int col )
  {
    M.symAtPutSubmatrix( row, col, *Q, 0, 0, nx, nx );
  }

  /** put equality constraint matrix A into coefficient matrix of
   * linear system to be solved at each interior-point iteration */
  void putAIntoAt( SymMatrix& M, int row, int col )
  {
    M.symAtPutSubmatrix( row, col, *A, 0, 0, my, nx );
  }

  /** put inequality constraint matrix C into coefficient matrix of
   * linear system to be solved at each interior-point iteration */
  void putCIntoAt( SymMatrix& M, int row, int col )
  {
    M.symAtPutSubmatrix( row, col, *C, 0, 0, mz, nx );
  }

  /** y <- beta * y + alpha * Q * x */
  void Qmult( double beta,  OoqpVector& y,
	      double alpha, OoqpVector& x )
  {
    Q->mult( beta, y, alpha, x );
  }

  /** y <- beta * y + alpha * A * x */
  void Amult( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x)
  {
    A->mult( beta, y, alpha, x );
  }

  /** y <- beta * y + alpha * C * x   */
  void Cmult( double beta,  OoqpVector& y,
	      double alpha, OoqpVector& x )
  {
    C->mult( beta, y, alpha, x );
  }
  
  /** y <- beta * y + alpha * A\T * x */
  void ATransmult( double beta,  OoqpVector& y,
			   double alpha, OoqpVector& x )
  {
    A->transMult( beta, y, alpha, x );
  }

  /** y = beta * y + alpha * C\T * x */
  void CTransmult( double beta,  OoqpVector& y,
		   double alpha, OoqpVector& x )
  {
    C->transMult( beta, y, alpha, x);
  }

  /** returns the linear term from the objective */
  void getg( OoqpVector& c_out ) { c_out.copyFrom( *c ); }

  /** returns right-hand side of equality constraints */
  void getb( OoqpVector& b_out ) { b_out.copyFrom( *b ); }

  /** returns right-hand side of inequality constraints */
  void getd( OoqpVector& d_out ) { d_out.copyFrom( *d ); }

  GenMatrix * getC() { IotrAddRef(&C); return C; }
};

#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPBOUNDDATA_H
#define QPBOUNDDATA_H

#include "Data.h"

#include "DoubleMatrixHandle.h"
#include "OoqpVectorHandle.h"
#include "OoqpVector.h"


class QpBound;
class MpsReader;

/**
 * Data for the bounded QP formulation.
 * 
 * @ingroup QpBound
 */

class QpBoundData : public Data 
{

protected:
  QpBound * factory;

  /** number of elements in x */
  int nx;

  /** number of actual lower bound constraints */
  int nlower;

  /** number of actual upper bound constraints */
  int nupper;

  /** Hessian of the objective */
  SymMatrixHandle mQ;

  /** scalar constant term in the obejctive */ 
  double alpha;

  /** linear term in the objective */
  OoqpVectorHandle c; 

  /** vector of lower bounds */
  OoqpVectorHandle lower;

  /** vector of upper bounds */
  OoqpVectorHandle upper;

  /** vector indicating locations of the finite components of the
   *  lower bound vector l */
  OoqpVectorHandle index_lower;

  /** vector indicating locations of the finite components of the
   *  upper bound vector l */
  OoqpVectorHandle index_upper;

public:

  QpBoundData( QpBound * f, SymMatrix * Q, int n_in );

  /** constructor in which storage is allocated externally 
   *
   * @param f contains the QpBoundData object constructed from the
   * other parameters */
  QpBoundData( QpBound * f, double alpha,
	       OoqpVector * g, SymMatrix * Q,
	       OoqpVector * l, OoqpVector * index_lower_,
	       OoqpVector * u, OoqpVector * index_upper_ );
  virtual ~QpBoundData();

  
  /** calculate objective function at given set of mariables */
  virtual double objectiveValue( Variables * vars );

  /** stuff the Hessian into the coefficient matrix M of the linear
   * system to be solved at each interior-point iteration. M is stored
   * as a symmetric matrix. */
  virtual void putQIntoAt( SymMatrix& M, int row, int col );

  /** stuff the Hessian into the coefficient matrix M of the linear
   * system to be solved at each interior-point iteration. M is stored
   * as a general matrix. */
  virtual void putQIntoAt( GenMatrix& M, int row, int col );

  /** perform matrix-vector multiplication with the Hessian Q: y <-
   * beta * y + alpha * Q * x */
  virtual void Qmult( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x );

  /** return problem dimension */
  virtual int getN();

  /** returns the actual number of lower bounds */
  virtual int getNlower();

  /** returns the actual number of upper bounds */
  virtual int getNupper();

  /** returns the linear term  from objective function */
  virtual void getg ( OoqpVector& c );

  /** returns the vector of lower bounds */
  virtual void getl ( OoqpVector& l );

  /** returns the vector of upper bounds */
  virtual void getu ( OoqpVector& u );

  virtual double datanorm();

  virtual void datainput() { };

  /** get problem data from a QPS file */
  virtual void datainput(MpsReader * reader, int& ierr );

  /** generate random problem data */
  virtual void datarandom();

  /** print the problem data */
  virtual void print();

  /** return indices of defined lower bounds */
  OoqpVector * getIndexLower() { return SpAsPointer(index_lower); };

  /** return indices of defined upper bounds */
  OoqpVector * getIndexUpper() { return SpAsPointer(index_upper); };

};

#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SVMLINSYS_H
#define SVMLINSYS_H

#include "LinearSystem.h"
#include "SimpleVectorHandle.h"

class Data;
class Variables;
class Residuals;
class DoubleLinearSolver;

/**
 * @ingroup Svm
 *
 * LinearSystem class for Svm.
 *
 */

#include "DenseSymMatrixHandle.h"
  
class SvmLinsys : public LinearSystem
{
private:
  /** stores Cholesky factor of the mtrix (X^T DX) that is calculated
      in factor */
  DenseSymMatrixHandle L;

  /** pointer to dense symmetric positive definite solver used to
      factor the compressed matrix */
  DoubleLinearSolver *solver;

  /** stores the complicated diagonal matrix that arises during the
   *  block elimination */
  SimpleVectorHandle mDinv;

  /** right-hand side for the system */
  SimpleVectorHandle mRhs;
public:

  /** dimension of Euclidean space in which each observation resides */
  int hyperplanedim;

  /** number of observations */
  int nobservations;

  SvmLinsys();
  ~SvmLinsys();
  void factor(Data *prob, Variables *vars);

  void solve(Data *prob, Variables *vars, Residuals *rhs,
	     Variables *step);

  /** allocates internal storage for the matrix and right-hand side
      obtained after block elimination */
  void create(Data *prob);
};
  

#endif

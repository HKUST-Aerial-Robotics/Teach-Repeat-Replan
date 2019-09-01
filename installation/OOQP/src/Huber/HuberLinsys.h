/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef HuberLINSYS_H
#define HuberLINSYS_H

#include "LinearSystem.h"

class Data;
class Variables;
class Residuals;
class DoubleLinearSolver;

#include "DenseSymMatrixHandle.h"
#include "SimpleVectorHandle.h"

/**
 * @ingroup Huber
 *
 * LinearSystem class for Huber
 *
 */
  
class HuberLinsys : public LinearSystem
{
private:
  /** Stores the Cholesky factor of the matrix of the form (X^T DX)
   * that is calculated in factor.  */
  DenseSymMatrixHandle L;

  /** pointer to dense symmetric positive definite solver used to
      factor the compressed matrix */
  DoubleLinearSolver * solver;

  /** stores the complicated diagonal matrix that arises during the
   *  block elimination */
  SimpleVectorHandle mDinv;

  /** temporary storage in forming the coefficient matrix for the main
   * linear system */
  SimpleVectorHandle mDinvColX, mColOfL;
public:


  HuberLinsys( Data * prob );
  ~HuberLinsys();
  virtual void factor(Data *prob, Variables *vars);
  virtual void solve(Data *prob, Variables *vars, Residuals *rhs,
		     Variables *step);
};


#endif


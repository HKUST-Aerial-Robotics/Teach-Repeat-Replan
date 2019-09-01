/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPBOUNDDENSELINSYS
#define QPBOUNDDENSELINSYS

#include "QpBoundLinsys.h"

class DenseSymMatrix;

/** Linear system solvers for the bound-constrained QP formulation in
 * which the Hessian is stored as a dense matrix
 * 
 * @ingroup QpBound
 */

class QpBoundDenseLinsys : public QpBoundLinsys {
public:

  /** constructor simply calls the constructor from the parent class
   *
   * @see QpBoundLinsys */
  QpBoundDenseLinsys( QpBound * f, DenseSymMatrix * Mat,
		      DoubleLinearSolver * solver, OoqpVector * dq, int nx_,
		      OoqpVector * index_lower, OoqpVector * index_upper );

  /** sets up the matrix for the main linear system and calls an
   * Lapack-based routine to factor it
   *
   * @see QpBoundLinsys::factor
   * */
  virtual void factor(Data *prob_in, Variables *vars_in);
};

#endif

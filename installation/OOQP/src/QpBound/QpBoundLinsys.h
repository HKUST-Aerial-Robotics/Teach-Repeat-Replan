/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPBOUNDLINSYS_H
#define QPBOUNDLINSYS_H

#include "LinearSystem.h"
#include "DoubleMatrixHandle.h"
#include "OoqpVectorHandle.h"
#include "OoqpVector.h"

class Data;
class Variables;
class Residuals;
class DoubleLinearSolver;
class QpBound;

/** Linear system solvers for the bound-constrained QP
 * formulation. Contains material common to the sparse and dense
 * variants (though in this distribution we provide the implementation
 * only of the dense variant).
 *
 * @ingroup QpBound */

class QpBoundLinsys : public LinearSystem
{
protected:

  /** problem dimension */
  int nx;

  /** stores the coefficient matrix and (later) its factor */
  DoubleMatrixHandle Mat;  

  DoubleLinearSolver * solver;
  /* QpBoundLinsys() {}; */
  QpBound * factory;
  OoqpVectorHandle index_lower;
  OoqpVectorHandle index_upper;

  /** diagonal of the coefficient matrix */
  OoqpVectorHandle dq;

  /** modified diagonal */
  OoqpVectorHandle dd;
public:
  QpBoundLinsys( QpBound * f, DoubleMatrix * Mat,
		   DoubleLinearSolver * solver, OoqpVector * dq, int nx_,
		   OoqpVector * index_lower, OoqpVector * index_upper );
  virtual ~QpBoundLinsys();

  /** sets up the matrix for the main linear system. The actual
   * factorization is performed by a routine specific to either the
   * sparse or dense case.
   *
   * @see QpBoundDenseLinsys::factor
   * */
  void factor(Data *prob, Variables *vars);

  /** solve the system for the given set of redisuals */
  void solve(Data *prob, Variables *vars, Residuals *rhs,
	     Variables *step);

};
  

#endif

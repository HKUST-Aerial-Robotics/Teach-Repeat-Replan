/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPEXAMPLEDENSELINSYS_H
#define QPEXAMPLEDENSELINSYS_H

#include "LinearSystem.h"

class Data;
class Variables;
class Residuals;
class DoubleLinearSolver;
class LinearAlgebraPackage;
class OoqpVector;
#include "OoqpVectorHandle.h"
#include "DenseSymMatrixHandle.h"

/** Linear system solver for example QP formulation, assuming dense
 *  data.  
 *
 * @ingroup QpExample
 */
  
class QpExampleDenseLinsys : public LinearSystem
{
protected:

  /** stores the coefficient matrix in "augmented system" form, and
   * the factor */
  DenseSymMatrix * Mat;

  /** number of components in x */
  int nx;

  /** number of equality constraints */
  int my;

  /** number of inequality constraints */
  int mz;

  /** pointer to  LAPACK solver */
  DoubleLinearSolver * solver;

  QpExampleDenseLinsys(){};
  LinearAlgebraPackage * la;

public:

  QpExampleDenseLinsys( LinearAlgebraPackage * la, DenseSymMatrix *  Mat,
			DoubleLinearSolver * solver, int nx, int my, int mz );

  virtual ~QpExampleDenseLinsys();

  /** form the linear system and call the LAPACK-based routine to
   *  solve it */
  virtual void factor(Data *prob, Variables *vars);

  /** solve the system  for a given set of residuals */
  virtual void solve(Data *prob, Variables *vars, Residuals *rhs,
		     Variables *step);

  /** assembles a single vector object from two given vectors
   *
   * @param rhs (output) final joined vector
   * @param rhs1 (input) first part of rhs
   * @param rhs2 (input) last part of rhs
   */  
  virtual void joinRHS( OoqpVector& rhs1,
			OoqpVector& rhs2,
			OoqpVector& rhs );

/** extracts three component vectors from a given aggregated vector.
   *
   * @param vars (input) aggregated vector
   * @param vars1 (output) first part of vars
   * @param vars2 (output) last part of vars
   */
  virtual void separateVars( OoqpVector& vars,
			     OoqpVector& vars1,
			     OoqpVector& vars2 );
};
  

#endif


/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPGENLINSYS
#define QPGENLINSYS

#include "LinearSystem.h"
#include "OoqpVectorHandle.h"
#include "OoqpVector.h"

class Data;
class QpGenData;
class QpGen;
class Variables;
class Residuals;
class DoubleLinearSolver;
class LinearAlgebraPackage;


/** 
 * Linear System solvers for the general QP formulation. This class
 * contains definitions of methods and data common to the sparse and
 * dense special cases of the general formulation. The derived classes
 * QpGenSparseLinsys and QpGenDenseLinsys contain the aspects that are
 * specific to the sparse and dense forms.
 *
 * @see QpGenSparseLinsys
 * @see QpGenDenseLinsys
 *
 * @ingroup QpGen 
 */

class QpGenLinsys : public LinearSystem {
protected:

  /** stores a critical diagonal matrix as a vector */
  OoqpVectorHandle nomegaInv;

  /** right-hand side of the system */
  OoqpVectorHandle rhs;

  QpGen * factory;

  /** dimensions of the vectors in the general QP formulation */
  int nx, my, mz;

  /** temporary storage vectors */
  OoqpVectorHandle dd, dq;

  /** index matrices for the upper and lower bounds on x and Cx */
  OoqpVectorHandle ixupp, icupp, ixlow, iclow;

  /** dimensions of the upper and lower bound vectors */
  int nxupp, nxlow, mcupp, mclow;

public:
  QpGenLinsys(  QpGen * factory,
		QpGenData * data,
		LinearAlgebraPackage * la );

  virtual ~QpGenLinsys() {}


  /** sets up the matrix for the main linear system in "augmented
   * system" form. The actual factorization is performed by a routine
   * specific to either the sparse or dense case.
   *
   * @see QpGenSparseLinsys::factor
   * @see QpGenDenseLinsys::factor
   */
  virtual void factor(Data *prob, Variables *vars);

  /** solves the system for a given set of residuals. Assembles the
   * right-hand side appropriate to the matrix factored in factor,
   * solves the system using the factorization produced there,
   * partitions the solution vector into step components, then
   * recovers the step components eliminated during the block
   * elimination that produced the augmented system form 
   * 
   * @see QpGenSparseLinsys::solveCompressed
   * @see QpGenDenseLinsys::solveCompressed
*/
  virtual void solve(Data *prob, Variables *vars, Residuals *res,
		     Variables *step);

  /** assembles a single vector object from three given vectors
   *
   * @param rhs (output) final joined vector
   * @param rhs1 (input) first part of rhs
   * @param rhs2 (input) middle part of rhs
   * @param rhs3 (input) last part of rhs
   */
  virtual void joinRHS( OoqpVector& rhs,  OoqpVector& rhs1,
			OoqpVector& rhs2, OoqpVector& rhs3 );

  /** extracts three component vectors from a given aggregated vector.
   *
   * @param vars (input) aggregated vector
   * @param vars1 (output) first part of vars
   * @param vars2 (output) middle part of vars
   * @param vars3 (output) last part of vars
   */
  virtual void separateVars( OoqpVector& vars1, OoqpVector& vars2,
			     OoqpVector& vars3, OoqpVector& vars );

  /** assemble right-hand side of augmented system and call
      solveCompressed to solve it */
  virtual void solveXYZS( OoqpVector& stepx, OoqpVector& stepy,
			  OoqpVector& stepz, OoqpVector& steps,
			  OoqpVector& ztemp, QpGenData * data );

  /** perform the actual solve using the factors produced in factor.
   *
   * @param rhs on input contains the aggregated right-hand side of
   * the augmented system; on output contains the solution in
   * aggregated form 
   *
   * @see QpGenSparseLinsys::solveCompressed
   * @see QpGenDenseLinsys::solveCompressed
   */
  virtual void solveCompressed( OoqpVector& rhs ) = 0;

  /** places the diagonal resulting from the bounds on x into the
   * augmented system matrix */
  virtual void putXDiagonal( OoqpVector& xdiag ) = 0;

  /** places the diagonal resulting from the bounds on Cx into the
   * augmented system matrix */
  virtual void putZDiagonal( OoqpVector& zdiag ) = 0;

  /** computes the diagonal matrices in the augmented system from the
      current set of variables */
  virtual void computeDiagonals( OoqpVector& dd, OoqpVector& omega,
				 OoqpVector& t,  OoqpVector& lambda,
				 OoqpVector& u,  OoqpVector& pi,
				 OoqpVector& v,  OoqpVector& gamma,
				 OoqpVector& w,  OoqpVector& phi );
};

#endif

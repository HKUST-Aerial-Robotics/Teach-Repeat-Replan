/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPGENSPARSELINSYS
#define QPGENSPARSELINSYS

#include "QpGenLinsys.h"
#include "SparseSymMatrixHandle.h"

class DoubleLinearSolver;

/** 
 * implements the aspects of the solvers for sparse general QP
 * formulation that are specific to the sparse case.
 *
 * @ingroup QpGen 
 */
class QpGenSparseLinsys : public QpGenLinsys {
protected:
  SparseSymMatrixHandle Mat;
  DoubleLinearSolver * solver;
public:
  QpGenSparseLinsys(  QpGen * factory,
		QpGenData * data,
		LinearAlgebraPackage * la, SparseSymMatrix * Mat,
		DoubleLinearSolver * solver );

  /** perform the actual solve using the factors produced in factor.
   *
   * @param rhs on input contains the aggregated right-hand side of
   * the augmented system; on output contains the solution in
   * aggregated form
   */
  virtual void solveCompressed( OoqpVector& rhs );

  virtual void putXDiagonal( OoqpVector& xdiag );
  virtual void putZDiagonal( OoqpVector& zdiag );

  /** calls QpGenLinsys::factor to assemble the augmented system
   * matrix, then calls matrixChanged to factor it
   *
   * @see QpGenLinsys::factor 
   */
  virtual void factor(Data *prob, Variables *vars);
  
  virtual ~QpGenSparseLinsys();
};
#endif

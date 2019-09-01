/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DOUBLELINEARSOLVER_H
#define DOUBLELINEARSOLVER_H

#include "OoqpVectorHandle.h"

/**
 * @defgroup LinearSolvers
 */

/**
 * Implements the main solver for linear systems that arise in
 * primal-dual interior-point methods for QP.
 * @ingroup LinearSolvers 
 * @ingroup AbstractLinearAlgebra
 */
class DoubleLinearSolver {
public:

  /** called if the diagonal elements of the matrix have
   *  changed. Triggers a refactorization of the matrix, if necessary.
   *
   *  @param idiag index of the first diagonal element that
   *               changed
   *  @param extent the number of diagonal element that changed.  */
  virtual void diagonalChanged( int idiag, int extent ) = 0;

  /** called if some elements of the matrix have changed.  Triggers a
   *  refactorization of the matrix, if necessary.  */
  virtual void matrixChanged() = 0;

  /** solves a linear system.
   *
   * @param x on entry the right hand side of the system to be solved.
   *           On exit, the solution.  */
  virtual void solve ( OoqpVector& x ) = 0;

  /** Destructor  */
  virtual ~DoubleLinearSolver() {};
};

#endif





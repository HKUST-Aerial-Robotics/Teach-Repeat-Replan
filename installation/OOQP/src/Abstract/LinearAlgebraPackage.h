/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef LINEARALGEBRA
#define LINEARALGEBRA
/**
 * @defgroup AbstractLinearAlgebra
 *
 * Abstract base classes for linear algebra object (vectors/matrices/solvers).
 * @{
 */

#include "DoubleMatrixHandle.h"
class DoubleLinearSolver;
class OoqpVector;

/**
 * A class whose instances creates matrices and vectors of
 * an appropriate type. */
class LinearAlgebraPackage {
protected:
  LinearAlgebraPackage() {};
  virtual ~LinearAlgebraPackage() {};
public:
  /** Create a new symmetric matrix (of appropriate type). */
  virtual SymMatrix * newSymMatrix( int size, int nnz ) = 0;
  /** Create a new non-symmetric matrix (of appropriate type). */
  virtual GenMatrix * newGenMatrix( int m, int n, int nnz ) = 0;
  /** Create a new vector (of appropriate type.) */
  virtual OoqpVector * newVector( int n ) = 0;
};

/**
 * @}
 */
#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DEQPBOUNDFACTORY
#define DEQPBOUNDFACTORY

#include "QpBound.h"
class QpBoundData;

/**
 * @ingroup QpBound
 *
 * Derived class of QpBound in which the Hessian Q is stored as a
 * dense matrix */

class QpBoundDense : public QpBound {
public:
  QpBoundDense(int n);
  virtual LinearSystem    * makeLinsys( Data * prob_in );

  /** constructor to build QpBoundData structure, partly using storage
   *  already allocated 
   * 
   * @param alpha constant term in objective
   *
   * @param c linear term in objective
   *
   * @param Q Hessian matrix, lower triangle stored as a vector of doubles
   *
   * @param xlow lower bound vector
   *
   * @param ixlow vector of chars with nonzero element in positions
   * corresponding to components of x for which lower bound is
   * specified
   *
   * @param xupp upper bound vector
   *
   * @param ixupp vector of chars with nonzero element in positions
   * corresponding to components of x for which upper bound is
   * specified
   */
  virtual QpBoundData     * makeData( double alpha, double c[], double Q[],
				      double xlow[],  char ixlow[],
				      double xupp[],  char ixupp[] );
  virtual QpBoundData     * makeData();
};

#endif



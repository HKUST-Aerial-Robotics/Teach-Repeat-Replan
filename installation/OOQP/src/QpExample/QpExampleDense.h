/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPEXAMPLEDENSE_H
#define QPEXAMPLEDENSE_H

#include "ProblemFormulation.h"
class LinearAlgebraPackage;

/** @defgroup QpExample
 *
 * OOQP solver for general QPs formulated in the fashion discussed in
 * the user manual:
 *
 *  <pre>
 *  minimize    c' x + ( 1/2 ) x' * Q x         ; 
 *  subject to                      A x  = b    ;
 *                                  C x <= d    .
 *  </pre> 
 *
 * Data objects Q, A, C are stored as dense matrices.
 */


class QpExampleDense : public ProblemFormulation {
 protected:

  /** number of components of x */
  int mNx;

  /** number of equality constraints (rows of A and b) */
  int mMy;

  /** number of inequality constraints (rows of C and d) */
  int mMz;
  
  LinearAlgebraPackage * la;
 public: 
  QpExampleDense( int nx, int my, int mz );

  virtual Data            * makeData();
  virtual Data            * makeRandomData();
  virtual Residuals       * makeResiduals(Data * prob_in);
  virtual Variables       * makeVariables(Data * prob_in );
  virtual LinearSystem    * makeLinsys( Data * prob_in );
};

#endif

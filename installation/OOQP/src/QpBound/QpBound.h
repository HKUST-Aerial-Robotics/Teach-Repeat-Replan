/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPBOUNDFACTORY
#define QPBOUNDFACTORY

#include "ProblemFormulation.h"
#include "OoqpVectorHandle.h"

class LinearAlgebraPackage;

/**
 * @defgroup QpBound
 * 
 * OOQP's solver for bound-constrained convex quadratic programs
 *
 *  <pre>
 *  minimize    c' x + ( 1/2 ) x' * Q x         ; 
 *  subject to       l <= x <= u, 
 *  </pre> 
 *
 * where l and u are vectors that may contain infinite elements (that
 * is, upper and lower bounds need not exist for all components).  */


class QpBound : public ProblemFormulation {
protected:
  /** number of elements in x */
  int nx;
  
  LinearAlgebraPackage * la;
  QpBound( int nx );
public: 
  virtual Data            * makeRandomData();
  virtual Residuals       * makeResiduals(Data * prob_in);
  virtual Variables       * makeVariables(Data * prob_in);
  virtual LinearSystem    * makeLinsys(Data * prob_in) = 0;
  virtual OoqpVector  *  newPrimalVector();
  virtual ~QpBound();
};

#endif



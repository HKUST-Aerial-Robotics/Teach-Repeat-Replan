/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPEXAMPLEVARS_H
#define QPEXAMPLEVARS_H

#include "Variables.h"
class OoqpVector;
class LinearAlgebraPackage;

/**
 * Variables for the example QP formulation.
 * 
 * @ingroup QpExample
 */

class QpExampleVars : public Variables
{
protected:
  int nx, my, mz;
 public:

  /** primal variables */
  OoqpVector *x;

  /** primal slacks */
  OoqpVector *s;

  /** Lagrange multipliers for equality constraints */
  OoqpVector *y;

  /** Lagrange multipliers for inequality constraints */
  OoqpVector *z;

  QpExampleVars( LinearAlgebraPackage * la, int nx_in, int my_in, int mz_in);
  virtual ~QpExampleVars();


  virtual void saxpy( Variables *b, double alpha);
  virtual void negate();

  virtual double mu();
  virtual double mustep(Variables *step, double alpha);

/** calculate the largest alpha in (0,1] such that the nonnegative
   * variables stay nonnegative in the given search direction. In the
   * general QP problem formulation, this is the largest value of
   * alpha such that (s,z) + alpha * (b->s,b->z) >= 0.
   *
   * @see findBlocking */
  virtual double stepbound( Variables *b);
  virtual double violation();

/** Performs the same function as stepbound, and supplies additional
   * information about which component of the nonnegative variables is
   * responsible for restricting alpha. In terms of the abstract
   * formulation, the components have the following meanings.
   *
   * @param primalValue the value of the blocking component of the
   * primal variables s.
   * 
   * @param primalStep the corresponding value of the blocking
   * component of the primal step variables b->s.
   * 
   * @param dualValue the value of the blocking component of the dual
   * variables z.
   *
   * @param dualStep the corresponding value of the blocking component
   * of the dual step variables b->z.
   * 
   * @param firstOrSecond  1 if the primal step is blocking, 2 if the dual
   * step is block, 0 if no step is blocking.  
   *
   * @see stepbound
   */
  virtual double findBlocking( Variables * step, 
			       double & primalValue,
			       double & primalStep,
			       double & dualValue,
			       double & dualStep,
			       int& firstOrSecond );

  /** sets components of s to alpha and of z to beta */
  virtual void interiorPoint( double alpha, double beta );

  /** adds alpha to components of s and beta to components of z */
  virtual void shiftBoundVariables( double alpha, double beta );

  virtual void print();
  virtual double onenorm();
  virtual double infnorm();

  virtual void copy(Variables *b);
};

  
#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPBOUNDVARS_H
#define QPBOUNDVARS_H

#include "Variables.h"
#include "OoqpVectorHandle.h"

class QpBound;
class QpBoundData;

/**
 * Variables for the bound-constrained QP formulation.
 * 
 * @ingroup QpBound
 */

class QpBoundVars : public Variables
{
public:
  QpBound * factory;

  /** primal variables */
  OoqpVectorHandle x;

  /** lower bound slacks */
  OoqpVectorHandle t;

  /** upper bound slacks */
  OoqpVectorHandle v;

  /** lower bound multipliers */
  OoqpVectorHandle tau;

  /** upper bound multipliers */
  OoqpVectorHandle   nu; 
  
  /** lower bound index vector */
  OoqpVectorHandle index_lower;

  /** upper bound index vector */
  OoqpVectorHandle index_upper;

  /** problem dimension */
  int nx;

  /** actual number of lower bounds */
  int nlower;

  /** actual number of upper bounds */
  int nupper;

  /** constructor that allocates its own storage */
  QpBoundVars( QpBound * f, QpBoundData *prob );

  /** constructor that uses storage already allocated */
  QpBoundVars( QpBound * f, QpBoundData *prob,
	       OoqpVector * x, 
	       OoqpVector * t,    OoqpVector * v, 
	       OoqpVector * tau,  OoqpVector * nu );
  virtual ~QpBoundVars();

  /** calculates the complementarity gap */
  virtual double mu();


  virtual double mustep(Variables *step, double alpha);

  /** adds (alpha b) to the  current variables */
  virtual void saxpy( Variables *b, double alpha);

  virtual void negate();

/** Performs the same function as stepbound, and supplies additional
   * information about which component of the nonnegative variables is
   * responsible for restricting alpha. In terms of the abstract
   * formulation, the components have the following meanings.
   *
   * @param primalValue the value of the blocking component of the
   * primal variables (t,v).
   * 
   * @param primalStep the corresponding value of the blocking
   * component of the primal step variables (b->t,b->v).
   * 
   * @param dualValue the value of the blocking component of the dual
   * variables (tau,nu).
   *
   * @param dualStep the corresponding value of the blocking component
   * of the dual step variables (b->tau,b->nu).
   * 
   * @param firstOrSecond  1 if the primal step is blocking, 2 if the dual
   * step is block, 0 if no step is blocking.  
   *
   * @see stepbound
   * */
  virtual double findBlocking( Variables * step, 
			       double & primalValue,
			       double & primalStep,
			       double & dualValue,
			       double & dualStep,
			       int& firstOrSecond );
  
  /** calculate the largest alpha in (0,1] such that the nonnegative
   * variables stay nonnegative in the given search direction. In the
   * general QP problem formulation, this is the largest value of
   * alpha such that (t,v,tau,nu) + alpha *
   * (b->t,b->v,b->tau,b->nu) >= 0.
   *
   * @see findBlocking */
  virtual double stepbound( Variables *b);

  /** set components of (t,v) to alpha and of (tau,nu) to beta */
  virtual void interiorPoint( double alpha, double beta );

  /** add alpha to components of (t,v) and beta to components of
      (tau,nu) */
  virtual void shiftBoundVariables( double alpha, double beta );

  virtual double violation();


  virtual void print();

  virtual void copy(Variables *b);
  virtual double onenorm();
  virtual double infnorm();
};

/** Indicates what type is the blocking variable in the step length
 * determination. If tblock, then the blocking variable is one of the
 * slack variables t for a lower bound, and so on. Special value
 * no_block is for the case in which a step length of 1 can be taken
 * without hitting the bound.  */

enum { no_block = 0,
       t_block = 1,
       tau_block = 2,
       v_block = 3,
       nu_block = 4
};  

#endif

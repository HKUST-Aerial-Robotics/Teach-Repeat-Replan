/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef VARIABLES_H
#define VARIABLES_H

/**
 * @file Variables.h
 * @ingroup AbstractProblemFormulation
 */

class LinearSystem;
class Residuals;
class Data;
class Solver;
/**
 * Holds the variables used by the interior point solver. In terms of
 * in our abstract problem formulation, these variables are the
 * vectors x, y, z and s.
 *
 * @ingroup AbstractProblemFormulation */
class Variables
{
public:
  /** number of complementary primal-dual variables. */
  int nComplementaryVariables;
  
  /** compute complementarity gap, obtained by taking the inner
      product of the complementary vectors and dividing by the total
      number of components */
  virtual double mu() = 0;

  /** compute the complementarity gap resulting from a step of length
   * "alpha" along direction "step" */
  virtual double mustep(Variables *step, double alpha) = 0;

  /** negate the value of all the variables in this structure */
  virtual void negate() = 0;

  /** given variables b, compute a <- a + alpha b, where a are the
      variables in this class */
  virtual void saxpy( Variables *b, double alpha) = 0;
  
  /** calculate the largest alpha in (0,1] such that the nonnegative
   * variables stay nonnegative in the given search direction. In the
   * abstract problem formulation, this is the largest value of alpha
   * such that (s,z) + alpha * (b->s,b->z) >= 0.  
   *
   * @see findBlocking
   */
  virtual double stepbound( Variables *b ) = 0;

  /** Performs the same function as stepbound, and supplies additional
   * information about which component of the nonnegative variables is
   * responsible for restricting alpha. In terms of the abstract
   * formulation, the components have the following meanings.
   *
   * @param step  step in the variables
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
   * */
  virtual double findBlocking( Variables * step, 
			       double & primalValue,
			       double & primalStep,
			       double & dualValue,
			       double & dualStep,
			       int& firstOrSecond ) = 0;

  /** In the abstract QP formulation, sets s to alpha, z to beta and
   *  the other variable components to zero. */
  virtual void interiorPoint( double alpha, double beta ) = 0;

  /** In the standard QP formulation, sets s += alpha, z += beta */
  virtual void shiftBoundVariables( double alpha, double beta ) = 0;

  /** The amount by which the current variables violate the
   *  non-negativity constraints. */
  virtual double violation() = 0;

  /** print the variables */
  virtual void print();

  /** copy the variables */
  virtual void copy(Variables *b) = 0;

  /** compute the 1-norm of the variables */
  virtual double onenorm() = 0;

  /** compute the inf-norm of the variables */
  virtual double infnorm() = 0;

  virtual ~Variables() {};
};

#endif



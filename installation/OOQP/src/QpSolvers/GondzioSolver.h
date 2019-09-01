/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef GONALGORITHM_H
#define GONALGORITHM_H

#include "Solver.h"

class Data;
class Variables;
class ProblemFormulation;


/** 
 * Derived class of Solver implementing Gondzio-correction version of
 * Mehrotra's original predictor-corrector algorithm.
 * @ingroup QpSolvers
 */
class GondzioSolver : public Solver
{
protected:

  /** parameter in range [0,100] determines verbosity. (Higher value
   *  => more verbose.) */
  int        printlevel;

  /** exponent in Mehrotra's centering parameter, which is usually
   *  chosen to me (muaff/mu)^tsig, where muaff is the predicted
   *  complementarity gap obtained from an affine-scaling step, while
   *  mu is the current complementarity gap */
  double     tsig;

  /** maximum number of Gondzio corrector steps */
  int        maximum_correctors;

  /** actual number of Gondzio corrections needed */
  int        NumberGondzioCorrections;

  /** various parameters associated with Gondzio correction */
  double     StepFactor0, StepFactor1, AcceptTol, beta_min, beta_max;

  /**  storage for step vectors */
  Variables *corrector_step, *step;

  /** storage for residual vectors */
  Residuals *corrector_resid;

  ProblemFormulation * factory;

public:

  GondzioSolver( ProblemFormulation * of, Data * prob );

  virtual ~GondzioSolver();

  virtual int solve( Data *prob, Variables *iterate, Residuals * resid );

  /** reset parameters to their default values */
  virtual void reset_parameters() {};

  virtual void defaultMonitor( Data * data, Variables * vars,
							   Residuals * resids,
							   double alpha, double sigma,
							   int i, double mu, 
							   int status_code,
							   int level ) ;

};

#endif

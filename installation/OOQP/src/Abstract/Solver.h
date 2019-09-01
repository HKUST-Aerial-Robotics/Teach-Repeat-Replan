/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef ALGORITHM_H
#define ALGORITHM_H

class Data;
class Variables;
class Residuals;
class LinearSystem;
class Status;
class OoqpMonitor;
class OoqpStartStrategy;
class ProblemFormulation;

/**  * @defgroup QpSolvers
 *
 * Interior-point QP solvers
 * @{
 */

/** 
 * Abstract base class for QP solvers.
 */
class Solver
{
protected:
  OoqpMonitor * itsMonitors;
  Status *status;
  OoqpStartStrategy * startStrategy;

  /** norm of problem data */
  double dnorm;

 /** termination parameters */
  double     mutol, artol;

  /** parameters associated with the step length heuristic */
  double gamma_f, gamma_a;

  /** merit function, defined as the sum of the complementarity gap
      the residual norms, divided by (1+norm of problem data) */
  double phi;

  /** maximum number of  iterations allowed */
  int        maxit;

  /** history of values of mu obtained on all iterations to date */
  double *mu_history;

  /** history of values of residual norm obtained on all iterations to
      date */
  double *rnorm_history;
  
  /** history of values of phi obtained on all iterations to date */
  double *phi_history;

  /** the i-th entry of this array contains the minimum value of phi
   *  encountered by the algorithm on or before iteration i */
  double *phi_min_history;

public:

  /** iteration counter */
  int iter;

  Solver();
  virtual ~Solver();

  /** starting point heuristic */
  virtual void start( ProblemFormulation * formulation, 
		      Variables * iterate, Data * prob,
		      Residuals * resid, Variables * step);

  /** default starting point heuristic */
  virtual void defaultStart( ProblemFormulation * formulation,
			     Variables * iterate, Data * prob,
			     Residuals * resid, Variables * step);

  /** alternative starting point heuristic */
  virtual void stevestart ( ProblemFormulation * formulation,
			    Variables * iterate, Data * prob, 
			    Residuals * resid, Variables * step );

  /** alternative starting point heuristic: sets the "complementary"
   * variables to a large positive value (based on the norm of the
   * problem data) and the remaining variables to zero */
  virtual void dumbstart ( ProblemFormulation * formulation,
			   Variables * iterate, Data * prob, 
			   Residuals * resid, Variables * step );

  /** implements the interior-point method for solving the QP */
  virtual int solve(Data *prob, Variables *iterate, Residuals * resids) = 0;

  /** Mehrotra's heuristic to calculate the final step length */
  virtual double finalStepLength( Variables *iterate, Variables *step );

  /** perform monitor operation at each interior-point iteration */
  virtual void doMonitor( Data * data, Variables * vars,
						  Residuals * resids,
						  double alpha, double sigma,
						  int i, double mu,
						  int stop_code,
						  int level );
  /** default monitor: prints out one line of information on each
   * interior-point iteration */
  virtual void defaultMonitor( Data * data, Variables * vars,
							   Residuals * resids,
							   double alpha, double sigma,
							   int i, double mu, 
							   int stop_code,
							   int level ) = 0;

  /** this method called to test for convergence status at the end of
   * each interior-point iteration */
  virtual int doStatus( Data * data, Variables * vars, 
			Residuals * resids,
			int i, double mu, 
			int level );

  /** default method for checking status. May be replaced by a
   * user-defined method */
  virtual int defaultStatus( Data * data, Variables * vars, 
			     Residuals * resids,
			     int i, double mu, 
			     int level );

  /** method to add user-defined monitors to the monitor operations
      performed at each iteration */
  void addMonitor( OoqpMonitor * );

  /** method to replace the defaultStatus method with a user-defined
   *  status checking method */
  void useStatus( Status * s ) { status = s; }

  /** method to replace the defaultStatus method with a user-defined
   *  status checking method */
  void useStartStrategy( OoqpStartStrategy * s ) { startStrategy = s; }

  /** enables defaultMonitor as one of the monitors */
  void monitorSelf();
 
  void setMuTol( double m ) { mutol = m; }
  double getMuTol() { return mutol; }

  void setArTol( double ar ) { artol = ar; }
  double getArTol() { return artol; }

  double dataNorm() { return dnorm; }
  /** returns a pointed to the linear system object stored in this
   *  class */
  LinearSystem * getLinearSystem() { return sys; };
protected:
  LinearSystem *sys;
};

//@}

#endif


/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef OOQPMONITOR
#define OOQPMONITOR

class Solver;
class Data;
class Variables;
class Residuals;

/** Represents objects that display progress information for interior
 *  point QP solvers.
 *  @ingroup QpSolvers
 */
class OoqpMonitor {
public:
  OoqpMonitor * nextMonitor;

  OoqpMonitor() { nextMonitor = 0; };

  virtual void doIt( Solver * solver, Data * data, Variables * vars,
					 Residuals * resids,
					 double alpha, double sigma,
					 int i, double mu, 
                     int status_code,
					 int level ) = 0;
  virtual ~OoqpMonitor() {};
};  


/** Monitors that simply call the solver's defaultMonitor method.
 * 
 *  Don't create instances of this class. Call the solver's monitorSelf
 *  method instead.
 *
 *  @ingroup QpSolvers
 */
class OoqpSelfMonitor : public OoqpMonitor {
public:
  virtual void doIt( Solver * solver, Data * data, Variables * vars,
					 Residuals * resids,
					 double alpha, double sigma,
					 int i, double mu,
                     int status_code,
					 int level );
};

#include "OoqpMonitorData.h"

/**
 * Represents monitors that use a C function to print progress information
 * for an algorithm.
 * @ingroup QpSolvers
 */
class COoqpMonitor : public OoqpMonitor {
protected:
  DoItCFunc doItC;
  void * ctx;
public:
  COoqpMonitor( DoItCFunc doItC_, void * ctx_ );
  virtual void doIt( Solver * solver, Data * data, Variables * vars,
					 Residuals * resids,
					 double alpha, double sigma,
					 int i, double mu,
                     int status_code,
					 int level );
};


#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "OoqpMonitor.h"
#include "Solver.h"
#include <iostream>
using namespace std;
#include "Residuals.h"

void OoqpSelfMonitor::doIt( Solver * solver, Data * qpdata, Variables * vars,
							Residuals * resids,
							double alpha, double sigma,
							int i, double mu, 
                            int status_code,
							int level )
{
  solver->defaultMonitor( qpdata, vars, resids, alpha, sigma, i, mu, 
                          status_code, level );
}

COoqpMonitor::COoqpMonitor( DoItCFunc doItC_, void * ctx_ )
{
  doItC = doItC_;
  ctx   = ctx_;
}

void COoqpMonitor::doIt( Solver * solver, Data * qpdata, Variables * vars,
						 Residuals * resids,
						 double alpha, double sigma,
						 int i, double mu,
						 int status_code,
						 int level )
{
  OoqpMonitorData data;
  data.solver   = (void *) solver;
  data.data     = (void *) qpdata;
  data.vars     = (void *) vars;
  data.resids   = (void *) resids;
  data.i        = i;
  data.mu       = mu;
  data.rnorm    = resids->residualNorm();
  data.dataNorm = solver->dataNorm();
  data.gap      = resids->dualityGap();
  data.alpha    = alpha;
  data.sigma    = sigma;
  data.status_code  = status_code;
  data.level  = level;
  data.ctx    = ctx;

  doItC( &data );
}




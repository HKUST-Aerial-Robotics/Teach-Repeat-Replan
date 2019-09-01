/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "HuberData.h"
#include "HuberVars.h"
#include "HuberResiduals.h"
#include "HuberLinsys.h"
#include "GondzioSolver.h"
#include "Huber.h"
#include <cstring>
#include <iostream>
#include <cstdio>
#include <cstdlib>

#include "OoqpMonitorData.h"
#include "OoqpMonitor.h"

extern "C" int mexDoMonitor( OoqpMonitorData * data );

extern "C"
void ooqp_huber( double beta[],    int m, double t[], int n,
		 double lambda1[], double lambda2[],
		 double gamma1[],  double gamma2[], 
		 double Xt[],      double y[],      double cutoff,
		 int * status )
{
  HuberData     *prob    = 0;
  HuberVars     *vars    = 0;
  Huber         *factory = 0; 
  GondzioSolver * s      = 0;
  Residuals * resid      = 0;

  *status = 0;
  OoqpMonitorData monitorCtx;
  try {
    factory = new Huber();
    prob    = (HuberData *) factory->makeData( m, n, cutoff, Xt, y );
    vars    = (HuberVars *) factory->makeVariables( prob, beta, t,
						    lambda1, lambda2,
						    gamma1, gamma2 );
    s       = new GondzioSolver( factory, prob );
    resid = factory->makeResiduals( prob );
    
    COoqpMonitor * mon = new COoqpMonitor( mexDoMonitor, &monitorCtx );
    s->addMonitor( mon );
    s->solve(prob,vars, resid);
  } 
  catch( ... ) {
    *status = 1;
  }

  delete vars;
  delete resid;
  delete s;
  delete prob;
  delete factory;
}







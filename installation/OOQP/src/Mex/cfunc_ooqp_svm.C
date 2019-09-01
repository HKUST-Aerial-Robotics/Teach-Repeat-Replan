/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

// #include "OoqpVector.h"
// #include "DoubleMatrix.h"
#include "DoubleMatrix.h"
#include "SimpleVector.h"
#include "SvmData.h"
#include "SvmVars.h"
#include "SvmResiduals.h"
#include "SvmLinsys.h"
#include "GondzioSolver.h"
#include "Svm.h"
#include <cstring>
#include <iostream>
#include <cstdio> 
#include <cstdlib>
#include "OoqpMonitorData.h"
#include "OoqpMonitor.h"

extern "C" int mexDoMonitor( OoqpMonitorData * data );

extern "C"
void ooqp_svm( double w[], int n, double * beta, double v[], int m,
	       double z[], double u[], double s[], double Xt[],
	       double d[], double rho, int * status )
{
  SvmData    * prob = 0;
  SvmVars    * vars = 0;
  Svm        * factory = 0;
  GondzioSolver * solver = 0;
  Residuals * resid = 0;

  OoqpMonitorData monitorCtx;
  try {
    int hyperplanedim = n, nobservations = m;

    // input format "(executable name) filename". Get the data from
    // the named file and solve a real problem.
    
    // problem dimensions hyperplanedim, nobservations must be read
    // from the input file, which appears in argv[1].
    
    factory = new Svm();
    prob = (SvmData *) factory->makeData( hyperplanedim, nobservations, Xt, d,
					  rho );
    
    
    solver     = new GondzioSolver( factory, prob );
    vars  = (SvmVars *) factory->makeVariables( prob, w, v, z, u, s );
    resid = factory->makeResiduals( prob );

    COoqpMonitor * mon = new COoqpMonitor( mexDoMonitor, &monitorCtx );
    solver->addMonitor( mon );
    solver->solve(prob, vars, resid);
    
    *beta = vars->beta;

    *status = 0;
  } catch (...) {
    *status = 1;
  }
  delete vars;  
  delete resid;
  delete solver;
  delete prob;
  delete factory;
}



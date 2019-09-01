/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cQpGen.h"
#include "QpGenData.h"
#include "QpGen.h"
#include "Solver.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "SimpleVector.h"
#include "OoqpMonitor.h"

#include <cstdlib>

extern "C" 
void QpGenFinish ( QpGenContext * ctx,
		   double   x[],  double gamma[],     double phi[],
		   double   y[], 
		   double   z[],  double lambda[],    double pi[],
		   double * objectiveValue, int * status_code )
{
  QpGenData    * prob    = (QpGenData *)    ctx->prob;
  QpGen * factory = (QpGen *) ctx->factory;
  Solver       * solver  = (Solver *)       ctx->solver;
  
  QpGenVars      * vars    = 0;
  QpGenResiduals * resid   = 0;
  
  try {
    vars  = (QpGenVars * )     factory->makeVariables( prob );
    resid = (QpGenResiduals *) factory->makeResiduals( prob );
    
    *status_code = solver->solve(prob,vars,resid);

    vars->x     ->copyIntoArray( x );
    vars->gamma ->copyIntoArray( gamma );
    vars->phi ->copyIntoArray( phi );

    vars->y->copyIntoArray( y );

    vars->z     ->copyIntoArray( z );
    vars->lambda->copyIntoArray( lambda );
    vars->pi    ->copyIntoArray( pi );
    *objectiveValue = prob->objectiveValue( vars );
  }
  catch( ... ) {
    *status_code = -1;
  }
  delete vars;
  delete resid;
}

extern "C"
void QpGenCleanup( QpGenContext * ctx )
{
  QpGen * factory = (QpGen *) ctx->factory;
  QpGenData    * prob    = (QpGenData *)    ctx->prob;
  Solver       * solver  = (Solver *)       ctx->solver;
  
  delete factory;       delete prob;       delete solver;
  ctx->factory = 0;     ctx->prob = 0;     ctx->solver = 0;
}

extern "C"
void QpGenAddMonitor( QpGenContext * ctx, DoItCFunc cmon,
		      void * mctx )
{
  COoqpMonitor * mon = new COoqpMonitor( cmon, mctx );
  ((Solver *) ctx->solver)->addMonitor( mon );
}

extern "C"
void QpGenMonitorSelf( QpGenContext * ctx )
{
  ((Solver *) ctx->solver)->monitorSelf();
}


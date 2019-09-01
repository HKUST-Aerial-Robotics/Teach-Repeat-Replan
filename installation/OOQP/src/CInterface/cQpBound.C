/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cQpBound.h"
#include "QpBoundData.h"
#include "QpBound.h"
#include "QpBoundVars.h"
#include "QpBoundResiduals.h"
#include "Solver.h"
#include "OoqpMonitor.h"

#include "SimpleVector.h"

extern "C"
void QpBoundDoSolve( double x[], double gamma[], double phi[],
		     QpBoundContext * ctx, int * ierr )
{
  QpBound * factory = (QpBound *) ctx->factory;
  QpBoundData    * prob    = (QpBoundData *)    ctx->prob;
  Solver         * solver  = (Solver *)         ctx->solver;

  QpBoundVars      * vars  = 0;
  QpBoundResiduals * resid = 0;

  try {
    vars  = (QpBoundVars * )     factory->makeVariables( prob );
    resid = (QpBoundResiduals *) factory->makeResiduals( prob );

    solver->solve(prob,vars,resid);

    vars->x   ->copyIntoArray( x );
    vars->tau ->copyIntoArray( gamma );
    vars->nu  ->copyIntoArray( phi );

    *ierr = 0;

  } catch ( ... ) {
    *ierr = 1;
  }
  delete vars;
  delete resid;
}

extern "C"
void QpBoundCleanup( QpBoundContext * ctx )
{
  QpBound * factory = (QpBound *) ctx->factory;
  QpBoundData    * prob    = (QpBoundData *)    ctx->prob;
  Solver         * solver  = (Solver *)         ctx->solver;
  
  delete factory;       delete prob;       delete solver;
  ctx->factory = 0;     ctx->prob = 0;     ctx->solver = 0;
}

extern "C"
void QpBoundAddMonitor( QpBoundContext * ctx, DoItCFunc cmon,
			void * mctx )
{
  COoqpMonitor * mon = new COoqpMonitor( cmon, mctx );
  ((Solver *) ctx->solver)->addMonitor( mon );
}

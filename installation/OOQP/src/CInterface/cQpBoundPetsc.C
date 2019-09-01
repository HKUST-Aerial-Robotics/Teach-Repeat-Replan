/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cQpBoundPetsc.h"
#include "QpBoundPetsc.h"
#include "QpBoundData.h"
#include "QpBoundVars.h"
#include "QpBoundResiduals.h"
#include "MehrotraSolver.h"
#include "PetscVector.h"
#include "OoqpMonitor.h"
#include "Status.h"
#include "cQpBound.h"

#define ABORT_ON_ERROR(n) CHKERRABORT(PETSC_COMM_WORLD, n) 

extern "C"
void PetscQpBoundSolve( double alpha, Vec c,     Mat Q,      int nnzQ,
			Vec xlow,  Vec ixlow,  Vec xupp,  Vec ixupp,
			Vec x, 
			int * ierr )
{
  QpBoundPetscContext ctx;
  PetscQpBoundSetup(  alpha, c, Q, nnzQ,
		      xlow, ixlow, xupp, ixupp,
		      &ctx, ierr );
  if( *ierr == 0 ) {
    Vec t, v, tau, nu;
    
    *ierr = VecDuplicate( x, &t );   ABORT_ON_ERROR( *ierr );
    *ierr = VecDuplicate( x, &v );   ABORT_ON_ERROR( *ierr );
    *ierr = VecDuplicate( x, &tau ); ABORT_ON_ERROR( *ierr );
    *ierr = VecDuplicate( x, &nu );  ABORT_ON_ERROR( *ierr );

    PetscQpBoundFinish( &ctx, x, t, v, tau, nu, ierr );
    
    *ierr = VecDestroy( t );   ABORT_ON_ERROR( *ierr );
    *ierr = VecDestroy( v );   ABORT_ON_ERROR( *ierr );
    *ierr = VecDestroy( tau ); ABORT_ON_ERROR( *ierr );
    *ierr = VecDestroy( nu );  ABORT_ON_ERROR( *ierr );
  }
  PetscQpBoundCleanUp( &ctx );
}
  
extern "C"
void PetscQpBoundSetup( double alpha, Vec c,    Mat Q,     int nnzQ,
			Vec xlow, Vec ixlow, Vec xupp, Vec ixupp,
			QpBoundPetscContext *context,
			int * ierr )
{
  int m, nx;
  *ierr = VecGetSize( c, &nx ); ABORT_ON_ERROR( *ierr );
  *ierr = VecGetLocalSize( c, &m ); ABORT_ON_ERROR( *ierr );

  context->solver   = 0;
  context->prob     = 0;
  context->qp       = 0;

  try {
    QpBoundPetsc * qp = new QpBoundPetsc( m, nx );
    context->qp              = qp;
    QpBoundData * prob       =
      (QpBoundData *) qp->makeData( alpha, c, Q, xlow, ixlow, xupp, ixupp );
    context->prob            = prob;
    MehrotraSolver * solver  = new MehrotraSolver( qp, prob );
    context->solver          = solver;
   *ierr = 0;
  }  
  catch( ... ) {
    PetscQpBoundCleanUp( context );
    *ierr = 1;
  }
}

extern "C"
void PetscQpBoundFinish( QpBoundPetscContext * ctx,
			 Vec x, Vec t, Vec v, Vec tau, Vec nu,
			 int * ierr )
{
  MehrotraSolver       * s    = (MehrotraSolver *)      ctx->solver;
  QpBoundData          * p    = (QpBoundData *)         ctx->prob;
  QpBoundPetsc  * qp   = (QpBoundPetsc *) ctx->qp;

  QpBoundVars          * vars = 0;
  QpBoundResiduals     * resid = 0;

  try {
    vars = (QpBoundVars * ) qp->makeVariables( p, x, t, v, tau, nu );
    Vec rx;
    *ierr = VecDuplicate( x, &rx ); ABORT_ON_ERROR( *ierr );
    resid = (QpBoundResiduals *) qp->makeResiduals( p, rx );
    s->solve(p, vars, resid);
    *ierr = VecDestroy( rx ); ABORT_ON_ERROR( *ierr );

    *ierr = 0;
  }
  catch( ... ) { *ierr = 1; }

  delete vars;
  delete resid;
}

extern "C"
void PetscQpBoundCleanUp( QpBoundPetscContext * ctx )
{
  MehrotraSolver       * s    = (MehrotraSolver *)      ctx->solver;
  QpBoundData          * p    = (QpBoundData *)         ctx->prob;
  QpBoundPetsc  * qp   = (QpBoundPetsc *) ctx->qp;

  delete s;         delete p;        delete qp;
  ctx->solver = 0;  ctx->prob = 0;   ctx->qp  = 0;
}

extern "C"
void PetscQpBoundAddMonitor( QpBoundPetscContext * ctx, DoItCFunc cmon,
			     void * mctx )
{
  COoqpMonitor * mon = new COoqpMonitor( cmon, mctx );
  ((MehrotraSolver *) ctx->solver)->addMonitor( mon );
}

extern "C"
void PetscQpBoundUseStatus( QpBoundPetscContext * ctx, int (*cstat)(void *),
			     void * mctx )
{
  CStatus * stat = new CStatus( cstat, mctx );
  ((MehrotraSolver *) ctx->solver)->useStatus( stat );
}

extern "C"
void PetscQpBoundMonitorSelf( QpBoundPetscContext * ctx )
{
  ((MehrotraSolver *) ctx->solver)->monitorSelf();
}


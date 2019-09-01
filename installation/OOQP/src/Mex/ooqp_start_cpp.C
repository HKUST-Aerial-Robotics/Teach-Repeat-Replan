#include "cQpGen.h"
#include "QpGenData.h"
#include "QpGen.h"
#include "Solver.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "SimpleVector.h"
#include "OoqpMonitor.h"
#include "OoqpStartStrategy.h"

class NilStartStrategy : public OoqpStartStrategy {
public:
  virtual void doIt( Solver * /* solver */,
		     ProblemFormulation * /* formulation */,
		     Variables * /* iterate */, Data * /* prob */,
		     Residuals * /* resid */, Variables * /* step */ ) 
  { /* iterate->print(); */ }
};

extern "C" 
void QpGenFinishWithSp ( double   x[],  
			 double   v[], double gamma[],
			 double   w[], double phi[],
			 double   y[], 
			 double   z[],  
			 double   s[],
			 double   t[], double lambda[],
			 double   u[], double pi[],
			 double mutol, double artol,
			 QpGenContext * ctx, int * status_code )
{
  QpGenData    * prob    = (QpGenData *)    ctx->prob;
  QpGen * factory = (QpGen *) ctx->factory;
  Solver       * solver  = (Solver *)       ctx->solver;
  
  QpGenVars      * vars    = 0;
  QpGenResiduals * resid   = 0;
  
  try {
    vars  = (QpGenVars * )     factory->makeVariables( prob );
    resid = (QpGenResiduals *) factory->makeResiduals( prob );

    vars->x->copyFromArray( x );

    vars->w->copyFromArray( w );
    vars->gamma->copyFromArray( gamma );
  
    vars->v->copyFromArray( v );
    vars->phi->copyFromArray( phi );
  
    vars->y->copyFromArray( y );
    vars->z->copyFromArray( z );
    vars->s->copyFromArray( s );

    vars->t->copyFromArray( t );
    vars->lambda->copyFromArray( lambda );

    vars->u->copyFromArray( u );
    vars->pi->copyFromArray( pi );
    
    if( vars->isInteriorPoint() ) {
      
      NilStartStrategy * nss = new NilStartStrategy;
      
      solver->useStartStrategy( nss );
      solver->setMuTol( mutol );
      solver->setArTol( artol );
      *status_code = solver->solve(prob,vars,resid);
      
      vars->x     ->copyIntoArray( x );
      vars->gamma ->copyIntoArray( gamma );
      vars->phi ->copyIntoArray( phi );
      
      vars->y->copyIntoArray( y );
      
      vars->z     ->copyIntoArray( z );
      vars->lambda->copyIntoArray( lambda );
      vars->pi    ->copyIntoArray( pi );
    } else {
      *status_code = 1024;
    }  
  }
  catch( ... ) {
    *status_code = -1;
  }

  delete vars;
  delete resid;
}

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundLinsys.h"
#include "QpBoundData.h"
#include "QpBoundVars.h"
#include "QpBoundResiduals.h"

#include "DoubleMatrix.h"
#include "DoubleLinearSolver.h"
#include "OoqpVector.h"
#include "QpBound.h"

QpBoundLinsys::QpBoundLinsys( QpBound * f,
                                  DoubleMatrix * Mat_,
                                  DoubleLinearSolver * solver_,
                                  OoqpVector * dq_, int nx_,
                                  OoqpVector * index_lower_,
                                  OoqpVector * index_upper_ )
{
  nx          = nx_;
  factory     = f;
  SpReferTo( Mat, Mat_ );
  SpReferTo( index_lower, index_lower_);
  SpReferTo( index_upper, index_upper_);
  // now the problem dimension is passed explicitly, no need for the
  // following call
  // Mat->getSize( size, dummy );
  solver = solver_;
  SpReferTo( dq, dq_ );
  dd = OoqpVectorHandle( factory->newPrimalVector() );
}

void QpBoundLinsys::factor(Data * /* prob_in */, Variables *vars_in)
{
  QpBoundVars *vars = (QpBoundVars *) vars_in;
    
  // add logic here: work through the indices 0..n-1, filling out the
  // appropriate value of each dd[i] (the elements to be added to the
  // diagonal)
  dd->copyFrom( *dq );
  dd->axdzpy( 1.0, *vars->tau, *vars->t, *index_lower );
  dd->axdzpy( 1.0, *vars->nu,  *vars->v, *index_upper );

  Mat->atPutDiagonal( 0, *dd );
  // perform the factorization using LAPACK
  solver->diagonalChanged( 0, nx );
}

void QpBoundLinsys::solve(Data * /* prob_in */, Variables *vars_in, 
			    Residuals *rhs_in, Variables *step_in)
{
  QpBoundVars *vars = (QpBoundVars *) vars_in;
  QpBoundVars *step = (QpBoundVars *) step_in;
  QpBoundResiduals *rhs = (QpBoundResiduals *) rhs_in;

  // make up the rhs:

  {
    OoqpVector & z    = *step->x; // Use step->x as the rhs
    z.copyFrom( *rhs->rc );

    // z += ( rhs->rttau + vars->tau .* rhs->rl) ./ vars->t
    if( vars->nlower > 0 ) {
      OoqpVector & temp = *step->t; // Use step->t as temporary space
      temp.copyFrom( *rhs->rttau );
      temp.axzpy ( 1.0, *vars->tau, *rhs->rl );
      z   .axdzpy( 1.0, temp, *vars->t, *index_lower );
    }
    if( vars->nupper > 0 ) {
      // z -= ( rhs->rvnu + vars->nu .* rhs->ru ) ./ vars->v
      OoqpVector & temp = *step->v; // Use step->v as temporary space
      temp.copyFrom( *rhs->rvnu );
      temp. axzpy(  1.0, *vars->nu, *rhs->ru );
      z   .axdzpy( -1.0, temp, *vars->v, *index_upper );
    }

    // solve the compressed system
    solver->solve( z );
  }
  // The solution is in step->x

  // recover step->tau and step->nu
  // step->tau = 
  // ( vars->tau .* (rhs->rl - step->x) + rhs->rttau ) ./ vars->t
  if( vars->nlower > 0 ) {
    {
      OoqpVector & temp = *step->t; // Use step->t as temp space
      
      temp.copyFrom( *rhs->rl );
      temp.axpy( -1.0, *step->x );
      
      step->tau->copyFrom( *rhs->rttau );
      step->tau->axzpy( 1.0, *vars->tau, temp );
      step->tau->divideSome( *vars->t, *index_lower );
    }

    step->t->copyFrom( *rhs->rttau );
    step->t->axzpy( -1.0, *vars->t, *step->tau );
    step->t->divideSome( *vars->tau, *index_lower );
    // step->nu =
    // (vars->nu .* (rhs->ru + step->x) + rhs->rvnu ) ./ vars->v
  }

  if( vars->nupper > 0 ) {
    {
      OoqpVector & temp = *step->v; // Use step->v as temp space
      
      temp.copyFrom( *rhs->ru );
      temp.axpy( 1.0, *step->x );
      
      step->nu->copyFrom( *rhs->rvnu );
      step->nu->axzpy( 1.0, *vars->nu, temp );
      step->nu->divideSome( *vars->v, *index_upper );
    }

    step->v->copyFrom( *rhs->rvnu );
    step->v->axzpy( -1.0, *vars->v, *step->nu );
    step->v->divideSome( *vars->nu, *index_upper );
  }
}

QpBoundLinsys::~QpBoundLinsys()
{
  delete solver;
}


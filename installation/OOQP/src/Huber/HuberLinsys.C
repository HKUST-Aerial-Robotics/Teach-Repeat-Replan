/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "HuberLinsys.h"
#include <cmath>
#include "DenseSymMatrix.h"
#include "DenseGenMatrix.h"
#include "HuberData.h"
#include "HuberVars.h"
#include "HuberResiduals.h"
#include "DoubleLinearSolver.h"
#include "DeSymPSDSolver.h"
#include "SimpleVector.h"

HuberLinsys::HuberLinsys(Data *prob_in)
{
  HuberData *prob = (HuberData *)prob_in;
  // allocates a square symmetric matrix of dimension npredictors,
  // formation and factorization of which is the key operation in
  // solving the linear system at each interior-point iteration.
  L = DenseSymMatrixHandle( new DenseSymMatrix(prob->npredictors) );
  solver = new DeSymPSDSolver( L );

  // allocates the messy diagonal matrix that arises in the block
  // elimination
  mDinv      = SimpleVectorHandle( new SimpleVector( prob->nobservations ) );

  // allocates the temp vectors that arise in formation of the linear
  // system.
  mDinvColX  = SimpleVectorHandle( new SimpleVector( prob->nobservations ) );
  mColOfL    = SimpleVectorHandle( new SimpleVector( prob->npredictors ) );
}

void HuberLinsys::factor(Data *prob_in, Variables *vars_in)
{
  // does a Cholesky factorization of the matrix (X^T D^{-1} X)

  HuberData *prob = (HuberData *) prob_in;
  HuberVars *vars = (HuberVars *) vars_in;
  int k;

  int npredictors = prob->npredictors;

  // compute and store Dinv, using vars
  mDinv->setToConstant( 1.0 );
  mDinv->axdzpy( 1.0, *vars->lambda1, *vars->gamma1 );
  mDinv->axdzpy( 1.0, *vars->lambda2, *vars->gamma2 );
  mDinv->invert();

  // now form L, a column at a time
  SimpleVector & colOfL   = *mColOfL;
  SimpleVector & DinvColX = *mDinvColX;
  L->symAtPutZeros(0, 0, npredictors, npredictors );
  for ( k = 0; k < npredictors; k++ ) {
    // grab column k of X (i.e. row k of Xt)
    prob->Xt->getRow(k, DinvColX);
    // scale it with Dinv
    mDinvColX->componentMult( *mDinv );
    // multiply by Xt
    prob->XtMult ( 0.0, colOfL, 1.0, DinvColX);
    // put into lower triangle of L
    L->symAtPutDense (k, k, &colOfL[k], 1, npredictors-k, 1);
  }
    
  // now compute the Cholesky factor, overwriting L
  solver->matrixChanged();

  return;
}

void HuberLinsys::solve(Data *prob_in, Variables *vars_in, 
			Residuals *rhs_in, Variables *step_in)
{
  
  HuberData *prob = (HuberData *) prob_in;
  HuberResiduals *rhs = (HuberResiduals *) rhs_in;
  HuberVars *vars = (HuberVars *) vars_in;
  HuberVars *step = (HuberVars *) step_in;

  int i;
  int nobservations = prob->nobservations;

  double * lambda1 = vars->lambda1->elements();
  double * lambda2 = vars->lambda2->elements();
  double * gamma1  = vars->gamma1->elements();
  double * gamma2  = vars->gamma2->elements();
  double * Yresid  = rhs ->Yresid->elements();
  double * gam1Resid = rhs->gam1->elements();
  double * lamgam1   = rhs->lamgam1->elements();
  double * gam2Resid = rhs->gam2->elements();
  double * lamgam2   = rhs->lamgam2->elements();

  double * lambda1Step = step->lambda1->elements();
  double * lambda2Step = step->lambda2->elements();
  double * gamma1Step  = step->gamma1->elements();
  double * gamma2Step  = step->gamma2->elements();
  double * tStep       = step->t->elements();


  SimpleVector & Dinv    = *mDinv;
  { 
    // Grab some scratch space from the step
    SimpleVector & rc = *step->gamma1;
    // fill in rc
    for(i=0; i<nobservations; i++) {
      rc[i] = Yresid[i];
      rc[i] -= (lambda1[i] / gamma1[i]) * 
	(gam1Resid[i] + lamgam1[i] / lambda1[i]);
      rc[i] += (lambda2[i] / gamma2[i]) * 
	(gam2Resid[i] + lamgam2[i] / lambda2[i]);
    }
    rc.componentMult( Dinv );
    
    // Form the rhs in step->beta
    step->beta->copyFrom( *rhs->Xtimest );
    prob->XtMult( 1.0, *step->beta, 1.0, rc );
    
    // solve for step->beta
    solver->solve( *step->beta );
 
    // substitute for (D t)
    prob->XtTransMult( 0.0, *step->t, 1.0, *step->beta );
    step->t->componentMult( Dinv );
    step->t->axpy( -1.0, rc ); 
  } // and let the scratch vector rc go out of scope.

  // substitute for (D lambda1) and (D lambda2)
  for(i=0; i < nobservations; i++) {
    lambda1Step[i] = 
      gam1Resid[i] + lamgam1[i] / lambda1[i] - tStep[i];
    lambda1Step[i] *= lambda1[i] / gamma1[i];
    lambda2Step[i] =
      gam2Resid[i] + lamgam2[i] / lambda2[i] + tStep[i];
    lambda2Step[i] *= lambda2[i] / gamma2[i];
  }

  // substitute for (D gamma1) and (D gamma2)
  for(i=0; i < nobservations; i++) {
    gamma1Step[i] = ( lamgam1[i] - gamma1[i] * lambda1Step[i] )
      / lambda1[i];
    gamma2Step[i] = ( lamgam2[i] - gamma2[i] * lambda2Step[i] )
      / lambda2[i];
  }
}


// define the destructor
HuberLinsys::~HuberLinsys()
{
  delete solver;
}





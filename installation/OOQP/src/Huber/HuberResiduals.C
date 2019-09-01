/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "HuberResiduals.h"
#include "HuberData.h"
#include "HuberVars.h"
#include "SimpleVector.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;


HuberResiduals::HuberResiduals(int nobservations_in, int npredictors_in)
{
  nobservations = nobservations_in;
  npredictors = npredictors_in;

  Xtimest  = SimpleVectorHandle( new SimpleVector( npredictors    ) );
  Yresid   = SimpleVectorHandle( new SimpleVector( nobservations ) );
  gam1     = SimpleVectorHandle( new SimpleVector( nobservations ) );
  gam2     = SimpleVectorHandle( new SimpleVector( nobservations ) );
  lamgam1  = SimpleVectorHandle( new SimpleVector( nobservations ) );
  lamgam2  = SimpleVectorHandle( new SimpleVector( nobservations ) );
}

HuberResiduals::~HuberResiduals()
{
}


// calculate the r1 and r2 components of the residual vector for the
// Huber case

void HuberResiduals::calcresids(Data *prob_in, Variables *vars_in )
{
  double norm, gap;
  HuberData *prob = (HuberData *) prob_in;
  HuberVars *vars = (HuberVars *) vars_in;

  prob->XtMult( 0.0, *Xtimest, 1.0, *vars->t );

  Yresid->setToZero();
  Yresid->axpy(-1.0, *prob->Y);
  Yresid->axpy(-1.0, *vars->t);
  prob->XtTransMult( 1.0, *Yresid, 1.0, *vars->beta );

  gap = - Yresid->dotProductWith( *vars->t );

  Yresid->axpy( 1.0, *vars->lambda1);
  Yresid->axpy(-1.0, *vars->lambda2);

  gam1->setToConstant(prob->cutoff);
  gap += gam1->dotProductWith( *vars->lambda1 );
  gap += gam1->dotProductWith( *vars->lambda2 );

  gam1->axpy( 1.0, *vars->t);
  gam1->axpy(-1.0, *vars->gamma1);
  gam2->setToConstant(prob->cutoff);
  gam2->axpy(-1.0, *vars->t);
  gam2->axpy(-1.0, *vars->gamma2);

  // calculate the norm of the (r1,r2) residual component
  double tempXtimest, tempYresid, tempgam1, tempgam2;
  tempXtimest = Xtimest->infnorm();
  tempYresid = Yresid->infnorm();
  tempgam1 = gam1->infnorm();
  tempgam2 = gam2->infnorm();
  norm = tempXtimest;
  if(norm < tempYresid) norm = tempYresid;
  if(norm < tempgam1)   norm = tempgam1;
  if(norm < tempgam2)   norm = tempgam2;

  mResidualNorm = norm;
  mDualityGap = gap;
}


void HuberResiduals::add_r3_xz_alpha( Variables *vars_in, double value )
{
  HuberVars *vars = (HuberVars *) vars_in;

  lamgam1->axzpy( 1.0, *vars->gamma1, *vars->lambda1 );
  lamgam2->axzpy( 1.0, *vars->gamma2, *vars->lambda2 );
  lamgam1->addConstant(value);
  lamgam2->addConstant(value);
}

void HuberResiduals::set_r3_xz_alpha( Variables *vars_in, double value )
{
  lamgam1->setToZero();
  lamgam2->setToZero();

  this->add_r3_xz_alpha(vars_in, value );
}

void HuberResiduals::clear_r1r2()
{
  Yresid->setToZero();
  gam1->setToZero();
  gam2->setToZero();
  Xtimest->setToZero();

}


void HuberResiduals::clear_r3()
{
  lamgam1->setToZero();
  lamgam2->setToZero();
}


void HuberResiduals::project_r3(double rmin, double rmax)
{
  lamgam1->gondzioProjection(rmin, rmax);
  lamgam2->gondzioProjection(rmin, rmax);
}


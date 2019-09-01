/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundResiduals.h"
#include "QpBoundData.h"
#include "QpBoundVars.h"
#include "QpBound.h"
#include "OoqpVector.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;

extern "C" void daxpy_ ( int * n, double * da,
			 double * x, int * incx, 
			 double * y, int * incy );

QpBoundResiduals::QpBoundResiduals( QpBound * f,
				    QpBoundData * prob, int nx_in)
{
  factory = f;

  // define the dimension variables in the parent class appropriately
  n  = nx; m = 0;

  nx    = nx_in;
  rc    = OoqpVectorHandle( factory->newPrimalVector() );
  rl    = OoqpVectorHandle( factory->newPrimalVector() );
  ru    = OoqpVectorHandle( factory->newPrimalVector() );
  rttau = OoqpVectorHandle( factory->newPrimalVector() );
  rvnu  = OoqpVectorHandle( factory->newPrimalVector() );

  index_lower = OoqpVectorHandle( prob->getIndexLower() );
  index_upper = OoqpVectorHandle( prob->getIndexUpper() );
}

QpBoundResiduals::QpBoundResiduals( QpBound * f, OoqpVector * rc_,
				    QpBoundData * prob, int nx_in)
{
  factory = f;

  // define the dimension variables in the parent class appropriately
  n  = nx; m = 0;

  nx    = nx_in;
  rc    = OoqpVectorHandle( rc_ );
  rl    = OoqpVectorHandle( factory->newPrimalVector() );
  ru    = OoqpVectorHandle( factory->newPrimalVector() );
  rttau = OoqpVectorHandle( factory->newPrimalVector() );
  rvnu  = OoqpVectorHandle( factory->newPrimalVector() );

  index_lower = OoqpVectorHandle( prob->getIndexLower() );
  index_upper = OoqpVectorHandle( prob->getIndexUpper() );
}

QpBoundResiduals::~QpBoundResiduals()
{
}

void QpBoundResiduals::calcresids(Data *prob_in, 
				  Variables *vars_in )
{
  double norm, gap;
  QpBoundData *prob = (QpBoundData *) prob_in;
  QpBoundVars *vars = (QpBoundVars *) vars_in;

  prob->getg( *rc );
  prob->Qmult( 1.0, *rc, 1.0, *vars->x );
  // the (g + Q x)' * x contribution to the duality gap
  gap = rc->dotProductWith( *vars->x );

  assert( vars->tau->matchesNonZeroPattern( *index_lower ) &&
	  vars->nu ->matchesNonZeroPattern( *index_upper ) );
  
  rc->axpy( -1.0, *vars->tau );
  rc->axpy(  1.0, *vars->nu  );
  
  prob->getl( *rl );
  rl->negate();
  gap += rl->dotProductWith( *vars->tau );

  rl->axpy(  1.0, *vars->x );
  rl->axpy( -1.0, *vars->t );
  rl->selectNonZeros( *index_lower );
  
  prob->getu( *ru );
  gap += ru->dotProductWith( *vars->nu );
  
  ru->axpy( -1.0, *vars->x );
  ru->axpy( -1.0, *vars->v );
  ru->selectNonZeros( *index_upper );

  // Now calculate the norm
  double temp;

  assert( rl->matchesNonZeroPattern( *index_lower ) );
  assert( ru->matchesNonZeroPattern( *index_upper ) );

  norm = rc->infnorm();
  temp = rl->infnorm();
  if( temp > norm ) norm = temp;
  temp = ru->infnorm();
  if( temp > norm ) norm = temp;

  mResidualNorm = norm;
  mDualityGap   = gap;
}

// add a constant to the r3 component of the residuals

void QpBoundResiduals::add_r3_xz_alpha(Variables *vars_in, double value)
{
  QpBoundVars *vars = (QpBoundVars *) vars_in;

  assert( vars->t  ->matchesNonZeroPattern( *index_lower ) &&
	  vars->tau->matchesNonZeroPattern( *index_lower ) );
  assert( vars->v  ->matchesNonZeroPattern( *index_upper ) &&
	  vars->nu ->matchesNonZeroPattern( *index_upper ) );

  rttau->axzpy( 1.0, *vars->t, *vars->tau );
  rvnu ->axzpy( 1.0, *vars->v, *vars->nu  );

  if( value != 0.0 ) {
    rttau->addSomeConstants( value, *index_lower );
    rvnu ->addSomeConstants( value, *index_upper );
  }
}

// add pairwise products to the r3 component

void QpBoundResiduals::set_r3_xz_alpha(Variables *vars_in, double value)
{
  rttau->setToZero();
  rvnu ->setToZero();
  this->add_r3_xz_alpha( vars_in, value );
}

// clear even the components for which bounds are not defined, to be
// thorough!!

void QpBoundResiduals::clear_r1r2()
{
  rc->setToZero();
  rl->setToZero();
  ru->setToZero();
}
  
// clear even the components for which bounds are not defined, to be
// thorough!!

void QpBoundResiduals::clear_r3()
{
  rttau->setToZero();
  rvnu ->setToZero();
}

void QpBoundResiduals::project_r3(double rmin, double rmax)
{
  rttau->gondzioProjection( rmin, rmax );
  rttau->selectNonZeros( *index_lower );
  rvnu ->gondzioProjection( rmin, rmax );
  rvnu ->selectNonZeros( *index_upper );
}

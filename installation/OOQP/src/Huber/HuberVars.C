/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "HuberVars.h"
#include "HuberData.h"
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;

#include <cmath>
#include "SimpleVector.h"

HuberVars::HuberVars(int nobservations_in,int npredictors_in)
{
  nobservations = nobservations_in;
  npredictors = npredictors_in;

  beta     = SimpleVectorHandle( new SimpleVector( npredictors ) );
  t        = SimpleVectorHandle( new SimpleVector( nobservations ) );

  gamma1   = SimpleVectorHandle( new SimpleVector( nobservations ) );
  gamma2   = SimpleVectorHandle( new SimpleVector( nobservations ) );

  lambda1  = SimpleVectorHandle( new SimpleVector( nobservations ) );
  lambda2  = SimpleVectorHandle( new SimpleVector( nobservations ) );

  nComplementaryVariables = 2*nobservations;
}

HuberVars::HuberVars(int nobservations_in,int npredictors_in,
		     double * dbeta, double * dt,
		     double * dlambda1, double * dlambda2,
		     double * dgamma1,  double * dgamma2 )
{
  nobservations = nobservations_in;
  npredictors = npredictors_in;

  beta     = SimpleVectorHandle( new SimpleVector( dbeta, npredictors ) );
  t        = SimpleVectorHandle( new SimpleVector( dt, nobservations ) );

  gamma1   = SimpleVectorHandle( new SimpleVector( dgamma1, nobservations ) );
  gamma2   = SimpleVectorHandle( new SimpleVector( dgamma2, nobservations ) );

  lambda1  = SimpleVectorHandle( new SimpleVector( dlambda1, nobservations ) );
  lambda2  = SimpleVectorHandle( new SimpleVector( dlambda2, nobservations ) );

  nComplementaryVariables = 2*nobservations;
}

HuberVars::~HuberVars()
{
}


double HuberVars::mu()
{
  double temp =
    gamma1->dotProductWith(*lambda1) + gamma2->dotProductWith(*lambda2);

  return ( nComplementaryVariables > 0 ) ? 
    temp / nComplementaryVariables : 0.0;
}


double HuberVars::mustep(Variables *step_in, double alpha)
{
  HuberVars * step = (HuberVars *) step_in;
 
  double temp =
    gamma1->shiftedDotProductWith( alpha, *step->gamma1,
				   *lambda1,
				   alpha, *step->lambda1 ) +
    gamma2->shiftedDotProductWith( alpha, *step->gamma2,
				   *lambda2,
				   alpha, *step->lambda2 );
  
  return ( nComplementaryVariables > 0 ) ? 
    temp / nComplementaryVariables : 0.0;
}


void HuberVars::saxpy( Variables *b_in, double alpha )
{
  HuberVars *b = (HuberVars *) b_in;

  // check dimensions
  assert( nobservations == b->nobservations && 
	  npredictors == b->npredictors  );

  // perform the saxpy operation
 
  beta->axpy(alpha, *b->beta);
  t   ->axpy(alpha, *b->t);

  gamma1->axpy(alpha, *b->gamma1);
  gamma2->axpy(alpha, *b->gamma2);

  lambda1->axpy(alpha, *b->lambda1);
  lambda2->axpy(alpha, *b->lambda2);
}


void HuberVars::negate()
{
  beta->negate();
  t   ->negate();

  gamma1->negate();
  gamma2->negate();

  lambda1->negate();
  lambda2->negate();
}

double HuberVars::findBlocking( Variables * step, 
				double & primalValue,
				double & primalStep,
				double & dualValue,
				double & dualStep,
				int& firstOrSecond )
{
  HuberVars * d = (HuberVars *) step;

  double bound = 1;
  firstOrSecond = 0;
  
  bound = gamma1->findBlocking( *d->gamma1, *lambda1, *d->lambda1, 
				bound,
				&primalValue, &primalStep,
				&dualValue,   &dualStep,
				firstOrSecond );
  bound = gamma2->findBlocking( *d->gamma2, *lambda2, *d->lambda2, 
				bound,
				&primalValue, &primalStep,
				&dualValue,   &dualStep,
				firstOrSecond );

  return bound;
}

double HuberVars::stepbound( Variables *d_in)
{
  HuberVars *d = (HuberVars *) d_in;
  assert( nobservations == d->nobservations || 
	  npredictors == d->npredictors ); 
  assert( gamma1 ->allPositive() && gamma2 ->allPositive() );
  assert( lambda1->allPositive() && lambda2->allPositive() );

  double bound = 1.0;
  
  bound = gamma1->stepbound( *d->gamma1, bound );
  bound = gamma2->stepbound( *d->gamma2, bound );

  bound = lambda1->stepbound( *d->lambda1, bound );
  bound = lambda2->stepbound( *d->lambda2, bound );
 
  return bound;
}


void HuberVars::interiorPoint( double alpha, double be )
{
  gamma1->setToConstant( alpha );
  gamma2->setToConstant( alpha );

  lambda1->setToConstant( be );
  lambda2->setToConstant( be );

  this->t   ->setToConstant(0.0);
  this->beta->setToConstant(0.0);
}


void HuberVars::print()
{
  cout << "nobservations= " << nobservations << " npredictors= " 
       << npredictors << endl;
  beta->writefToStream( cout, "beta[%{index}] = %{value}" );
  t   ->writefToStream( cout, "t[%{index}] = %{value}" );
  
  gamma1->writefToStream( cout, "gamma1[%{index}] = %{value}" );
  gamma2->writefToStream( cout, "gamma2[%{index}] = %{value}" );

  lambda1->writefToStream( cout, "lambda1[%{index}] = %{value}" );
  lambda2->writefToStream( cout, "lambda2[%{index}] = %{value}" );
}

//  print only the interesting variables (the coefficients beta)

void HuberVars::printBeta()
{
  cout << endl <<  "nobservations= " << nobservations 
       << " npredictors= " << npredictors << endl << endl;
  beta->writefToStream( cout, "beta[%{index}] = %{value}" );
}


void HuberVars::copy(Variables *b_in)
{
  HuberVars *b = (HuberVars *) b_in;

  beta->copyFrom(*b->beta);
  t->copyFrom(*b->t);

  gamma1->copyFrom(*b->gamma1);
  gamma2->copyFrom(*b->gamma2);

  lambda1->copyFrom(*b->lambda1);
  lambda2->copyFrom(*b->lambda2);
}

void HuberVars::shiftBoundVariables( double a, double b )
{
  gamma1->addConstant( a );
  gamma2->addConstant( a );
  
  lambda1->addConstant( b );
  lambda2->addConstant( b );
}

double HuberVars::violation()
{
  double viol = 0.0;
  double cmin;
  int    iblock;

  gamma1->min( cmin, iblock );
  if( cmin < viol ) viol = cmin;

  gamma2->min( cmin, iblock );
  if( cmin < viol ) viol = cmin;

  lambda1->min( cmin, iblock );
  if( cmin < viol ) viol = cmin;

  lambda2->min( cmin, iblock );
  if( cmin < viol ) viol = cmin;

  return -viol;
}

double HuberVars::onenorm()
{
  double norm = 0.0;
  norm += beta->onenorm();
  norm += t   ->onenorm();

  norm += gamma1->onenorm();
  norm += gamma2->onenorm();

  norm += lambda1->onenorm();
  norm += lambda2->onenorm();

  return norm;
}

double HuberVars::infnorm()
{
  double norm  = beta->infnorm();
  double cnorm = t   ->infnorm();
  if( cnorm > norm ) norm = cnorm;

  cnorm = gamma1->infnorm();
  if( cnorm > norm ) norm = cnorm;

  cnorm = gamma2->infnorm();
  if( cnorm > norm ) norm = cnorm;

  cnorm = lambda1->infnorm();
  if( cnorm > norm ) norm = cnorm;

  cnorm = lambda2->infnorm();
  if( cnorm > norm ) norm = cnorm;

  return norm;
}


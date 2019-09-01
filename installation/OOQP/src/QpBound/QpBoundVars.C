/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundVars.h"
#include "QpBoundData.h"
#include "QpBound.h"
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;

#include <cmath>
#include <cassert>
#include "OoqpVector.h"

QpBoundVars::QpBoundVars( QpBound * f, QpBoundData *prob  )
{
  factory = f;

  nx = prob->getN();
  nlower = prob->getNlower();
  nupper = prob->getNupper();

  x   = OoqpVectorHandle( factory->newPrimalVector() );
  t   = OoqpVectorHandle( factory->newPrimalVector() );
  t->setToZero();
  v   = OoqpVectorHandle( factory->newPrimalVector() );
  v->setToZero();
  tau = OoqpVectorHandle( factory->newPrimalVector() );
  tau->setToZero();
  nu  = OoqpVectorHandle( factory->newPrimalVector() );
  nu->setToZero();

  index_lower = OoqpVectorHandle( prob->getIndexLower() );
  index_upper = OoqpVectorHandle( prob->getIndexUpper() );

  nComplementaryVariables = nlower + nupper;
}

QpBoundVars::QpBoundVars( QpBound * f, QpBoundData *prob,
			  OoqpVector * x_, 
			  OoqpVector * t_,    OoqpVector * v_, 
			  OoqpVector * tau_,  OoqpVector * nu_ )
{
  factory = f;

  nx = prob->getN();
  nlower = prob->getNlower();
  nupper = prob->getNupper();

  SpReferTo( x, x_ );
  SpReferTo( t, t_ );
  SpReferTo( v, v_ );
  SpReferTo( tau, tau_);
  SpReferTo( nu, nu_);

  index_lower = OoqpVectorHandle( prob->getIndexLower() );
  index_upper = OoqpVectorHandle( prob->getIndexUpper() );

  nComplementaryVariables = nlower + nupper;
}

QpBoundVars::~QpBoundVars()
{
}

// the QpBoundVars implementation of mu.

// Aha! This is where we need to know index_upper and index_lower, in
// order to form the inner product correctly and to divide by the
// right integer

double QpBoundVars::mu()
{
  double temp;

  temp =   t->dotProductWith( *tau );
  temp +=  v->dotProductWith( *nu  );

  if(nComplementaryVariables > 0) {
    temp /= nComplementaryVariables;
  }
  return temp;
}

// returns (x + alpha dx)'(z + alpha dz) / n

double QpBoundVars::mustep(Variables *step_in, double alpha)
{
  QpBoundVars *step = (QpBoundVars *) step_in;
  double temp;

  temp  =  t->shiftedDotProductWith( alpha, *step->t,
				     *tau,
				     alpha, *step->tau );

  temp +=  v->shiftedDotProductWith( alpha, *step->v,
				     *nu,
				     alpha, *step->nu );

  if(nComplementaryVariables > 0) temp /= nComplementaryVariables;

  return temp;
}

// the QpBoundVars implementation of saxpy

void QpBoundVars::saxpy(Variables *b_in, double alpha)
{
  QpBoundVars *b = (QpBoundVars *)b_in;

  // check dimensions
  assert(nx == b->nx);
  
  // perform the saxpy operation

  x  ->axpy( alpha, *b->x );

  assert( b->t  ->matchesNonZeroPattern( *index_lower ) &&
	  b->tau->matchesNonZeroPattern( *index_lower ) );
  t  ->axpy( alpha, *b->t );
  tau->axpy( alpha, *b->tau );

  assert( b->v  ->matchesNonZeroPattern( *index_upper ) &&
	  b->nu ->matchesNonZeroPattern( *index_upper ) );
  v  ->axpy( alpha, *b->v );
  nu ->axpy( alpha, *b->nu );
}

void QpBoundVars::negate()
{
  x  ->negate();
  t  ->negate();
  tau->negate();
  v  ->negate();
  nu ->negate();
}

// the QPVars implementation of stepbound

double QpBoundVars::stepbound( Variables *b_in)
{
  QpBoundVars *b = (QpBoundVars *) b_in;
  
  // check dimensions
  assert(nx == b->nx);

  // check positivity of t, tau, v, nu
  assert( t  ->somePositive( *index_lower ) &&
	  tau->somePositive( *index_lower ) );
  assert( v  ->somePositive( *index_upper ) &&
	  nu ->somePositive( *index_upper ) );

  double bound = 1.0;
  bound = t  ->stepbound( *b->t,   bound );
  bound = tau->stepbound( *b->tau, bound );
  bound = v  ->stepbound( *b->v,   bound );
  bound = nu ->stepbound( *b->nu,  bound );

  return bound;
}

// Mehrotra's step bound heuristic, modified for quadratic
// programming, in which we take the same step in both primal and dual
// variables.

double QpBoundVars::findBlocking( Variables * step_in,
				  double & primalValue,
				  double & primalStep,
				  double & dualValue,
				  double & dualStep,
				  int& firstOrSecond )
{
  QpBoundVars * d = (QpBoundVars *) step_in;

  double bound = 1.0; 
  firstOrSecond = 0;

  bound = t->findBlocking( *d->t, *tau, *d->tau, bound,
			   &primalValue, &primalStep,
			   &dualValue, &dualStep,
			   firstOrSecond );
  bound = v->findBlocking( *d->v, *nu, *d->nu,   bound,
			   &primalValue, &primalStep,
			   &dualValue, &dualStep,
			   firstOrSecond );
  return bound;
}


void QpBoundVars::interiorPoint( double alpha, double beta )
{
  x  ->setToZero();
  t  ->setToZero();
  tau->setToZero();
  v  ->setToZero();
  nu ->setToZero();

  t  ->addSomeConstants( alpha, *index_lower );
  tau->addSomeConstants( beta,  *index_lower );

  v  ->addSomeConstants( alpha, *index_upper );
  nu ->addSomeConstants( beta,  *index_upper );
}

double QpBoundVars::violation()
{
  double viol = 0.0, cmin = 0.0; 
  int iblock;
  
  if( nlower > 0 ) {
    t->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
    
    tau->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
  }
  if( nupper > 0 ) {
    v->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
    
    nu->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
  }
  return -viol;
}

void QpBoundVars::shiftBoundVariables( double alpha, double beta ) 
{
  t  ->addSomeConstants( alpha, *index_lower );
  tau->addSomeConstants( beta,  *index_lower );

  v  ->addSomeConstants( alpha, *index_upper );
  nu ->addSomeConstants( beta,  *index_upper );
}

// the QpBoundVars implementation of print()

void QpBoundVars::print()
{
  cout << endl << "Computed Solution x (Primal):" << endl;
  x  ->writefToStream( cout, "%{index}: x= %{value}" );

  cout << endl << "Lower Bound Multipliers:" << endl;
  tau->writefSomeToStream( cout, "tau[%{index}] = %{value}", *index_lower );

  cout << endl << "Upper Bound Multipliers:" << endl;
  nu ->writefSomeToStream( cout, "nu[%{index}] = %{value}", *index_upper );
}

// QPVars implementation of copy

void QpBoundVars::copy(Variables *b_in)
{
  QpBoundVars *b = (QpBoundVars *) b_in;

  x  ->copyFrom( *b->x );
  t  ->copyFrom( *b->t );
  tau->copyFrom( *b->tau );
  v  ->copyFrom( *b->v );
  nu ->copyFrom( *b->nu );
}

double QpBoundVars::onenorm()
{
  double sum = 0;
  sum += x  ->onenorm();
  sum += t  ->onenorm();
  sum += tau->onenorm();
  sum += v  ->onenorm();
  sum += nu ->onenorm();

  return sum;
}

double QpBoundVars::infnorm()
{
  double componentNorm, norm = 0;
  componentNorm = x->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  componentNorm = t->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  componentNorm = tau->infnorm();
  if( componentNorm > norm ) norm = componentNorm;
  
  componentNorm = v->infnorm();
  if( componentNorm > norm ) norm = componentNorm;
  
  componentNorm = nu->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  return norm;
}

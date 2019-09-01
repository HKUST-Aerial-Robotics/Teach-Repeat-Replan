/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <cstring>
#include <iostream>
#include <cmath>
#include <cassert>
using namespace std;

#include "QpExampleVars.h"
#include "DoubleMatrix.h"
#include "QpExampleData.h"
#include "memory.h"
#include "OoqpVector.h"
#include "LinearAlgebraPackage.h"

QpExampleVars::QpExampleVars(LinearAlgebraPackage * la,
			     int nx_in, int my_in, int mz_in)
  : nx( nx_in), my(my_in), mz(mz_in)
{
  x  = la->newVector( nx );
  s  = la->newVector( mz );
  y  = la->newVector( my );
  z  = la->newVector( mz );
}

QpExampleVars::~QpExampleVars()
{
  IotrRelease( &x ); IotrRelease( &s );
  IotrRelease( &y ); IotrRelease( &z );
}


double QpExampleVars::mu()
{
  return ( mz > 0 ) ? ( z->dotProductWith( *s ) / mz ): 0;
}

// returns (x + alpha dx)'(z + alpha dz) / n
double QpExampleVars::mustep(Variables *step_in, double alpha)
{
  QpExampleVars *step = dynamic_cast<QpExampleVars *>(step_in);
  assert( step );

  double dmustep = 0.0;
  if( mz > 0 ) {
    dmustep = z->shiftedDotProductWith( alpha, *step->z,
					*s,
					alpha, *step->s );
    dmustep /= mz;
  }

  return dmustep;
}

// the QpExampleVars implementation of saxpy

void QpExampleVars::saxpy(Variables *b_in, double alpha)
{
  QpExampleVars *b = dynamic_cast<QpExampleVars *>(b_in);

  assert( b && nx == b->nx && my == b->my && mz == b->mz);

  s->axpy( alpha, *b->s );
  z->axpy( alpha, *b->z );
  y->axpy( alpha, *b->y );
  x->axpy( alpha, *b->x );
}


void QpExampleVars::negate()
{
  x->negate();
  y->negate();
  z->negate();
  s->negate();
}


double QpExampleVars::stepbound( Variables *b_in)
{
  QpExampleVars *b = (QpExampleVars *) b_in;
  
  assert(nx == b->nx && my == b->my && mz == b->mz);
  assert( s->allPositive() && z->allPositive() );
  
  // now compute step bound
  double bound = 1.0;
  bound = s->stepbound( *b->s, bound );
  bound = z->stepbound( *b->z, bound );
  
  return bound;
}


void QpExampleVars::print()
{
  cout << "nx= " << nx << " my= " << my << " mz= " << mz << endl;
  
  x->writefToStream( cout, "x[%{index}]=%{value}" );
  y->writefToStream( cout, "y[%{index}]=%{value}" );
  z->writefToStream( cout, "z[%{index}]=%{value}" );
  s->writefToStream( cout, "s[%{index}]=%{value}" );
}


double QpExampleVars::onenorm()
{
  double norm = 0.0;
  norm  = x->onenorm();
  norm += s->onenorm();
  norm += y->onenorm();
  norm += z->onenorm();

  return norm;
}


double QpExampleVars::infnorm()
{
  double norm = x->infnorm();
  
  double componentNorm;
  componentNorm = s->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  componentNorm = y->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  componentNorm = z->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  return norm;
}

void QpExampleVars::copy(Variables *b_in)
{
  QpExampleVars *b = dynamic_cast<QpExampleVars *>(b_in);
  assert(b);

  x->copyFrom( *b->x );
  s->copyFrom( *b->s );
  y->copyFrom( *b->y );
  z->copyFrom( *b->z );
}

double QpExampleVars::findBlocking( Variables * step_in, 
				    double & primalValue,
				    double & primalStep,
				    double & dualValue,
				    double & dualStep,
				    int& firstOrSecond )
{
  QpExampleVars * step = dynamic_cast<QpExampleVars *>(step_in);

  double maxstep = 1.0;
  firstOrSecond  = 0;

  if( mz > 0 ) {
    maxstep = s->findBlocking( *step->s, *z, *step->z, maxstep,
			       &primalValue, &primalStep,
			       &dualValue, &dualStep,	
			       firstOrSecond );
  }

  return maxstep;
}


void QpExampleVars::interiorPoint( double alpha, double beta ) 
{
  x->setToZero();
  y->setToZero();
  if( mz > 0 ) {
    s->setToConstant( alpha );
    z->setToConstant( beta  );
  }
}


void QpExampleVars::shiftBoundVariables( double alpha, double beta )
{
  if( mz > 0 ) {
    s->addConstant( alpha );
    z->addConstant( beta  );
  }
}

double QpExampleVars::violation()
{ 
  double viol = 0.0, cmin = 0.0;
  int iblock;

  if( mz > 0 ) {
    s->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
    z->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
  }

  return -viol;
}

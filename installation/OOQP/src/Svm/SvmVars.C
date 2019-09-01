/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "SvmVars.h"
#include "SvmData.h"
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;

#include <cmath>
#include "SimpleVector.h"
#include "DoubleMatrix.h"

SvmVars::SvmVars(int hyperplanedim_in, int nobservations_in) :
    beta(0)
{
  hyperplanedim = hyperplanedim_in;
  nobservations = nobservations_in;

  w    = SimpleVectorHandle( new SimpleVector( hyperplanedim ) );
  z    = SimpleVectorHandle( new SimpleVector( nobservations ) );
  u    = SimpleVectorHandle( new SimpleVector( nobservations ) );
  v    = SimpleVectorHandle( new SimpleVector( nobservations ) );
  s    = SimpleVectorHandle( new SimpleVector( nobservations ) );

  nComplementaryVariables = 2*nobservations;
}

SvmVars::SvmVars(int hyperplanedim_in, int nobservations_in,
		 double w_in[], double v_in[],
		 double z_in[], double u_in[], double s_in[] ) :
    beta(0)
{
  hyperplanedim = hyperplanedim_in;
  nobservations = nobservations_in;

  w = SimpleVectorHandle( new SimpleVector( w_in, hyperplanedim ) );
  z = SimpleVectorHandle( new SimpleVector( z_in, nobservations ) );
  u = SimpleVectorHandle( new SimpleVector( u_in, nobservations ) );
  v = SimpleVectorHandle( new SimpleVector( v_in, nobservations ) );
  s = SimpleVectorHandle( new SimpleVector( s_in, nobservations ) );

  nComplementaryVariables = 2*nobservations;
}

SvmVars::~SvmVars()
{
  return;
}


// the SvmVars implementation of mu.

double SvmVars::mu()
{
  double temp;
  temp = u->dotProductWith(*z)  + v->dotProductWith(*s);
  if( nComplementaryVariables > 0 ) temp /= nComplementaryVariables;
  return temp;
}

// returns (x + alpha dx)'(z + alpha dz) / n

double SvmVars::mustep(Variables *step_in, double alpha)
{
  SvmVars *step = (SvmVars *) step_in;
  double temp = 0.0;

  temp += u->shiftedDotProductWith( alpha, *step->u,
				    *z,
				    alpha, *step->z );
  temp += v->shiftedDotProductWith( alpha, *step->v,
				    *s,
				    alpha, *step->s );

  if( nComplementaryVariables > 0 ) temp /= nComplementaryVariables;
  return temp;
}

// the SVMars implementation of saxpy

void SvmVars::saxpy( Variables *s_in, double alpha )
{
  SvmVars *step = (SvmVars *) s_in;

  // check dimensions
  assert( hyperplanedim == step->hyperplanedim && 
      nobservations == step->nobservations );

  w->axpy(alpha, *step->w);
  z->axpy(alpha, *step->z);
  u->axpy(alpha, *step->u);
  s->axpy(alpha, *step->s);
  v->axpy(alpha, *step->v);

  beta += alpha * step->beta;

  return;
}

void SvmVars::negate()
{
  // negate each component of the variable OoqpVector
  w->negate();
  z->negate();
  u->negate();
  s->negate();
  v->negate();
  beta = -beta;

  return;
}

// the SvmVars implementation of stepbound

double SvmVars::stepbound( Variables *step_in)
{
  SvmVars *step = (SvmVars *) step_in;
   
  // check dimensions
  assert(  hyperplanedim == step->hyperplanedim &&
	   nobservations == step->nobservations );
  assert( z->allPositive() && s->allPositive() );

  double bound = 1.0;
  bound = z->stepbound( *step->z, bound );
  bound = s->stepbound( *step->s, bound );
  bound = u->stepbound( *step->u, bound );
  bound = v->stepbound( *step->v, bound );

  return bound;
}

void SvmVars::interiorPoint( double a, double b )
{
  w->setToConstant( 0.0 );
  beta = 0.0;
  
  z->setToConstant( a );
  s->setToConstant( a );
  u->setToConstant( b  );
  v->setToConstant( b  );
}

double SvmVars::violation()
{
  double viol = 0.0, cmin;
  int iblock;
  z->min( cmin, iblock );
  if( cmin < viol ) viol = cmin;
  
  s->min( cmin, iblock );
  if( cmin < viol ) viol = cmin;
  
  u->min( cmin, iblock );
  if( cmin < viol ) viol = cmin;
  
  v->min( cmin, iblock );
  if( cmin < viol ) viol = cmin;

  return -viol;
}

void SvmVars::shiftBoundVariables( double a, double b )
{
  s->addConstant( a );
  z->addConstant( a );
  u->addConstant( b );
  v->addConstant( b );
}

// the SvmVars implementation of print()

void SvmVars::print()
{
  cout << "hyperplanedim= " << hyperplanedim << 
    " nobservations= " << nobservations << endl;
  w->writefToStream( cout, "w[%{index}]=%{value}" );
  cout << "beta = " << beta << endl << endl;

  z->writefToStream( cout, "z[%{index}]=%{value}" );
  s->writefToStream( cout, "s[%{index}]=%{value}" );
  u->writefToStream( cout, "u[%{index}]=%{value}" );
  v->writefToStream( cout, "v[%{index}]=%{value}" );
}

// print only the interesting variables

void SvmVars::printCoefs()
{
  cout << endl << endl 
       << "dimension of hyperplane: " << hyperplanedim 
       << ", number of observations: " << nobservations 
       << endl;
  w->writefToStream( cout, "w[%{index}]=%{value}" );
  cout << "beta = " << beta << endl << endl;
}

// SvmVars implementation of copy

void SvmVars::copy(Variables *src_in)
{
  SvmVars *src = (SvmVars *) src_in;
  w->copyFrom(*src->w);
  z->copyFrom(*src->z);
  s->copyFrom(*src->s);
  u->copyFrom(*src->u);
  v->copyFrom(*src->v);
  
  beta = src->beta;
}

double SvmVars::onenorm()
{
  double norm = fabs(beta);
  norm += w->onenorm();
  norm += z->onenorm();
  norm += s->onenorm();
  norm += u->onenorm();
  norm += v->onenorm();

  return norm;
}

double SvmVars::infnorm() 
{
  double norm = fabs(beta);

  double componentNorm = w->infnorm();
  if( norm < componentNorm ) norm = componentNorm;

  componentNorm = s->infnorm();
  if( norm < componentNorm ) norm = componentNorm;

  componentNorm = z->infnorm();
  if( norm < componentNorm ) norm = componentNorm;

  componentNorm = u->infnorm();
  if( norm < componentNorm ) norm = componentNorm;

  componentNorm = v->infnorm();
  if( norm < componentNorm ) norm = componentNorm;

  return norm;
}

double SvmVars::findBlocking( Variables * step_in, 
			      double & primalValue,
			      double & primalStep,
			      double & dualValue,
			      double & dualStep,
			      int& firstOrSecond )
{
  SvmVars * step = (SvmVars *) step_in;

  double alpha = 1.0;
  firstOrSecond = 0;
  alpha = s->findBlocking( *step->s, *v, *step->v, alpha, 
		   &primalValue, &primalStep,
		   &dualValue, &dualStep, 
		   firstOrSecond );

  alpha = z->findBlocking( *step->z, *u, *step->u, alpha,
		   &primalValue, &primalStep,
		   &dualValue, &dualStep, 
		   firstOrSecond );

  return alpha;
}

void SvmVars::asMfile( ostream& os )
{
  os << "function vars = my_vars()\n\n";
  os << "w = [...\n";
  w->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "beta = " << beta << ";\n";

  os << "z = [...\n";
  z->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "s = [...\n";
  s->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "u = [...\n";
  u->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "v = [...\n";
  v->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "vars = struct( 'w', w, 'beta',  beta,  's', s,...\n";
  os << "               'z', z, 'u', u, 'v', v );\n";
}

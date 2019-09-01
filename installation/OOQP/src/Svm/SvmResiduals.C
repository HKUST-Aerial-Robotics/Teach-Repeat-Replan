/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "SvmResiduals.h"
#include "SvmData.h"
#include "SvmVars.h"
#include "SimpleVector.h"
#include "DoubleMatrix.h"
#include <cassert>
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;

#include <cmath>

SvmResiduals::SvmResiduals(int hyperplanedim_in, int nobservations_in)
{
  hyperplanedim = hyperplanedim_in;
  nobservations = nobservations_in;

  wRes     = SimpleVectorHandle( new SimpleVector(  hyperplanedim ) );
  betaRes  = 0;
  zRes     = SimpleVectorHandle( new SimpleVector(  nobservations ) );
  sRes     = SimpleVectorHandle( new SimpleVector(  nobservations ) );
  
  rSV      = SimpleVectorHandle( new SimpleVector(  nobservations ) );
  rUZ      = SimpleVectorHandle( new SimpleVector(  nobservations ) );
}

SvmResiduals::~SvmResiduals()
{
}

// calculate the r1 and r2 components of the residual vector for the
// Huber case

void SvmResiduals::calcresids(Data *prob_in, Variables *vars_in )
{
  SvmData *prob = (SvmData *) prob_in;
  SvmVars *vars = (SvmVars *) vars_in;
  double gap;

  SimpleVector & wVar    = *vars->w;
  SimpleVector & vVar    = *vars->v;
  SimpleVector & uVar    = *vars->u;
  SimpleVector & zVar    = *vars->z;
  SimpleVector & sVar    = *vars->s;
  double             betaVar =  vars->beta;

  // wRes = 2 * w - Y\T v
  wRes->copyFrom( wVar );
  wRes->scale( 2.0 );
  prob->YTransMult( 1.0, *wRes, -1.0, vVar );

  // betaRes = < categories, v >
  betaRes = prob->dotCategories( vVar );

  // zRes = e - v - u
  zRes->setToConstant( prob->penalty() );
  zRes->axpy( -1.0, vVar );
  // Compute the contribution to the gap before subtracting the dual slacks
  gap = zRes->dotProductWith( zVar );
  zRes->axpy( -1.0, uVar );

  // sRes = -e + Y * w - beta * categories + z - s
  sRes->setToConstant( -1.0 );
  prob->YMult(  1.0, *sRes, 1.0, wVar );
  sRes->axpy( -betaVar, *prob->categories );
  sRes->axpy(   1.0, zVar );
  // Compute the contribution to the gap before subtracting the primal slacks
  gap += sRes->dotProductWith( vVar );
  sRes->axpy( -1.0, sVar );

  // calculate the norm of the (r1,r2) residual component
  double norm = fabs( betaRes );
  double componentNorm;
  componentNorm = wRes->infnorm();
  if( norm < componentNorm ) norm = componentNorm;
  componentNorm = zRes->infnorm();
  if( norm < componentNorm ) norm = componentNorm;
  componentNorm = sRes->infnorm();
  if( norm < componentNorm ) norm = componentNorm;
 
  mDualityGap   = gap;
  mResidualNorm = norm;
}

// add componentwise products and a given constant to the "r3"
// residual component

void SvmResiduals::add_r3_xz_alpha( Variables *vars_in, double value )
{
  SvmVars *vars = (SvmVars *) vars_in;

  rSV->axzpy( 1.0, *vars->s, *vars->v );
  rSV->addConstant( value );
  rUZ->axzpy( 1.0, *vars->u, *vars->z );
  rUZ->addConstant( value );

}

// set "r3" component to componentwise products plus a given constant

void SvmResiduals::set_r3_xz_alpha( Variables *vars_in, double value )
{
  rSV->setToZero();
  rUZ->setToZero();
  this->add_r3_xz_alpha(vars_in, value );
  return;
}

void SvmResiduals::clear_r1r2()
{
  wRes->setToZero();
  betaRes = 0;
  zRes->setToZero();
  sRes->setToZero();
}


void SvmResiduals::clear_r3()
{
  rSV->setToZero();
  rUZ->setToZero();
}

void SvmResiduals::project_r3(double rmin, double rmax)
{
  rSV->gondzioProjection(rmin, rmax);
  rUZ->gondzioProjection(rmin, rmax);
  return;
}


void SvmResiduals::asMfile( ostream& os )
{
  os << "function vars = my_vars()\n\n";
  os << "wRes = [...\n";
  wRes->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "betaRes = " << betaRes << ";\n";

  os << "zRes = [...\n";
  zRes->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "sRes = [...\n";
  sRes->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "uRes = [...\n";
  rUZ->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os << "vRes = [...\n";
  rSV->writefToStream( os, "%{value};..." );
  os << "];\n\n";

  os <<
    "vars = struct( 'wRes', wRes, 'betaRes',  betaRes,  'sRes', sRes,...\n";
  os << "               'zRes', zRes, 'uRes', uRes, 'vRes', vRes );\n";
}


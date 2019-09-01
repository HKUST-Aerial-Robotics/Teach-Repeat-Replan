/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "DoubleMatrix.h"
#include "QpExampleResids.h"
#include "QpExampleData.h"
#include "QpExampleVars.h"
#include "OoqpVector.h"
#include "LinearAlgebraPackage.h"


QpExampleResids::QpExampleResids( LinearAlgebraPackage * la,
				  int nx_in, int my_in, int mz_in )
  : nx( nx_in ), my( my_in ), mz( mz_in )
{
  rQ = la->newVector( nx );
  rA = la->newVector( my );
  rC = la->newVector( mz );
  r3 = la->newVector( mz );
}


QpExampleResids::~QpExampleResids()
{
  IotrRelease( &rQ ); IotrRelease( &rA );
  IotrRelease( &rC ); IotrRelease( &r3 );
}


void QpExampleResids::calcresids(Data *prob_in, 
				 Variables *vars_in )
{
  QpExampleData *prob = dynamic_cast<QpExampleData *>(prob_in);
  QpExampleVars *vars = dynamic_cast<QpExampleVars *>(vars_in);
  assert( prob && vars );

  // rQ = c + Q * x - A\T * y - C\T * z
  prob->getg( *rQ );
  prob->Qmult( 1.0, *rQ, 1.0, *vars->x );
  prob->ATransmult( 1.0, *rQ, -1.0, *vars->y );
  prob->CTransmult( 1.0, *rQ, -1.0, *vars->z  );

  // rA = A * x - b
  if( my > 0 ) {
    prob->getb( *rA );
    prob->Amult( -1.0, *rA, 1.0, *vars->x );
  }
  // rC = C * x - d - s
  if( mz > 0 ) {
    prob->getd( *rC );
    rC->axpy( 1.0, *vars->s );
    prob->Cmult( -1.0, *rC, 1.0, *vars->x );
  }

  double temp;
  mResidualNorm = rQ->infnorm();
  temp  = rA->infnorm();
  if( temp > mResidualNorm ) mResidualNorm = temp;
}


// add vars->s * vars->z to the r3 component
void QpExampleResids::add_r3_xz_alpha(Variables *vars_in, double alpha)
{
  if( mz > 0 ) {
    QpExampleVars * vars = dynamic_cast<QpExampleVars *>(vars_in);
    assert( vars );
    
    r3->axzpy( 1.0, *vars->s, *vars->z );
    if( alpha != 0.0 ) r3->addConstant( alpha );
  }
}

void QpExampleResids::set_r3_xz_alpha(Variables *vars_in, double alpha)
{
  if ( mz > 0 ) {
    r3->setToZero();
    this->add_r3_xz_alpha(vars_in, alpha);
  }
}


void QpExampleResids::clear_r1r2()
{
  rQ->setToZero(); rA->setToZero(); rC->setToZero();
}
  

void QpExampleResids::clear_r3()
{
  r3->setToZero();
}


void QpExampleResids::project_r3(double rmin, double rmax)
{
  r3->gondzioProjection( rmin, rmax );
}


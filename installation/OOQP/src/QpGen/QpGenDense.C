/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenDense.h"

#include "DenseLinearAlgebraPackage.h"
#include "DenseSymMatrix.h"
#include "DenseGenMatrix.h"
#include "SimpleVector.h"
#include "SimpleVectorHandle.h"
#include "DeSymIndefSolver.h"
#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenDenseLinsys.h"

QpGenDense::QpGenDense( int nx_, int my_, int mz_ )
  : QpGen( nx_, my_, mz_ ) 
{
  la = DenseLinearAlgebraPackage::soleInstance();
}

QpGenDense::QpGenDense( int nx_, int my_, int mz_,
			int /* nnzQ */, int /* nnzA */, int /* nnzC */ )
  : QpGen( nx_, my_, mz_ ) 
{
  la = DenseLinearAlgebraPackage::soleInstance();
}

LinearSystem  * QpGenDense::makeLinsys( Data * prob_in )
{
  QpGenData * prob = (QpGenData *) prob_in;

  int n = nx + my + mz;
  DenseSymMatrixHandle Mat( new DenseSymMatrix(n) );

  DeSymIndefSolver * solver = new DeSymIndefSolver( Mat );

  return new QpGenDenseLinsys( this, prob, la, Mat, solver );
}

QpGenData * QpGenDense::makeData( double    c[],  double   Q[],
				  double xlow[],  char ixlow[], 
				  double xupp[],  char ixupp[],
				  double    A[],  double  bA[],
				  double    C[],  
				  double clow[],  char iclow[],
				  double cupp[],  char icupp[] )
{
  SimpleVectorHandle   vc( new SimpleVector( c, nx ) );
  DenseSymMatrixHandle mQ   ( new DenseSymMatrix( Q, nx ) );
  SimpleVectorHandle   vxlow( new SimpleVector( xlow, nx ) );
  SimpleVectorHandle   vxupp( new SimpleVector( xupp, nx ) );
  DenseGenMatrixHandle mA   ( new DenseGenMatrix( A, my, nx ) );
  SimpleVectorHandle   vbA( new SimpleVector( bA, my ) );
  DenseGenMatrixHandle mC   ( new DenseGenMatrix( C, mz, nx) );
  SimpleVectorHandle   vclow( new SimpleVector( clow, mz ) );
  SimpleVectorHandle   vcupp( new SimpleVector( cupp, mz ) );
  
  SimpleVectorHandle   vixlow( new SimpleVector( nx ) );
  SimpleVectorHandle   vixupp( new SimpleVector( nx ) );
  int i;
  for( i = 0; i < nx; i++ ) {
    (*vixlow)[i] = (ixlow[i] == 0) ? 0.0 : 1.0;
    (*vixupp)[i] = (ixupp[i] == 0) ? 0.0 : 1.0;
  }

  SimpleVectorHandle   viclow( new SimpleVector( mz ) );
  SimpleVectorHandle   vicupp( new SimpleVector( mz ) );

  for( i = 0; i < mz; i++ ) {
    (*viclow)[i] = (iclow[i] == 0) ? 0.0 : 1.0;
    (*vicupp)[i] = (icupp[i] == 0) ? 0.0 : 1.0;
  }

  return new QpGenData( la, vc, mQ, vxlow, vixlow, vxupp, vixupp,
			mA, vbA,
			mC, vclow, viclow, vcupp, vicupp );
}

QpGenData *  QpGenDense::makeData()
{
  return new QpGenData( la, nx, my, mz, nx * nx, my * nx, mz * nx );
}



void QpGenDense::joinRHS( OoqpVector& rhs_in,  OoqpVector& rhs1_in,
			  OoqpVector& rhs2_in, OoqpVector& rhs3_in )
{
  SimpleVector & rhs  = (SimpleVector &) rhs_in;
  SimpleVector & rhs1 = (SimpleVector &) rhs1_in;
  SimpleVector & rhs2 = (SimpleVector &) rhs2_in;
  SimpleVector & rhs3 = (SimpleVector &) rhs3_in;

  memcpy( &rhs[0], &rhs1[0], nx * sizeof( double ) );
  if( my > 0 ) memcpy( &rhs[nx],      &rhs2[0], my * sizeof( double ) );
  if( mz > 0 ) memcpy( &rhs[nx + my], &rhs3[0], mz * sizeof( double ) );
}

void
QpGenDense::separateVars( OoqpVector& x_in, OoqpVector& y_in,
			      OoqpVector& z_in, OoqpVector& vars_in )
{
  SimpleVector & vars  = (SimpleVector &) vars_in;
  SimpleVector & x = (SimpleVector &) x_in;
  SimpleVector & y = (SimpleVector &) y_in;
  SimpleVector & z = (SimpleVector &) z_in;

  memcpy( &x[0], &vars[0], nx * sizeof( double ) );
  if ( my > 0 ) memcpy( &y[0], &vars[nx],      my * sizeof( double ) );
  if ( mz > 0 ) memcpy( &z[0], &vars[nx + my], mz * sizeof( double ) );
}

void QpGenDense::makeRandomData( QpGenData *& data, QpGenVars *& soln )
{
  data =
    new QpGenData( la, nx, my, mz, nx * nx, nx * my, nx * mz );

  OoqpVectorHandle x( la->newVector( nx ) );
  OoqpVectorHandle y( la->newVector( my ) );
  OoqpVectorHandle z( la->newVector( mz ) );
  OoqpVectorHandle s( la->newVector( mz ) );

  data->datarandom( *x, *y, *z, *s );

  soln = (QpGenVars * ) this->makeVariables( data );

  soln->x->copyFrom( *x );
  soln->y->copyFrom( *y );
  soln->z->copyFrom( *z );
  soln->s->copyFrom( *s );
}

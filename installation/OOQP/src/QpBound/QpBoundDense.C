/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundDense.h"
#include "QpBoundDenseLinsys.h"
#include "QpBoundData.h"
#include "DeSymPSDSolver.h"
#include "DenseSymMatrix.h"
#include "DenseLinearAlgebraPackage.h"

#include "SimpleVector.h"
#include "SimpleVectorHandle.h"

LinearSystem  * QpBoundDense::makeLinsys( Data * prob_in )
{
  QpBoundData *prob = (QpBoundData *)prob_in;

  DenseSymMatrixHandle Mat( new DenseSymMatrix( prob->getN() ) );

  DoubleLinearSolver * solver = new DeSymPSDSolver( Mat );
  
  OoqpVectorHandle dq( this->newPrimalVector() );
  OoqpVectorHandle ixlow( prob->getIndexLower() );
  OoqpVectorHandle ixupp( prob->getIndexUpper() );

  return new QpBoundDenseLinsys( this, Mat, solver, dq, prob->getN(),
				 ixlow, ixupp );
}

QpBoundData      * QpBoundDense::makeData()
{
  SymMatrixHandle Q( la->newSymMatrix(nx, nx*nx ) );
  QpBoundData * data = new QpBoundData( this, Q, nx );

  return data;
}

QpBoundDense::QpBoundDense(int n) : QpBound( n )
{
  la = DenseLinearAlgebraPackage::soleInstance();
}


QpBoundData * QpBoundDense::makeData( double alpha,
					   double dc[],     double dQ[],
					   double dxlow[],  char cxlow[],
					   double dxupp[],  char cxupp[] )
{
  SimpleVectorHandle    c( new SimpleVector( dc, nx ) );
  DenseSymMatrixHandle  Q( new DenseSymMatrix( dQ, nx ) );
  SimpleVectorHandle    xlow( new SimpleVector( dxlow, nx ) );
  SimpleVectorHandle    xupp( new SimpleVector( dxupp, nx ) );
  SimpleVectorHandle    ixlow( new SimpleVector( nx ) );
  SimpleVectorHandle    ixupp( new SimpleVector( nx ) );

  int i;
  for( i = 0; i < nx ; i++ ) {
    (*ixlow)[i] = ( cxlow[i] == 0 ) ? 0.0 : 1.0;
    (*ixupp)[i] = ( cxupp[i] == 0 ) ? 0.0 : 1.0;
  }

  return new QpBoundData( this, alpha, c, Q, xlow, ixlow, xupp, ixupp );
}

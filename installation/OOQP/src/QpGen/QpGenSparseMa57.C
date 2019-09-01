/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenSparseMa57.h"
#include "QpGenSparseLinsys.h"
#include "QpGenData.h"

#include "SparseLinearAlgebraPackage.h"
#include "SparseSymMatrix.h"
#include "SimpleVector.h"
#include "SimpleVectorHandle.h"
#include "Ma57Solver.h"
#include "SparseLinearAlgebraPackage.h"


QpGenSparseMa57::QpGenSparseMa57( int nx, int my, int mz,
					  int nnzQ, int nnzA, int nnzC ) :
  QpGenSparseSeq( nx, my, mz, nnzQ, nnzA, nnzC )
{
  la = SparseLinearAlgebraPackage::soleInstance();
}

LinearSystem * QpGenSparseMa57::makeLinsys( Data * prob_in )
{
  QpGenData * prob = (QpGenData *) prob_in;
  int n = nx + my + mz;

  SparseSymMatrixHandle Mat( new SparseSymMatrix( n,n + nnzQ
						      + nnzA + nnzC ) );

  SimpleVectorHandle v( new SimpleVector(n) );
  v->setToZero();
  Mat->setToDiagonal(*v);

  prob->putQIntoAt( *Mat, 0, 0 );
  prob->putAIntoAt( *Mat, nx, 0);
  prob->putCIntoAt( *Mat, nx + my, 0 );
  
  Ma57Solver * solver = new Ma57Solver( Mat );
  
  return new QpGenSparseLinsys( this, prob, la, Mat, solver );
}





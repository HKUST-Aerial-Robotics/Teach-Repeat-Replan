/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/*
 * Edited by: Quan H. Nguyen 
 */
#include "QpGenSparseOblio.h"
#include "QpGenData.h"
#include "QpGenSparseLinsys.h"

#include "SparseLinearAlgebraPackage.h"
#include "SparseGenMatrix.h"
#include "SimpleVector.h"
#include "OblioSolver.h"
#include "SparseLinearAlgebraPackage.h"
#include "SparseSymMatrix.h"

OblioQpGen::OblioQpGen( int nx_, int my_, int mz_,
					  int nnzQ_, int nnzA_, int nnzC_ ) :
  QpGenSparseSeq( nx_, my_, mz_, nnzQ_, nnzA_, nnzC_ )
{
  la = SparseLinearAlgebraPackage::soleInstance();
}


LinearSystem * OblioQpGen::makeLinsys( Data * prob_in )
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

  OblioSolver * solver = new OblioSolver( Mat);
  
  return new QpGenSparseLinsys( this, prob, la, Mat, solver );  
}


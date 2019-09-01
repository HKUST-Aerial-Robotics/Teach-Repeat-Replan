/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/*
 * Edited by: Quan H. Nguyen 
 */
#include "QpGenSparseSuperLu.h"
#include "QpGenSparseLinsys.h"
#include "QpGenData.h"

#include "SparseLinearAlgebraPackage.h"
#include "SparseGenMatrix.h"
#include "SimpleVector.h"
#include "SuperLuSolver.h"
#include "SparseLinearAlgebraPackage.h"

SuperLUQpGen::SuperLUQpGen( int nx, int my, int mz,
					  int nnzQ, int nnzA, int nnzC ) :
  QpGenSparseSeq( nx, my, mz, nnzQ, nnzA, nnzC )
{
  la = SparseLinearAlgebraPackage::soleInstance();
}

LinearSystem * SuperLUQpGen::makeLinsys( Data * prob_in )
{
  QpGenData * prob = (QpGenData *) prob_in;
  int n = nx + my + mz;

  SparseGenMatrix *Mat= new SparseGenMatrix( n, n, n + 2*nnzQ + 2 * nnzA + 2 * nnzC );

  SimpleVector v(n);
  v.setToZero();
  Mat->setToDiagonal(v);

  prob->putQIntoAt( *Mat, 0, 0 );
  prob->putAIntoAt( *Mat, nx, 0);
  prob->putCIntoAt( *Mat, nx + my, 0 );
   
  int info;
  Mat->symmetrize( info );
  assert( info == 0 ); // Has to be Ok, we allocated the matrix at max size.

  SuperLUSolverHandle solver( new SuperLUSolver( Mat ) );
  
  return new QpGenSparseLinsys( this, prob, la, (SparseSymMatrix*) Mat, solver);
}

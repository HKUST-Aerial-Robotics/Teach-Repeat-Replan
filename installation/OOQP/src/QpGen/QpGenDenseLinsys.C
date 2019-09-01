/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenDenseLinsys.h"
#include "QpGenData.h"
#include "DenseSymMatrix.h"
#include "DoubleLinearSolver.h"

QpGenDenseLinsys::QpGenDenseLinsys( QpGen * qpgen,
			      QpGenData * data,
			      LinearAlgebraPackage * la, DenseSymMatrix * Mat,
			      DoubleLinearSolver * solver_in ) :
  QpGenLinsys( qpgen, data, la ), solver(solver_in)
{
  SpReferTo( kkt, Mat );
}

void QpGenDenseLinsys::factor(Data *prob_in, Variables *vars)
{
  QpGenData * prob = (QpGenData *) prob_in;
  
  prob->putQIntoAt( *kkt, 0,       0 );
  prob->putAIntoAt( *kkt, nx,      0 );
  prob->putCIntoAt( *kkt, nx + my, 0 );
  kkt->symAtPutZeros( nx, nx, my+mz, my+mz );

  QpGenLinsys::factor( prob, vars );
  solver->matrixChanged();
}

void QpGenDenseLinsys::putXDiagonal( OoqpVector& xdiag )
{
  kkt->atPutDiagonal( 0, xdiag );
}


void QpGenDenseLinsys::putZDiagonal( OoqpVector& zdiag )
{
  kkt->atPutDiagonal( nx + my, zdiag );
}


void QpGenDenseLinsys::solveCompressed( OoqpVector & compressedRhs )
{
  solver->solve( compressedRhs );
}  


QpGenDenseLinsys::~QpGenDenseLinsys()
{
  delete solver;
}

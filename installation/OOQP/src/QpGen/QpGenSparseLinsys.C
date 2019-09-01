/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenSparseLinsys.h"
#include "DoubleLinearSolver.h"
#include "SparseSymMatrix.h"

QpGenSparseLinsys::QpGenSparseLinsys(  QpGen * factory_in,
				       QpGenData * data,
				       LinearAlgebraPackage * la,
				       SparseSymMatrix * Mat_in,
				       DoubleLinearSolver * solver_in )
  : QpGenLinsys( factory_in, data, la ), solver(solver_in)
{
  SpReferTo( Mat, Mat_in );
}


void QpGenSparseLinsys::putXDiagonal( OoqpVector& xdiag )
{
  Mat->atPutDiagonal( 0, xdiag );
}


void QpGenSparseLinsys::putZDiagonal( OoqpVector& zdiag )
{
  Mat->atPutDiagonal( nx + my, zdiag );
}


void QpGenSparseLinsys::solveCompressed( OoqpVector & arhs )
{
  solver->solve( arhs );
}  


void QpGenSparseLinsys::factor(Data *prob, Variables *vars)
{
  this->QpGenLinsys::factor( prob, vars );
  solver->matrixChanged();
}

QpGenSparseLinsys::~QpGenSparseLinsys()
{
  delete solver;
}

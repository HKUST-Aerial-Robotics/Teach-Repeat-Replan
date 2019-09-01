/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundDenseLinsys.h"
#include "QpBoundData.h"
#include "DenseSymMatrix.h"
  
QpBoundDenseLinsys::QpBoundDenseLinsys( QpBound * f, DenseSymMatrix * M,
										DoubleLinearSolver * s,
					OoqpVector * diagq, int nx_,
					OoqpVector * ixlow,
					OoqpVector * ixupp ) :
  QpBoundLinsys( f, M, s, diagq, nx_, ixlow, ixupp )
{
}

void QpBoundDenseLinsys::factor(Data *prob_in, Variables *vars_in)
{
  QpBoundData *prob = (QpBoundData *) prob_in;

  DenseSymMatrix & dMat = dynamic_cast<DenseSymMatrix &>( *Mat );

  prob->putQIntoAt( dMat, 0, 0 );
  dMat.getDiagonal( *dq );

  this->QpBoundLinsys::factor( prob_in, vars_in );
}

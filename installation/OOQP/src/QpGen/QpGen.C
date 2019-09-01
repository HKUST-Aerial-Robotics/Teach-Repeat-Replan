/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGen.h"
#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"

QpGen::QpGen( int nx_, int my_, int mz_ ) : la(0)
{
  nx = nx_;
  my = my_;
  mz = mz_;
}


Residuals * QpGen::makeResiduals( Data * prob_in )
{
  QpGenData * prob = (QpGenData *) prob_in;

  return new QpGenResiduals( la,
			     nx, my, mz,
			     prob->ixlow, prob->ixupp,
			     prob->iclow, prob->icupp );
}


Variables * QpGen::makeVariables( Data * prob_in )
{
  QpGenData * prob = (QpGenData *) prob_in;

  return new QpGenVars( la,
			nx, my, mz,
			prob->ixlow, prob->ixupp,
			prob->iclow, prob->icupp );
}

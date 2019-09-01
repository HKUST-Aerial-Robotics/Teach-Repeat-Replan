/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpExampleDense.h"
#include "QpExampleData.h"
#include "QpExampleVars.h"
#include "QpExampleResids.h"
#include "QpExampleDenseLinsys.h"

#include "DeSymIndefSolver.h"
#include "DenseSymMatrix.h"
#include "DenseLinearAlgebraPackage.h"

QpExampleDense::QpExampleDense( int nx, int my, int mz )  
  : mNx(nx), mMy(my), mMz(mz)
{
  la = DenseLinearAlgebraPackage::soleInstance();
}


Data * QpExampleDense::makeData()
{
  return new QpExampleData( la, mNx, mMy, mMz );
}


Data * QpExampleDense::makeRandomData()
{
  QpExampleData * data = (QpExampleData *) this->makeData();
  data->datarandom();
  
  return data;
}


Residuals * QpExampleDense::makeResiduals(Data * prob_in)
{
  return new QpExampleResids( la, mNx, mMy, mMz );
}


Variables * QpExampleDense::makeVariables( Data * prob_in )
{
  return new QpExampleVars( la, mNx, mMy, mMz );
}


LinearSystem  * QpExampleDense::makeLinsys( Data * prob_in )
{
  QpExampleData *prob = dynamic_cast<QpExampleData *>(prob_in);
  assert(prob);

  DenseSymMatrix * Mat          = new DenseSymMatrix ( mNx + mMy );
  DoubleLinearSolver * solver   = new DeSymIndefSolver( Mat );
  QpExampleDenseLinsys * linsys = new QpExampleDenseLinsys( la, Mat, solver,
							    mNx, mMy, mMz );
  IotrRelease( &Mat );

  return linsys;
}




/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundPetsc.h"
#include "PetscIterativeSolver.h"
#include "PetscSpSymMatrix.h"
#include "PetscVector.h"
#include "PetscLinearAlgebraPackage.h"
#include "QpBoundLinsys.h"
#include "QpBoundData.h"
#include "QpBoundVars.h"
#include "QpBoundResiduals.h"
#include "petscksp.h"

QpBoundPetsc::QpBoundPetsc( int nx ) :
  QpBound( nx ), m(PETSC_DECIDE), mKsp(0)
{
  la = PetscLinearAlgebraPackage::soleInstance();
}

QpBoundPetsc::QpBoundPetsc( int m_in, int n_in )
  : QpBound( n_in ), m(m_in), mKsp(0)
{
  la = PetscLinearAlgebraPackage::soleInstance();
}

QpBoundPetsc::QpBoundPetsc( int m_in, int nx_in, KSP ksp )
  : QpBound( nx_in ), m(m_in), mKsp(ksp)
{
  la = PetscLinearAlgebraPackage::soleInstance();
}

OoqpVector * QpBoundPetsc::newPrimalVector()
{
  return new PetscVector( m, nx );
}

LinearSystem  * QpBoundPetsc::makeLinsys( Data * prob_in )
{
  QpBoundData * prob = (QpBoundData *) prob_in;

  PetscSpSymMatrixHandle Mat( new PetscSpSymMatrix( m, m, nx, nx*nx ) );
  
  prob->putQIntoAt( *Mat, 0, 0 );
  OoqpVectorHandle dq   = OoqpVectorHandle( this->newPrimalVector() );
  Mat->getDiagonal( *dq );

  PetscIterativeSolver * solver = 0;
  if( mKsp ) {
    solver = new PetscIterativeSolver( Mat, mKsp );
  } else {
    solver = new PetscIterativeSolver( Mat, KSPCG, PCJACOBI );
  }

  OoqpVectorHandle ixlow( prob->getIndexLower() );
  OoqpVectorHandle ixupp( prob->getIndexUpper() );

  return new QpBoundLinsys( this, Mat, solver, dq, nx,
			    ixlow, ixupp );
} 


QpBoundData *QpBoundPetsc::makeData( double alpha, Vec g_, Mat Q_,
				    Vec l_, Vec il_, Vec u_, Vec iu_ )
{
  PetscSpSymMatrixHandle Q( new PetscSpSymMatrix(Q_));
  PetscVectorHandle g( new PetscVector(g_) );
  PetscVectorHandle l( new PetscVector(l_) );
  PetscVectorHandle u( new PetscVector(u_) );
  PetscVectorHandle il( new PetscVector(il_) );
  PetscVectorHandle iu( new PetscVector(iu_) );

  return new QpBoundData( this, alpha, g, Q, l, il, u, iu );
}


Variables * QpBoundPetsc::makeVariables(Data * prob)
{
  return this->QpBound::makeVariables( prob );
}

Residuals       * QpBoundPetsc::makeResiduals(Data * prob)
{
  return this->QpBound::makeResiduals( prob );
}

Variables * QpBoundPetsc::makeVariables( Data * problem, 
						Vec x_, 
						Vec t_,   Vec v_, 
						Vec tau_, Vec nu_ )
{
  PetscVectorHandle x(   new PetscVector(x_) );
  PetscVectorHandle t(   new PetscVector(t_) );
  PetscVectorHandle v(   new PetscVector(v_) );
  PetscVectorHandle tau( new PetscVector(tau_) );
  PetscVectorHandle nu(  new PetscVector(nu_) );

  return new QpBoundVars( this, (QpBoundData *) problem,
			   x, t, v, tau, nu );
}

Residuals * QpBoundPetsc::makeResiduals( Data * problem, 
						Vec gl_ )
{
  PetscVectorHandle gl( new PetscVector(gl_) );
  
  return new QpBoundResiduals( this, gl, (QpBoundData *) problem, nx );
}

QpBoundData *QpBoundPetsc::makeData()
{
  PetscSpSymMatrixHandle Q( new PetscSpSymMatrix( m, m, nx, nx * nx ) );
  
  return new QpBoundData( this, Q, nx ); 
}


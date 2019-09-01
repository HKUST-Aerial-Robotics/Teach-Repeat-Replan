/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpExampleDenseLinsys.h"
#include <cmath>
#include "memory.h"
#include <cstring>
#include <iostream>
using namespace std;

#include "QpExampleData.h"
#include "QpExampleVars.h"
#include "QpExampleResids.h"
#include "DoubleLinearSolver.h"

#include "DeSymIndefSolver.h"
#include "DenseSymMatrix.h"
#include "DenseGenMatrix.h"
#include "SimpleVector.h"
#include "LinearAlgebraPackage.h"



QpExampleDenseLinsys::QpExampleDenseLinsys( LinearAlgebraPackage * la_in, 
					    DenseSymMatrix *  Mat_in,
					    DoubleLinearSolver * solver_in,
				  int nx_in, int my_in, int mz_in )
  : la(la_in), Mat( Mat_in ), solver(solver_in), 
  nx( nx_in ), my( my_in ), mz( mz_in )
{
  IotrAddRef( &Mat );
}

QpExampleDenseLinsys::~QpExampleDenseLinsys()
{
  IotrRelease( &Mat );
  delete solver;
}

// performs a symmetric indefinite factorization of the matrix
// [ Q + C'*inv(S)*z*C    A' ]
// [         A               0  ]  
// stores the factors in M, of size (my+nx) x (my+nx).
void QpExampleDenseLinsys::factor(Data *prob_in, Variables *vars_in)
{
  int i;

  QpExampleData *prob = (QpExampleData *) prob_in;
  QpExampleVars *vars = (QpExampleVars *) vars_in;

  // setup the system by copying from "prob" and "vars"
  Mat->symAtPutZeros(nx, nx, my, my );

  prob->putAIntoAt( *Mat, nx, 0 );
  prob->putQIntoAt( *Mat, 0, 0 );

  SimpleVector &  z   = dynamic_cast<SimpleVector&>(*vars->z);
  SimpleVector &  s   = dynamic_cast<SimpleVector&>(*vars->s);
  {
    // Get C as a DenseMatrix
    DenseGenMatrix * C  = dynamic_cast<DenseGenMatrix *>(prob->getC());

    for( i = 0; i < mz; i++ ) {
      Mat->atAddOuterProductOf( 0,    0, z[i]/s[i], 
				(*C)[i], 1, nx );
      //      prob->addSymProdCRowToAt( z[i]/s[i], i, Mat, 0 );
    }
    IotrRelease( &C );
    // perform the factorization using LAPACK
  }
  solver->matrixChanged();
}

// solves the system
// [ Q + C'*inv(S)* z *C    A' ] [ dw    ]   [C'inv(S)[r3 + Z rC] + rQ]
// [         A               0  ] [ dlam  ] = [         rA                ]
//
// and computes ds, dpi by
// ds = C dw - rC
// dpi = (r3 - z ds)/s
//
// (dw,dlam,dpi,ds) are stored in 'step'.
void QpExampleDenseLinsys::solve(Data *prob_in, Variables *vars_in, 
			    Residuals *rhs_in, Variables *step_in)
{
  QpExampleData *prob = (QpExampleData *) prob_in;
  QpExampleVars *vars = (QpExampleVars *) vars_in;
  QpExampleVars *step = (QpExampleVars *) step_in;
  QpExampleResids *rhs = (QpExampleResids *) rhs_in;

  int i, j;
  int size = my+nx;

  // z will hold the rhs of the reduced system.
  OoqpVectorHandle z( la->newVector( size ) );
  // The top part of the right-hand size
  OoqpVectorHandle rhs1( la->newVector( nx ) );

  // z[0..nx-1] = rQ + C'inv(S)[r3 + Z rC]
  // Temporarily let step->z be scratch space.
  // scratch = step->z = inv(S)[r3 + Z rC]
  step->z->copyFrom( *rhs->r3 );
  step->z->axzpy( 1.0, *vars->z, *rhs->rC );
  step->z->componentDiv( *vars->s );

  // z[0..nx-1] = rQ + C' * scratch
  rhs1->copyFrom( *rhs->rQ );
  prob->CTransmult( 1.0, *rhs1, 1.0, *step->z );

  this->joinRHS( *rhs1, *rhs->rA, *z );

  // solve the linear system M*x = z, using the previously computed
  // factor.
  solver->solve( *z );
  // now fill in step:
  
  OoqpVectorHandle vars2( la->newVector( my ) );
  this->separateVars( *z, *step->x, *step->y );
  step->y->negate();

  // ds = C dw - rC
  step->s->copyFrom( *rhs->rC );
  prob->Cmult( -1.0, *step->s, 1.0, *step->x );

  // dpi = (r3 - z ds)/s
  step->z->copyFrom( *rhs->r3 );
  step->z->axzpy( -1.0, *vars->z, *step->s );
  step->z->componentDiv( *vars->s );
}

void QpExampleDenseLinsys::joinRHS( OoqpVector& rhs1, 
				    OoqpVector& rhs2,
				    OoqpVector& rhs )
{
  SimpleVector & srhs  = dynamic_cast<SimpleVector &>(rhs);
  SimpleVector & srhs1 = dynamic_cast<SimpleVector &>(rhs1);
  SimpleVector & srhs2 = dynamic_cast<SimpleVector &>(rhs2);

  int i;
  for( i = 0; i < nx; i++ ) {
    srhs[i] = srhs1[i];
  }
  for( i = 0; i < my; i++ ) {
    srhs[nx + i] = srhs2[i];
  }
}

void QpExampleDenseLinsys::separateVars( OoqpVector& vars, 
					 OoqpVector& vars1,
					 OoqpVector& vars2 )
{
  SimpleVector & svars  = dynamic_cast<SimpleVector &>(vars);
  SimpleVector & svars1 = dynamic_cast<SimpleVector &>(vars1);
  SimpleVector & svars2 = dynamic_cast<SimpleVector &>(vars2);

  int i;
  for( i = 0; i < nx; i++ ) {
    svars1[i] = svars[i];
  }
  for( i = 0; i < my; i++ ) {
    svars2[i] = svars[i + nx];
  }
}


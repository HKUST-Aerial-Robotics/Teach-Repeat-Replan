/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenSparseSeq.h"
#include "QpGenData.h"
#include "SimpleVector.h"
#include "SparseGenMatrix.h"
#include "SparseSymMatrix.h"
#include "SparseLinearAlgebraPackage.h"
#include "QpGenVars.h"

Data * QpGenSparseSeq::makeData()
{
  return new QpGenData( la, nx, my, mz, nnzQ, nnzA, nnzC );
}

void QpGenSparseSeq::joinRHS( OoqpVector& rhs_in,  OoqpVector& rhs1_in,
				   OoqpVector& rhs2_in, OoqpVector& rhs3_in )
{
  SimpleVector & rhs  = dynamic_cast<SimpleVector &>(rhs_in);
  SimpleVector & rhs1 = dynamic_cast<SimpleVector &>(rhs1_in);
  SimpleVector & rhs2 = dynamic_cast<SimpleVector &>(rhs2_in);
  SimpleVector & rhs3 = dynamic_cast<SimpleVector &>(rhs3_in);

  memcpy( &rhs[0], &rhs1[0], nx * sizeof( double ) );
  if( my > 0 ) memcpy( &rhs[nx],      &rhs2[0], my * sizeof( double ) );
  if( mz > 0 ) memcpy( &rhs[nx + my], &rhs3[0], mz * sizeof( double ) );
}

void
QpGenSparseSeq::separateVars( OoqpVector& x_in, OoqpVector& y_in,
				   OoqpVector& z_in, OoqpVector& vars_in )
{
  SimpleVector & vars  = dynamic_cast<SimpleVector &>(vars_in);
  SimpleVector & x = dynamic_cast<SimpleVector &>(x_in);
  SimpleVector & y = dynamic_cast<SimpleVector &>(y_in);
  SimpleVector & z = dynamic_cast<SimpleVector &>(z_in);

  memcpy( &x[0], &vars[0], nx * sizeof( double ) );
  if ( my > 0 ) memcpy( &y[0], &vars[nx],      my * sizeof( double ) );
  if ( mz > 0 ) memcpy( &z[0], &vars[nx + my], mz * sizeof( double ) );
}


Data         * 
QpGenSparseSeq::makeData( double    c_[],
			     int    krowQ[],  int  jcolQ[],  double dQ[],
			     double  xlow_[],  char ixlow_[],
			     double  xupp_[],  char ixupp_[],
			     int    krowA[],
			     int    jcolA[],  double dA[],
			     double    b_[],
			     int    krowC[],
			     int    jcolC[],  double dC[],
			     double  clow_[],  char  iclow_[],
			     double  cupp_[],  char  icupp_[] )
{
  // Objective funcition
  SimpleVectorHandle c( new SimpleVector( c_, nx ) );

  nnzQ = krowQ[nx];
  SparseSymMatrixHandle Q( new SparseSymMatrix( nx, nnzQ,
						    krowQ, jcolQ, dQ ) );

  // Bounds on variables
  SimpleVectorHandle xlow( new SimpleVector( xlow_, nx ) );
  SimpleVectorHandle ixlow( new SimpleVector( nx ) );
  ixlow->copyFromArray( ixlow_ );

  SimpleVectorHandle xupp( new SimpleVector( xupp_, nx ) );
  SimpleVectorHandle ixupp( new SimpleVector( nx ) );
  ixupp->copyFromArray( ixupp_ );

  // Equality constraints
  nnzA = 0;
  if( my > 0 )    nnzA = krowA[my];
  SparseGenMatrixHandle A( new SparseGenMatrix( my, nx,
						    nnzA, krowA, jcolA, dA ) );

  SimpleVectorHandle b( new SimpleVector( b_, my ) );

  // Inequality constraints
  nnzC = 0;
  if( mz > 0 )    nnzC = krowC[mz];
  SparseGenMatrixHandle C( new SparseGenMatrix( mz, nx,
						    nnzC, krowC, jcolC, dC ) );

  SimpleVectorHandle clow( new SimpleVector( clow_, mz ) );
  SimpleVectorHandle iclow( new SimpleVector( mz ) );
  iclow->copyFromArray( iclow_ );

  SimpleVectorHandle cupp( new SimpleVector( cupp_, mz ) );
  SimpleVectorHandle icupp( new SimpleVector( mz ) );
  icupp->copyFromArray( icupp_ );

  QpGenData * 
    data = new QpGenData( SparseLinearAlgebraPackage::soleInstance(),
			  c, Q, xlow, ixlow, xupp, ixupp,
			  A, b,
			  C, clow, iclow, cupp, icupp );

  return data;
}

Data   *
QpGenSparseSeq::
copyDataFromSparseTriple( double c[],
			  int irowQ[], int lenQ,  int jcolQ[],  double dQ[],
			  double xlow[],  char ixlow[],
			  double xupp[],  char ixupp[],
			  int irowA[], int lenA,  int jcolA[],  double dA[],
			  double   bA[],
			  int irowC[],  int lenC,  int jcolC[], double dC[],
			  double clow[], char iclow[],
			  double cupp[], char icupp[] )
{
  QpGenData * prob =
    (QpGenData *) new QpGenData( la, nx, my, mz, nnzQ, nnzA, nnzC );  
  int info;

  assert( lenQ <= nnzQ );
  assert( lenA <= nnzA );
  assert( lenC <= nnzC );

  prob->g->copyFromArray( c );
  prob->Q->putSparseTriple( irowQ, lenQ, jcolQ, dQ, info );

  prob-> blx->copyFromArray( xlow );
  prob->ixlow->copyFromArray( ixlow );

  prob-> bux->copyFromArray( xupp );
  prob->ixupp->copyFromArray( ixupp );
  
  prob->A->putSparseTriple( irowA, lenA, jcolA, dA, info );
  prob->bA->copyFromArray( bA );
  
  prob->C->putSparseTriple( irowC, lenC, jcolC, dC, info );

  prob->bl   ->copyFromArray(  clow );
  prob->iclow->copyFromArray( iclow );
  
  prob->bu   ->copyFromArray(  cupp );
  prob->icupp->copyFromArray( icupp );

  return prob;
}

void QpGenSparseSeq::makeRandomData( QpGenData *& data, QpGenVars *& soln )
{
  data =
    new QpGenData( la, nx, my, mz, nnzQ, nnzA, nnzC );

  OoqpVectorHandle x( la->newVector( nx ) );
  OoqpVectorHandle y( la->newVector( my ) );
  OoqpVectorHandle z( la->newVector( mz ) );
  OoqpVectorHandle s( la->newVector( mz ) );

  data->datarandom( *x, *y, *z, *s );

  soln = (QpGenVars * ) this->makeVariables( data );

  soln->x->copyFrom( *x );
  soln->y->copyFrom( *y );
  soln->z->copyFrom( *z );
  soln->s->copyFrom( *s );
}

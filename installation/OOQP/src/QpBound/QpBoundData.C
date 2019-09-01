/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundData.h"
#include "QpBound.h"

#include <cassert>
#include <memory>
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;
#include <cmath>
#include <cstring>
#include "DoubleMatrix.h"
#include "OoqpVector.h"
#include "SimpleVector.h"
#include "SimpleVectorHandle.h"
#include "QpBoundVars.h"
#include "MpsReader.h"

// Needed for QPData::dataRandom
#include "DenseGenMatrix.h"
#include "DenseSymMatrix.h"

QpBoundData::QpBoundData( QpBound * f, SymMatrix * Q, int n_in )
{
  factory = f;

  nx  = n_in;
  alpha       = 0;
  SpReferTo( mQ, Q );
  c           = OoqpVectorHandle( factory->newPrimalVector() );
  lower       = OoqpVectorHandle( factory->newPrimalVector() );
  upper       = OoqpVectorHandle( factory->newPrimalVector() );
  index_lower = OoqpVectorHandle( factory->newPrimalVector() );
  index_upper = OoqpVectorHandle( factory->newPrimalVector() );
  nlower = -1;
  nupper = -1;
}

double QpBoundData::objectiveValue(Variables * vars_in) 
{
  QpBoundVars * vars = (QpBoundVars *) vars_in;

  OoqpVectorHandle temp = OoqpVectorHandle( factory->newPrimalVector() );
  temp->copyFrom( *c );
  
  mQ->mult( 1.0, *temp, 0.5, *vars->x );
  
  return temp->dotProductWith( *vars->x );
  
}

QpBoundData::QpBoundData( QpBound * f, double alpha_,
			  OoqpVector * c_in, SymMatrix * Q,
			  OoqpVector * l_in, OoqpVector *index_lower_in,
			  OoqpVector * u_in, OoqpVector *index_upper_in)
{
  alpha   = alpha_;
  factory = f;
  SpReferTo( c, c_in );
  nx    = c->length();
  SpReferTo( mQ, Q );
  SpReferTo( lower, l_in );
  SpReferTo( upper, u_in );

  SpReferTo( index_lower, index_lower_in );
  SpReferTo( index_upper, index_upper_in );
  nlower = index_lower->numberOfNonzeros();
  nupper = index_upper->numberOfNonzeros();
}

QpBoundData::~QpBoundData()
{
}

void QpBoundData::putQIntoAt( SymMatrix& M, int row, int col )
{
  M.symAtPutSubmatrix( row, col, *mQ, 0, 0, nx, nx );
}

void QpBoundData::putQIntoAt( GenMatrix& M, int row, int col )
{
  M.atPutSubmatrix( row, col, *mQ, 0, 0, nx, nx );
}

void QpBoundData::Qmult( double b, OoqpVector& y,
						 double a, OoqpVector& x )
{
  mQ->mult( b, y, a, x );
}

int QpBoundData::getN()
{
  return nx;
}

int QpBoundData::getNlower()
{
  if( nlower < 0 ) nlower = index_lower->numberOfNonzeros();
  return nlower;
}

int QpBoundData::getNupper()
{
  if( nupper < 0 ) nupper = index_upper->numberOfNonzeros();
  return nupper;
}

void QpBoundData::getg( OoqpVector& ccopy )
{
  ccopy.copyFrom( *c );
}

void QpBoundData::getl( OoqpVector& lcopy )
{
  lcopy.copyFrom( *lower );
}

void QpBoundData::getu( OoqpVector& ucopy )
{
  ucopy.copyFrom( *upper );
}

double QpBoundData::datanorm()
{
  double norm          = mQ->abmaxnorm();
  double componentNorm = c->infnorm();
  if( componentNorm > norm ) norm = componentNorm;
  
  assert( lower->matchesNonZeroPattern( *index_lower ) );
  componentNorm = lower->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  assert( upper->matchesNonZeroPattern( *index_upper ) );
  componentNorm = upper->infnorm();
  if( componentNorm > norm ) norm = componentNorm;
  return norm;
}

void QpBoundData::datarandom()
{
  double drand(double *), ix, temp;
  int i;

  ix = 89917682.0;
  SimpleVectorHandle psw( new SimpleVector (nx) );
  SimpleVector & sw = *psw;

  DenseSymMatrixHandle Qtemp( new DenseSymMatrix(nx) );
  Qtemp->randomizePSD( &ix );
  mQ->symAtPutSubmatrix( 0, 0, *Qtemp, 0, 0, nx, nx );
 
  SimpleVectorHandle pslower( new SimpleVector(nx) );
  SimpleVector & slower = *pslower;

  SimpleVectorHandle psupper( new SimpleVector(nx) );
  SimpleVector & supper = *psupper;

  SimpleVectorHandle psindex_lower( new SimpleVector(nx) );
  SimpleVector & sindex_lower = *psindex_lower;

  SimpleVectorHandle psindex_upper( new SimpleVector(nx) );
  SimpleVector & sindex_upper = *psindex_upper;

  slower.setToZero();
  supper.setToZero();
  sindex_lower.setToZero();
  sindex_upper.setToZero();

  for(i=0; i<nx; i++) {

    if(drand(&ix) < 0.2) {
      // make a lower bound
      sindex_lower[i] = 1.0;
      slower[i] = (drand(&ix) - 0.5) * 3.0;
    } else 
      sindex_lower[i] = 0.0;

    if(drand(&ix) < 0.2) {
      // make an upper bound
      sindex_upper[i] = 1.0;
      if(sindex_lower[i] == 0.0) 
	supper[i] = (drand(&ix) - 0.5) * 3.0;
      else
	supper[i] = slower[i] + drand(&ix) * 10.0;
    }

  }
  lower      ->copyFrom( slower );
  upper      ->copyFrom( supper );
  index_lower->copyFrom( sindex_lower );
  index_upper->copyFrom( sindex_upper );

  // now choose the elements of the optimal sw, s, lam1, lam2

  for(i=0; i < nx; i++) {
    if (sindex_lower[i] == 0.0 && sindex_upper[i] == 0.0) {
      sw[i] = (drand(&ix) - 0.5) * 3.0;
    } else if (sindex_lower[i] != 0.0 && sindex_upper[i] == 0.0) {
      if(drand(&ix) < 0.33) 
	// stick to the lower bound
	sw[i] = slower[i];
      else 
	// make the lower bound inactive
	sw[i] = slower[i] + drand(&ix) * 10.0;
    } else if (sindex_lower[i] == 0.0 && sindex_upper[i] != 0.0) {
      if(drand(&ix) < 0.33) 
	// stick to the upper bound
	sw[i] = supper[i];
      else 
	// make the upper bound inactive
	sw[i] = supper[i] - drand(&ix) * 10.0;
    } else {
      temp = drand(&ix);
      if(temp < 0.33) 
	// stick to the lower bound
	sw[i] = slower[i];
      else if (temp > 0.66)
	// stick to the upper bound
	sw[i] = supper[i];
      else
	// somewhere in the middle
	sw[i] = slower[i] + drand(&ix)* ( supper[i] - slower[i]);
    }
  }

  // choose c to make it all consistent

  // c <-- -Qw 
  OoqpVectorHandle w = OoqpVectorHandle( factory->newPrimalVector() );
  w->copyFrom( sw );
  mQ->mult( 0.0, *c, -1.0, *w );
  {  
    SimpleVectorHandle psc( new SimpleVector(nx) );
    SimpleVector & sc = *psc;
    sc.copyFrom(*c);
    for(i=0; i<nx; i++) {
      if(sindex_lower[i] != 0.0 && sw[i] == slower[i])
	sc[i] += drand(&ix);
      if(sindex_upper[i] != 0.0 && sw[i] == supper[i])
	sc[i] -= drand(&ix);
    }
    c->copyFrom(sc);
  }  
  // calculate the number of actual lower and upper bounds
  for(i=0, nlower=0, nupper=0; i<nx; i++) {
    if(sindex_lower[i] != 0.0) nlower++;
    if(sindex_upper[i] != 0.0) nupper++;
  }

}


void QpBoundData::print()
{
  cout << "********** Q ******* \n";
  mQ->writeToStream( cout );

  cout << "*********** c **********\n";
  c->writefToStream( cout, "%{value}" );

  cout << "*********** lower **********\n";
  lower->writefSomeToStream( cout, "component %{index} = %{value}",
			     *index_lower );

  cout << "*********** upper **********\n";
  upper->writefSomeToStream( cout, "component %{index} = %{value}",
			     *index_upper );
}

void QpBoundData::datainput(MpsReader * reader, int& ierr )
{
  reader->readQpBound( *c, *mQ,
		       *lower, *index_lower, *upper, *index_upper,
		       ierr );
}

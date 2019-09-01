/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpExampleData.h"

#include <cassert>
#include <iostream>
#include "DoubleMatrix.h"
#include "OoqpVector.h"
#include "LinearAlgebraPackage.h"

// Needed for QPData::dataRandom
#include "DenseGenMatrix.h"
#include "DenseSymMatrix.h"
#include "SimpleVector.h"

QpExampleData::QpExampleData(LinearAlgebraPackage * la_in,
			     int nx_in, int my_in, int mz_in )
  : la( la_in ), nx( nx_in ), my( my_in ), mz( mz_in )
{
  Q = la->newSymMatrix( nx, nx * nx );
  A = la->newGenMatrix( my, nx, my * nx );
  C = la->newGenMatrix( mz, nx, mz * nx );

  c = la->newVector(nx);
  b = la->newVector(my);
  d = la->newVector(mz);
}

QpExampleData::~QpExampleData()
{
  IotrRelease( &Q );
  IotrRelease( &A );
  IotrRelease( &C );
  IotrRelease( &c );
  IotrRelease( &b );
  IotrRelease( &d );
}


double QpExampleData::datanorm()
{
  int i;
  double norm = Q->abmaxnorm();
  double componentNorm;

  componentNorm = A->abmaxnorm();
  if ( componentNorm > norm ) norm = componentNorm;
  componentNorm = C->abmaxnorm();
  if ( componentNorm > norm ) norm = componentNorm;

  componentNorm = b->infnorm();
  if ( componentNorm > norm ) norm = componentNorm;

  componentNorm = c->infnorm();
  if ( componentNorm > norm ) norm = componentNorm;

  componentNorm = d->infnorm();
  if ( componentNorm > norm ) norm = componentNorm;
  
  return norm;
}

void QpExampleData::print()
{
  int i;

  cout << "********** Q ******* \n";
  cout << Q << endl;

  cout << "********** A ******* \n";
  cout << A << endl;

  cout << "********** C ******* \n";
  cout << C << endl;

  cout << "*********** c **********\n";
  cout << c << endl;

  cout << "*********** b **********\n";
  cout << b << endl;

  cout << "*********** d **********\n";
  cout << d << endl;
}


void QpExampleData::datarandom()
{
  double drand(double *), ix; //, *x, *y, *z;
  int i;

  ix          = 8917682.0;
  OoqpVector * x = la->newVector( nx );
  OoqpVector * s = la->newVector( mz );
  OoqpVector * y = la->newVector( my );
  OoqpVector * z = la->newVector( mz );

  {
    DenseSymMatrix * Qtemp = new DenseSymMatrix( nx );
    Qtemp->randomizePSD( &ix );
    Q->symAtPutSubmatrix( 0, 0, *Qtemp, 0, 0, nx, nx );
    IotrRelease( &Qtemp );
  }
  {
    DenseGenMatrix * Atemp = new DenseGenMatrix( my, nx );
    Atemp->randomize( -1.5, 1.5, &ix );
    A->atPutSubmatrix(0, 0, *Atemp, 0, 0, my, nx );
    IotrRelease( &Atemp );
  }
  {
    DenseGenMatrix * Ctemp = new DenseGenMatrix ( mz, nx );
    Ctemp->randomize( -2.5, 2.5, &ix );
    C->atPutSubmatrix( 0, 0, *Ctemp, 0, 0, mz, nx );
    IotrRelease( &Ctemp );
  }

  // now choose the elements of the optimal x, s, y, z
  {
    SimpleVector * pZtemp = new SimpleVector( mz );
    SimpleVector & ztemp  = *pZtemp;
    SimpleVector * pStemp = new SimpleVector( mz );
    SimpleVector & stemp  = *pStemp;
    
    for(i=0; i<mz; i+=2) {
      ztemp[i] = 10.0 * drand(&ix);
      stemp[i] = 0.0;
    }
    for(i=1; i<mz; i+=2) {
      ztemp[i] = 0.0;
      stemp[i] = 6.0 * drand(&ix);
    }
    s->copyFrom( stemp );
    z->copyFrom( ztemp );

    IotrRelease( &pZtemp ); IotrRelease( &pStemp );
  }
  x->randomize( -1.5, 1.5, &ix );
  y->randomize( -2.5, 2.5, &ix );

  // choose c, b, and d to make it all consistent

  // Qw - A' y - C' z = -c
  Q->mult( 0.0, *c, -1.0, *x );
  A->transMult( 1.0, *c, 1.0, *y );
  C->transMult( 1.0, *c, 1.0, *z  );

  // Aw = b
  A->mult( 0.0, *b, 1.0, *x );
  // Cw - s = d
  d->copyFrom( *s );
  C->mult( -1.0, *d, 1.0, *x );

  IotrRelease( &x ); IotrRelease( &s );
  IotrRelease( &y ); IotrRelease( &z );
}

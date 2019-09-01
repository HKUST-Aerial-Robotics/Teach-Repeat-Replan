/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenData.h"
#include "QpGenVars.h"
#include "DoubleMatrix.h"
#include "OoqpVector.h"
#include <cmath>

#include "SimpleVector.h"
#include "LinearAlgebraPackage.h"
#include "MpsReader.h"


QpGenData::QpGenData(LinearAlgebraPackage * la_,
		     int nx_, int my_, int mz_,
		     int nnzQ, int nnzA, int nnzC)
{
  la = la_;

  nx = nx_;
  my = my_;
  mz = mz_;

  Q = SymMatrixHandle( la->newSymMatrix( nx,     nnzQ ) );
  A = GenMatrixHandle( la->newGenMatrix( my, nx, nnzA ) );
  C = GenMatrixHandle( la->newGenMatrix( mz, nx, nnzC ) );

  g     = OoqpVectorHandle( la->newVector( nx ) );
  blx   = OoqpVectorHandle( la->newVector( nx ) );
  ixlow = OoqpVectorHandle( la->newVector( nx ) );
  bux   = OoqpVectorHandle( la->newVector( nx ) );
  ixupp = OoqpVectorHandle( la->newVector( nx ) );

  bA    = OoqpVectorHandle( la->newVector( my ) );
  
  bl    = OoqpVectorHandle( la->newVector( mz ) );
  iclow = OoqpVectorHandle( la->newVector( mz ) );
  bu    = OoqpVectorHandle( la->newVector( mz ) );
  icupp = OoqpVectorHandle( la->newVector( mz ) );
  sc    = OoqpVectorHandle( la->newVector( nx  ) );
}

QpGenData::QpGenData( LinearAlgebraPackage * la_in,
		      OoqpVector * c_in, SymMatrix * Q_in,
		      OoqpVector * xlow_in, OoqpVector * ixlow_in,
		      OoqpVector * xupp_in, OoqpVector * ixupp_in,
		      GenMatrix  * A_in, OoqpVector * bA_in,
		      GenMatrix  * C_in,
		      OoqpVector * clow_in, OoqpVector * iclow_in,
		      OoqpVector * cupp_in, OoqpVector * icupp_in )
{
  SpReferTo( g,     c_in  );
  SpReferTo( bA,    bA_in );
  SpReferTo( blx,   xlow_in  );
  SpReferTo( ixlow, ixlow_in );
  SpReferTo( bux,   xupp_in  );
  SpReferTo( ixupp, ixupp_in );
  SpReferTo( bl,    clow_in  );
  SpReferTo( iclow, iclow_in );
  SpReferTo( bu,    cupp_in  );
  SpReferTo( icupp, icupp_in );

  int dummy;
  la = la_in;

  nx = g->length();
  SpReferTo( Q, Q_in );

  SpReferTo( A, A_in );
  A->getSize( my, dummy );
  
  SpReferTo( C, C_in );
  C->getSize( mz, dummy );
}


void QpGenData::Qmult( double beta,  OoqpVector& y,
		       double alpha, OoqpVector& x )
{
  Q->mult( beta, y, alpha, x );
}

void QpGenData::Amult( double beta,  OoqpVector& y,
		       double alpha, OoqpVector& x)
{
  A->mult( beta, y, alpha, x );
}

void QpGenData::Cmult( double beta,  OoqpVector& y,
		       double alpha, OoqpVector& x )
{
  C->mult( beta, y, alpha, x );
}

void QpGenData::ATransmult( double beta,  OoqpVector& y,
			    double alpha, OoqpVector& x )
{
  A->transMult( beta, y, alpha, x );
}

void QpGenData::CTransmult( double beta,  OoqpVector& y,
			    double alpha, OoqpVector& x )
{
  C->transMult( beta, y, alpha, x );
}

void QpGenData::getg( OoqpVector& myG )
{
  myG.copyFrom( *g );
}

void QpGenData::getbA( OoqpVector& bout )
{
  bout.copyFrom( *bA );
}

double QpGenData::datanorm()
{
  double norm = 0.0;
  double componentNorm;

  componentNorm = g->infnorm();
  if( componentNorm > norm ) norm = componentNorm;
  
  componentNorm = Q->abmaxnorm();
  if( componentNorm > norm ) norm = componentNorm;

  componentNorm = bA->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  componentNorm = A->abmaxnorm();
  if( componentNorm > norm ) norm = componentNorm;

  componentNorm = C->abmaxnorm();
  if( componentNorm > norm ) norm = componentNorm;

  assert( blx->matchesNonZeroPattern( *ixlow ) );
  componentNorm = blx->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  assert( bux->matchesNonZeroPattern( *ixupp ) );
  componentNorm = bux->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  assert( bl->matchesNonZeroPattern( *iclow ) );
  componentNorm = bl->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  assert( bu->matchesNonZeroPattern( *icupp ) );
  componentNorm = bu->infnorm();
  if( componentNorm > norm ) norm = componentNorm;

  return norm;
}

void QpGenData::datainput( MpsReader * reader, int scale, int& iErr )
{
    reader->readQpGen( *g, *Q, *blx, *ixlow, *bux, *ixupp,
		     *A, *bA,
		     *C, *bl, *iclow, *bu, *icupp, iErr );

    if( scale ){
        // Create the scaling vector
        this->createScaleFromQ();

        //Scale the variables
        this->scaleQ();
        this->scaleA();
        this->scaleC();
        this->scaleg();
        this->scalexlow();
        this->scalexupp();
        }

    /* If objective sense is "MAX", flip the C and Q matrices */
    if( !reader->doMinimize() ) {
        this->flipg();
        this->flipQ();
        }  
}

void 
QpGenData::randomlyChooseBoundedVariables( OoqpVector& x,
					   OoqpVector& dualx,
					   OoqpVector& xlow_,
					   OoqpVector& ixlow_,
					   OoqpVector& xupp_,
					   OoqpVector& ixupp_,
					   double * ix,
					   double percentLowerOnly,
					   double percentUpperOnly,
					   double percentBound )
{
  int i;
  double drand( double * );
  // Initialize the upper and lower bounds on x
  int n = x.length();

  
  double * sxlow  = new double[n];
  double * sixlow = new double[n];
  double * sxupp  = new double[n];
  double * sixupp = new double[n];
  double * sx     = new double[n];
  double * sdualx = new double[n];
  
  for( i = 0; i < n; i++ ) {
    double r = drand(ix);
    //cout << " r: " << r << "   ";
	
    if( r < percentLowerOnly ) {
      //cout << "i= " << i << " Lower bound " << endl;
      sixlow[i]  = 1.0;
      sxlow[i]    = (drand(ix) - 0.5) * 3.0;
      sixupp[i]  = 0.0;
      sxupp[i]    = 0.0;
    } else if ( r < percentLowerOnly + percentUpperOnly ) { 
      //cout << "i= " << i << " Upper bound " << endl;
      sixlow[i]  = 0.0;
      sxlow[i]    = 0.0;
      sixupp[i]  = 1.0;
      sxupp[i]    = (drand(ix) - 0.5) * 3.0;
    } else if ( r < percentLowerOnly + percentUpperOnly
		+ percentBound ) {
      //cout << "i= " << i << " Two-sided bound " << endl;
      sixlow[i]  = 1.0;
      sxlow[i]    = (drand(ix) - 0.5) * 3.0;
      sixupp[i]  = 1.0;
      sxupp[i]    = sxlow[i] +  drand(ix) * 10.0;
    } else {
      // it is free
      //cout << "i= " << i << " Free " << endl;
      sixlow[i]  = 0.0;
      sxlow[i]    = 0.0;
      sixupp[i]  = 0.0;
      sxupp[i]    = 0.0;
    }
  }
  xlow_. copyFromArray( sxlow );
  ixlow_.copyFromArray( sixlow );
  xupp_. copyFromArray( sxupp );
  ixupp_.copyFromArray( sixupp );

  for ( i = 0; i < n; i++ ) {
    if( sixlow[i] == 0.0 && sixupp[i] == 0.0 ) {
      // x[i] not bounded
      sx[i] = 20.0 * drand(ix) - 10.0;
      sdualx[i] = 0.0;
    } else if ( sixlow[i] != 0.0 && sixupp[i] != 0.0 ) {
      // x[i] is bounded above and below
      double r = drand( ix );
      if( r < 0.33 ) {
	// x[i] is on its lower bound
	sx[i]     =  sxlow[i];
	sdualx[i] =  10.0 * drand( ix );
      } else if ( r > .66 ) {
	// x[i] is on its upper bound
	sx[i]     =  sxupp[i];
	sdualx[i] = -10.0 * drand( ix );
      } else {
	// x[i] is somewhere in between
	double theta = .99 * drand( ix ) + .005;
	sx[i] = (1 - theta) * sxlow[i] + theta * sxupp[i];
	sdualx[i] = 0.0;
      }
    } else if ( sixlow[i] != 0.0 ) {
      // x[i] is only bounded below
      if( drand( ix ) < .33 ) {
	// x[i] is on its lower bound
	sx[i]     =  sxlow[i];
	sdualx[i] =  10.0 * drand( ix );
      } else {
	// x[i] is somewhere above its lower bound
	sx[i]     = sxlow[i] + 0.005 + 10.0 * drand(ix);
	sdualx[i] = 0.0;
      }
    } else { // x[i] only has an upper bound
      if( drand(ix) > .66 ) {
	// x[i] is on its upper bound
	sx[i]     =  sxupp[i];
	sdualx[i] = -10.0 * drand( ix );
      } else {
	// x[i] is somewhere below its upper bound
	sx[i]     =  sxupp[i] - 0.005 - 10.0 * drand(ix);
	sdualx[i] = 0.0;
      }
    } // end else x[i] only has an upper bound
  } // end for ( i = 0; i < n; i++ )
  x.copyFromArray( sx );
  dualx.copyFromArray( sdualx );

  delete [] sxlow;
  delete [] sxupp;
  delete [] sixlow;
  delete [] sixupp;
  delete [] sx;
  delete [] sdualx;
}

void QpGenData::print()
{
  cout << "begin Q\n";
  Q->writeToStream( cout );
  cout << "end Q\n";
  cout << "begin c\n";
  g->writeToStream( cout );
  cout << "end c\n";

  cout << "begin xlow\n";
  blx->writeToStream( cout );
  cout << "end xlow\n";
  cout << "begin ixlow\n";
  ixlow->writeToStream( cout );
  cout << "end ixlow\n";

  cout << "begin xupp\n";
  bux->writeToStream( cout );
  cout << "end xupp\n";  
  cout << "begin ixupp\n";
  ixupp->writeToStream( cout );
  cout << "end ixupp\n";
  cout << "begin A\n";

  A->writeToStream( cout );
  cout << "end A\n";
  cout << "begin b\n";
  bA->writeToStream( cout );
  cout << "end b\n";
  cout << "begin C\n";
  C->writeToStream( cout );
  cout << "end C\n";
  
  cout << "begin clow\n";
  bl->writeToStream( cout );
  cout << "end clow\n";
  cout << "begin iclow\n";
  iclow->writeToStream( cout );
  cout << "end iclow\n";

  cout << "begin cupp\n";
  bu->writeToStream( cout );
  cout << "end cupp\n";
  cout << "begin icupp\n";
  icupp->writeToStream( cout );
  cout << "end icupp\n";

}

#include <fstream>

void QpGenData::datarandom( OoqpVector & x, OoqpVector & y,
			    OoqpVector & z, OoqpVector & s )
{
  double drand( double * );
  double ix = 3074.20374;
  
  OoqpVectorHandle xdual(la->newVector( nx ));
  this->randomlyChooseBoundedVariables( x, *xdual,
  					*blx, *ixlow, *bux, *ixupp,
  					&ix, .25, .25, .25 );

  {
//      ofstream x_vec( "x" );
//      x->writeToStream( x_vec );
//      ofstream eta_vec( "xdual" );
//      eta_vec.precision(16);
//      xdual->writeToStream( eta_vec );
//      ofstream blx_vec( "blx" );
//      blx->writeToStream( blx_vec );
//      ofstream bux_vec( "bux" );
//      bux->writeToStream( bux_vec );
  } 

  OoqpVectorHandle sprime(la->newVector( mz ));
  this->randomlyChooseBoundedVariables( *sprime, z,
  					*bl, *iclow, *bu, *icupp,
  					&ix, .25, .25, .5 );
  
  {
    //      	ofstream z_vec( "z" );
    //      	z_vec << z << endl;
  }
  {
    Q->randomizePSD( &ix );
//      ofstream Q_mat( "Q" );
//      Q_mat << Q << endl;
  }
  { 
    A->randomize( -10.0, 10.0, &ix );
    //  	ofstream A_mat( "A" );
    //  	A_mat << A << endl;
  }
  {
    C->randomize( -10.0, 10.0, &ix );
    //  	ofstream C_mat( "C" );
    //  	C_mat << C << endl;
  }

  y.randomize( -10.0, 10.0, &ix );
  {
    //  	ofstream y_vec( "y" );
    //  	y_vec << y << endl;
  }
  // g = - Q x + A\T y + C\T z + xdual 
  g->copyFrom( *xdual );
  Q->mult( 1.0, *g, -1.0, x );
  A->transMult( 1.0, *g, 1.0, y );
  C->transMult( 1.0, *g, 1.0, z );
  // bA = A x
  A->mult( 0.0, *bA, 1.0, x );
  {
    //  	ofstream bA_vec( "bA" );
    //  	bA_vec << bA << endl;
  }
  // Have a randomly generated sprime.
  // C x - s = 0. Let q + sprime = s, i.e. q = s - sprime.
  // Compute s and temporarily store in q
  C->mult( 0.0, s, 1.0, x );
  // Now compute the real q = s - sprime
  OoqpVectorHandle q(la->newVector( mz ));
  q->copyFrom( s );
  q->axpy( -1.0, *sprime );
  // Adjust bl and bu appropriately
  bl->axpy( 1.0, *q );
  bu->axpy( 1.0, *q );
  
  bl->selectNonZeros( *iclow );
  bu->selectNonZeros( *icupp );
  
  {
    //   	ofstream bl_vec( "bl" );
    //      	bl_vec << bl << endl;
    //      	ofstream bu_vec( "bu" );
    //      	bu_vec << bu << endl;
  }
}


void QpGenData::putQIntoAt( GenMatrix& M, int row, int col )
{
  M.atPutSubmatrix( row, col, *Q, 0, 0, nx, nx );
}

void QpGenData::putQIntoAt( SymMatrix& M, int row, int col )
{
  M.symAtPutSubmatrix( row, col, *Q, 0, 0, nx, nx );
}

void QpGenData::putAIntoAt( GenMatrix& M, int row, int col )
{
  M.atPutSubmatrix( row, col, *A, 0, 0, my, nx );
}

void QpGenData::putAIntoAt( SymMatrix& M, int row, int col )
{
  M.symAtPutSubmatrix( row, col, *A, 0, 0, my, nx );
}

void QpGenData::putCIntoAt( GenMatrix& M, int row, int col )
{
  M.atPutSubmatrix( row, col, *C, 0, 0, mz, nx );
}

void QpGenData::putCIntoAt( SymMatrix& M, int row, int col )
{
  M.symAtPutSubmatrix( row, col, *C, 0, 0, mz, nx );
}

void QpGenData::getDiagonalOfQ( OoqpVector& dq )
{
  Q->fromGetDiagonal(0, dq);
}

double QpGenData::objectiveValue( QpGenVars * vars )
{
  OoqpVectorHandle temp( la->newVector( nx ) );
  this->getg( *temp );
  this->Qmult( 1.0, *temp, 0.5, *vars->x );

  return temp->dotProductWith( *vars->x );
}

void QpGenData::createScaleFromQ()
{
  // Stuff the diagonal elements of Q into the vector "sc"
  this->getDiagonalOfQ( *sc);

  // Modifying scVector is equivalent to modifying sc
  SimpleVector & scVector = dynamic_cast<SimpleVector &>(*sc);

  int scLength = scVector.length();

  for( int i = 0; i < scLength; i++){
    if( scVector[i] > 1)
        scVector[i] = 1.0/sqrt( scVector[i]);
    else
        scVector[i] = 1.0;
    }
}

void QpGenData::scaleQ()
{
    Q->SymmetricScale( *sc);

}


void QpGenData::scaleA()
{
    A->ColumnScale( *sc);

}

void QpGenData::scaleC()
{

    C->ColumnScale( *sc);

}

void QpGenData::scaleg()
{
  SimpleVector & scVector = dynamic_cast<SimpleVector &>(*sc);

  assert ( scVector.length() == g->length());

  // D * g
  g->componentMult( scVector);
}

void QpGenData::scalexupp()
{
  SimpleVector & scVector = dynamic_cast<SimpleVector &>(*sc);

  assert ( scVector.length() == bux->length());

  // inverse(D) * bux
  bux->componentDiv( scVector);

}


void QpGenData::scalexlow()
{
  SimpleVector & scVector = dynamic_cast<SimpleVector &>(*sc);

  assert ( scVector.length() == blx->length());

  // inverse(D) * blx
  blx->componentDiv( scVector);

}

void QpGenData::flipg()
{
  // Multiply C matrix by -1
  g->scalarMult( -1.0);
}

void QpGenData::flipQ()
{
  // Multiply Q matrix by -1
  Q->scalarMult( -1.0);
}


QpGenData::~QpGenData()
{
}







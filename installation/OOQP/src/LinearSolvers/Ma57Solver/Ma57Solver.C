/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "Ma57Solver.h"
#include "SparseStorage.h"
#include "SparseSymMatrix.h"
#include "SimpleVector.h"
#include "SimpleVectorHandle.h"

#ifdef HAVE_GETRUSAGE
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

extern int gOoqpPrintLevel;

#endif


Ma57Solver::Ma57Solver( SparseSymMatrix * sgm ) :
  icntl(), info(), cntl(), rinfo(), lkeep(0), keep(0),
  lifact(0), ifact(0), lfact(0), freshFactor(0)
{
  irowM = 0;
  jcolM = 0;
  fact  = 0;
  ifact = 0;
  keep  = 0;

  ipessimism = 1.2;
  rpessimism = 1.2;

  ma57id_( cntl, icntl );
  icntl[1] = -1; // don't print warning messages
  icntl[8] = 10; // up to 10 steps of iterative refinement

  // set initial value of "Treat As Zero" parameter
  kTreatAsZero = 1.e-10;      this->setTreatAsZero();

  // set initial value of Threshold parameter
  kThresholdPivoting = 1.e-8; this->setThresholdPivoting();

  // set the largest value of ThresholdPivoting parameter we are
  // willing to tolerate.
  kThresholdPivotingMax = 1.e-2;

  // set the increase factor for ThresholdPivoting parameter
  kThresholdPivotingFactor = 10.0;

  // set the required precision for each linear system solve
  kPrecision = 1.e-7;

  mStorage = SparseStorageHandle( sgm->getStorage() );
  n        = mStorage->n;
  M        =  mStorage->M;

  nnz = mStorage->krowM[n];
}

void Ma57Solver::firstCall()
{
  irowM = new int[nnz];
  jcolM = new int[nnz];

  int * krowM = mStorage->krowM;
  for( int i = 0; i < n; i++ ) {
    for( int k = krowM[i]; k < krowM[i+1]; k++ ) {
      irowM[k] = i + 1;
    }
  }
  for( int k = 0; k < nnz; k++ ) {
    jcolM[k] = mStorage->jcolM[k] + 1;
  }

  lkeep = ( nnz > n ) ? (5 * n + 2 *nnz + 42) : (6 * n + nnz + 42);
  keep = new int[lkeep];
  // Initialize keep to all zeros to keep some versions of MA57
  // from accessing uninitialized memory.
  memset(keep,0,lkeep*sizeof(int));

  int * iwork = new int[5 * n];
  ma57ad_( &n, &nnz, irowM, jcolM, &lkeep, keep, iwork, icntl,
	   info, rinfo );

  delete [] iwork;

  lfact = info[8];
  lfact = (int) (rpessimism * lfact);
  fact  = new double[lfact];
  lifact = info[9];
  lifact = (int) (ipessimism * lifact);
  ifact  = new int[lifact];
}  
void Ma57Solver::diagonalChanged( int /* idiag */, int /* extent */ )
{
  this->matrixChanged();
}

void Ma57Solver::matrixChanged()
{
  if( !keep ) this->firstCall();
  
  int * iwork = new int[n];

  int done = 0, tries = 0;;
  do {
#ifdef HAVE_GETRUSAGE
    rusage before;
    if( gOoqpPrintLevel >= 100 ) {
      getrusage( RUSAGE_SELF, &before );
    }
#endif

    ma57bd_( &n,       &nnz,    M,     fact,  &lfact,  ifact,
	     &lifact,  &lkeep,  keep,  iwork,  icntl,  cntl,
	     info,     rinfo );
#ifdef HAVE_GETRUSAGE
    rusage  after;
    if( gOoqpPrintLevel >= 100 ) {
      getrusage( RUSAGE_SELF, &after );
      cout << "For try " << tries + 1 
	   << " the factorization took " 
	   << (double) (after.ru_utime.tv_sec - before.ru_utime.tv_sec)
	+ (after.ru_utime.tv_usec - before.ru_utime.tv_usec) / 1000000.0
	   << " seconds.\n";
    }
#endif

    if( info[0] != 0 ) cout << "Factorization info: " << info[0] << endl;
    switch( info[0] ) {
    case 0: done = 1;     
      break;
    case -3: {
      int ic = 0;
      int lnfact = (int) (info[16] * rpessimism);
      double * newfact = new double[lnfact];
      ma57ed_( &n, &ic, keep, fact, &lfact, newfact, &lnfact,
	       ifact, &lifact, ifact, &lifact, info );
      delete [] fact;
      fact = newfact;
      lfact = lnfact;
      rpessimism *= 1.1;
      cout << "Resizing real part. pessimism = " << rpessimism << endl;
    }; break;
    case -4: {
      int ic = 1;
      int lnifact = (int) (info[17] * ipessimism);
      int * nifact = new int[ lnifact ];
      ma57ed_( &n, &ic, keep, fact, &lfact, fact, &lfact,
	       ifact, &lifact, nifact, &lnifact, info );
      delete [] ifact;
      ifact = nifact;
      lifact = lnifact;
      ipessimism *= 1.1;
      cout << "Resizing int part. pessimism = " << ipessimism << endl;
    }; break;
    default:
      if( info[0] >= 0 ) done = 1;
      assert( info[0] >= 0 );
    } // end switch      
    tries++;
  } while( !done );
  freshFactor = 1;
  
  delete [] iwork;
}

void Ma57Solver::solve( OoqpVector& rhs_in )
{
//    int job = 0; // Solve using A
//    int one = 1;
  
//    SimpleVectorHandle work( new SimpleVector(n) );
//    SimpleVector & rhs = dynamic_cast<SimpleVector &>(rhs_in);

//    double * drhs = rhs.elements();
//    double * dwork  = work->elements();

//    int * iwork = new int[n];

//    rusage before;
//    getrusage( RUSAGE_SELF, &before );

//    ma57cd_( &job,       &n,        
//  	   fact,       &lfact,    ifact,  &lifact,  
//  	   &one,       drhs,      &n,   
//  	   dwork,      &n,        iwork, 
//  	   icntl,      info );
	   

//    rusage after;
//    getrusage( RUSAGE_SELF, &after );
//    cout << "Solution with the factored matrix took "
//         << (double) (after.ru_utime.tv_sec - before.ru_utime.tv_sec)
//        + (after.ru_utime.tv_usec - before.ru_utime.tv_usec) / 1000000.0
//  	 << " seconds.\n";

//    delete [] iwork;

  int job = 0;
  if( freshFactor ) {
    icntl[8] = 1; // No iterative refinement
  } else {
    icntl[8] = 10; // Iterative refinement
  }    

  // MIKE: are these structure ever released??

  SimpleVectorHandle x( new SimpleVector(n) );
  SimpleVectorHandle resid( new SimpleVector(n) );
  SimpleVectorHandle work( new SimpleVector(5 * n) );
  SimpleVector & rhs = dynamic_cast<SimpleVector &>(rhs_in);

  double * drhs = rhs.elements();
  double * dx   = x->elements();
  double * dresid = resid->elements();
  double * dwork  = work->elements();

  int * iwork = new int[n];

#ifdef HAVE_GETRUSAGE
  rusage before;
  if( gOoqpPrintLevel >= 100 ) {
    getrusage( RUSAGE_SELF, &before );
  }
#endif

  int done = 0;
  int refactorizations = 0;
  int dontRefactor =  (kThresholdPivoting > kThresholdPivotingMax);

  while( !done && refactorizations < 10 ) {
    ma57dd_( &job,       &n,        &nnz,   M,        irowM,   jcolM,
	     fact,       &lfact,    ifact,  &lifact,  drhs,    dx,
	     dresid,      dwork,    iwork,  icntl,    cntl,    info,
	     rinfo );
    if( resid->infnorm() < kPrecision*( 1 + rhs.infnorm() ) ) {
      // resids are fine, use them
      done = 1;
    } else {
      // resids aren't good enough.
      if( freshFactor ) { // We weren't doing iterative refinement,
	// let's do so
	job = 2;
	icntl[8] = 10;
	// Mark this factorization as stale
	freshFactor = 0;
	// And grow more pessimistic about the next factorization
	if( kThresholdPivoting >= kThresholdPivotingMax ) {
	  // We have already refactored as with a high a pivtol as we
	  // are willing to use
	  dontRefactor = 1; 
	} else {
	  // refactor with a higher Threshold Pivoting parameter
	  kThresholdPivoting *= kThresholdPivotingFactor;
	  if( kThresholdPivoting > kThresholdPivotingMax ) 
	    kThresholdPivoting = kThresholdPivotingMax;
	  this->setThresholdPivoting();
	  cout << "Setting ThresholdPivoting parameter to " 
	       << kThresholdPivoting << " for future factorizations" << endl;
	}
      } else if ( dontRefactor ) {
	// We might have tried a refactor, but the pivtol is above our
	// limit.
	done = 1;
      } else {
	// Otherwise, we have already tried iterative refinement, and
	// have already increased the ThresholdPivoting parameter
	cout << "Refactoring with Threshold Pivoting parameter" 
	     << kThresholdPivoting << endl;
	this->matrixChanged();
	refactorizations++;
	// be optimistic about the next factorization
	job = 0;
	icntl[8] = 1;
      } // end else we hava already tried iterative refinement
    } // end else resids aren't good enough
  } // end while not done

#ifdef HAVE_GETRUSAGE
  rusage after;
  if( gOoqpPrintLevel >= 100 ) {
    getrusage( RUSAGE_SELF, &after );
    cout << "Solution with the factored matrix took "
	 << (double) (after.ru_utime.tv_sec - before.ru_utime.tv_sec)
      + (after.ru_utime.tv_usec - before.ru_utime.tv_usec) / 1000000.0
	 << " seconds.\n";
  }
#endif
  rhs.copyFrom( *x );

  delete [] iwork;
}


Ma57Solver::~Ma57Solver()
{
  delete [] irowM;
  delete [] jcolM;
  delete [] fact;
  delete [] ifact;
  delete [] keep;
}


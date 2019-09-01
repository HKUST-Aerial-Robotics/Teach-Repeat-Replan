/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <stdexcept>

#include "Ma27Solver.h"
#include "SparseStorage.h"
#include "SparseSymMatrix.h"
#include "SimpleVector.h"
#include "SimpleVectorHandle.h"

#ifdef HAVE_GETRUSAGE
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#endif

extern int gOoqpPrintLevel;
const double kInitTreatAsZero        = 1.0e-12;
const double kInitThresholdPivoting  = 1.0e-8;
const double kInitPrecision          = 1e-7;

/** the Threshold Pivoting parameter may need to be increased during
 * the algorithm if poor precision is obtained from the linear
 * solves.  kThresholdPivoting indicates the largest value we are
 * willing to tolerate.  */
const double   kThresholdPivotingMax = 1.e-2;

/** the factor in the range (1,inf) by which kThresholdPivoting is
 * increased when it is found to be inadequate.  */
const double   kThresholdPivotingFactor = 10.0; 


Ma27Solver::Ma27Solver( SparseSymMatrix * sgm ) :
  Ma27SolverBase( sgm->size(), sgm->numberOfNonZeros() )
{
  SpReferTo( mMat, sgm );
}

Ma27SolverBase::Ma27SolverBase( int n_in, int nnz_in ) :
  icntl(), info(), cntl(),
  precision(kInitPrecision), irowM(0), jcolM(0), fact(0),
  n(n_in), nnz(nnz_in), la(0), 
  ikeep(0), iw(0), liw(0), iw1(0), iw2(0), nsteps(0), maxfrt(0),
  w(0), ipessimism(1.2), rpessimism(1.2)
{
  ma27id_(icntl, cntl);
  // set initial value of "Treat As Zero" parameter
  this->setTreatAsZero( kInitTreatAsZero );

  // set initial value of Threshold parameter
  this->setThresholdPivoting( kInitThresholdPivoting );
  
  if( gOoqpPrintLevel < 100 ) {
    icntl[0] = 0;
    icntl[1] = 0;
  }
}

void Ma27SolverBase::firstCall()
{
  irowM = new int[nnz]();
  jcolM = new int[nnz]();

  this->getIndices( irowM, jcolM );

  // set array lengths as recommended in ma27 docs
  liw = (int)(1.3 * (2*nnz + 3*n + 1));
  iw = new int[liw]();
  iw1 = new int[2*n]();

  // define ikeep (which stores the pivot sequence)
  ikeep = new int[3*n]();

  // set iflag to zero to make ma27ad choose a pivot order.
  int iflag = 0;

  double ops;
  ma27ad_( &n, &nnz, irowM, jcolM, iw, &liw, ikeep, iw1, &nsteps, &iflag,
	   icntl, cntl, info, &ops);

  delete [] iw;
  delete [] iw1;

  // if this->ma27ErrFlg() != 0, there's trouble
  switch ( this->ma27ErrFlg() ) {
  case -1 : {
    cerr << "n out of range: " << n << endl;
    assert(0);
  }; break;
  case -2 : {
    cerr << "nnz out of range: " << nnz << endl;
    assert(0);
  }; break;
  case -3 : {
    if( gOoqpPrintLevel >= 100 ) {
      cout << "insufficient space in iw: " << liw 
	   << " suggest reset to " << this->ierror() << endl;
    }
  }; break;
  case 1 : {
    cerr << "detected " << this->ierror() 
	 << " entries out of range in irowM and jcolM; ignored" 
	 << endl;
  }; break;
  }

  // set la in prep for subsequent calls to ma27bd
  la = (int) (1.2 *  this->minimumRealWorkspace());
  // allocate space to hold factors of M (increase this later if it
  // proves inadequate)
  fact = new double[la]();

  // set iw and iw1 in prep for calls to ma27bd and ma27cd
  liw = (int) (1.2 *  this->minimumIntWorkspace());
  iw = new int[liw]();
  iw1 = new int[n]();
  iw2 = new int[nsteps]();

  // allocate w too - since we know it needs at most n locations
  w = new double[n]();

}  

void Ma27SolverBase::diagonalChanged( int /* idiag */, int /* extent */ )
{
  this->matrixChanged();
}

void Ma27SolverBase::matrixChanged()
{
  // if fact has not been allocated, this must be the first call.
  int print_level = gOoqpPrintLevel;
  if( !fact ) this->firstCall();  
  int done = 0, tries = 0;
  do {
    // copy M to fact
    this->copyMatrixElements( fact, la );

#ifdef HAVE_GETRUSAGE
    rusage before;
    if( print_level >= 100 ) {
      getrusage( RUSAGE_SELF, &before );
    }
#endif
    ma27bd_( &n,       &nnz,    irowM, jcolM, 
	     fact,     &la,
	     iw,       &liw,
	     ikeep,
	     &nsteps,  &maxfrt,
	     iw1,      icntl,   cntl,  info );
#ifdef HAVE_GETRUSAGE
    rusage  after;
    if( print_level >= 100 ) {
      getrusage( RUSAGE_SELF, &after );
      cout << "For try " << tries + 1 
	   << " the factorization took " 
	   << (double) (after.ru_utime.tv_sec - before.ru_utime.tv_sec)
	+ (after.ru_utime.tv_usec - before.ru_utime.tv_usec) / 1000000.0
	   << " seconds.\n";
    }
#endif

    switch ( this->ma27ErrFlg() ) {
    case 0: done = 1;
      break;
    case -1 : {
      cerr << "n out of range: " << n << endl; assert(0);
    }; break;
    case -2 : {
      cerr << "nnz out of range: " << nnz << endl; assert(0);
    }; break;
    case -3 : {
      if ( print_level >= 100 )
	cout << "insufficient space in iw: " << liw;
      delete [] iw;
      liw = (this->ierror() > ipessimism * liw) ? 
	this->ierror() : (int)(ipessimism * liw);
      iw = new int[liw]();
      if( print_level >= 100 )
	cout << " resetting to " << liw << endl;

      ipessimism *= 1.1;
    }; break;
    case -4 : {
      if( print_level >= 100 )
	cout << "insufficient factorization space: " << la;
      delete [] fact;
      la = (this->ierror() > rpessimism * la)  ?
	this->ierror() : (int) (rpessimism * la);
      fact = new double [la]();
      this->copyMatrixElements( fact, la );
      if( print_level >= 100 ) cout << " resetting to " << la << endl;
      rpessimism *= 1.1;
    }; break;
    case -5 : {
      if( print_level >= 100 ) {
	cout << "matrix apparently numerically singular, detected at stage " 
	     << this->ierror() << endl;
	cout << "accept this factorization and hope for the best.." << endl;
      }
      done = 1;
    }; break;
    case -6 : {
      if( print_level >= 100 ) {
	cout << "change of sign of pivots detected at stage " 
	     << this->ierror() << endl;
	cout << "but who cares " << endl;
      }
      done = 1;
    }; break;
    case -7 : {
      cerr << "value of NSTEPS out of range " << nsteps << endl;
      assert(0);
    }; break;
    case 1 : {
      if( print_level >= 100 ) {
	cout << "detected " << this->ierror() 
	     << " entries out of range in irowM and jcolM; ignored" 
	     << endl;
      }
      done = 1;
    }; break;
    case 3 : {
      if( print_level >= 100 ) {
	cout << "rank deficient matrix detected; apparent rank is " 
	     << this->ierror() << endl;
      }
      done = 1;
    }; break;  
    default: {
    }; break;
    }    
    tries++;
  } while( !done && tries < 10);

  if ( !done && tries >= 10) {
    if( print_level >= 100 ) {
      cout << "we are screwed; did not get a factorization after 10 tries " 
	   << endl;
    }
    throw std::runtime_error("MA27 cannot factor matrix");
  }

  // allocate w for subsequent calls to ma27cd_
  // w = new double[maxfrt];
  
}

void Ma27SolverBase::basicSolve( double * drhs, int nn )
{
#ifdef HAVE_GETRUSAGE
  int print_level = gOoqpPrintLevel;
  rusage before;
  if( print_level >= 100 ) {
    getrusage( RUSAGE_SELF, &before );
  }
#endif
  ma27cd_( &nn,     fact,     &la,
	   iw,     &liw,
	   w,      &maxfrt,
	   drhs,   iw1,
	   &nsteps,
	   icntl, info );
#ifdef HAVE_GETRUSAGE
  rusage after;
  if( print_level >= 100 ) {
    getrusage( RUSAGE_SELF, &after );
    cout << "Solution with the factored matrix took "
	 << (double) (after.ru_utime.tv_sec - before.ru_utime.tv_sec)
      + (after.ru_utime.tv_usec - before.ru_utime.tv_usec) / 1000000.0
	 << " seconds.\n";
  }
#endif

}

void Ma27Solver::solve( OoqpVector& rhs_in )
{
  SimpleVector & rhs = dynamic_cast<SimpleVector &>(rhs_in);

  // define structures to save rhs and store residuals
  SimpleVectorHandle resid  ( new SimpleVector(n) );
  SimpleVectorHandle rhsSave( new SimpleVector(n) );

  double * drhs = rhs.elements();
  double * dresid = resid->elements();

  rhsSave->copyFrom(rhs);
  resid->copyFrom(rhs);
  double rhsnorm = rhs.infnorm();

  // compute norm of rhs, and save it
  //  double * resids_ma27 = new double[n];
  //  double * save_rhs    = new double[n];
  //  double rhsnorm=0.0, rnorm=0.0; int ii;
  //  for(ii=0, rhsnorm=0.0; ii<n; ii++) {
  //    resids_ma27[ii] = drhs[ii]; save_rhs[ii] = drhs[ii];
  //    rhsnorm += drhs[ii] * drhs[ii];
  //  }
  //  rhsnorm = sqrt(rhsnorm);

  int done = 0, refactorizations = 0;

  while (!done && refactorizations < 10) {
    this->basicSolve( drhs, rhs.length() );
    
    // compute residuals
    mMat->mult(-1.0, dresid, 1, 1.0, drhs, 1);
    //    for(ii=0, rnorm=0.0; ii<n; ii++) 
    //        rnorm += resids_ma27[ii] * resids_ma27[ii]; 
    //    rnorm = sqrt(rnorm);
    double rnorm = resid->infnorm();
    //    cout << "relative norm of residuals for linear system: " 
    //	 << rnorm/rhsnorm << endl;
    
    if(rnorm < precision*(1.e0+rhsnorm)) {
      // residuals are small enough, use this solution
      done = 1;
    } else if (this->thresholdPivoting() >= kThresholdPivotingMax 
	       || refactorizations > 10)  {
      // ThresholdPivoting parameter is already too high; give up and
      // use this solution, whatever it is (the algorithm may bomb in
      // an outer loop).
      done = 1;
    } else {
      // refactor with a higher Threshold Pivoting parameter
      double tp = this->thresholdPivoting();
      tp *= kThresholdPivotingFactor;
      if( tp > kThresholdPivotingMax ) tp = kThresholdPivotingMax;
      this->setThresholdPivoting(tp);

      if( gOoqpPrintLevel >= 100 ) {
	cout << "Setting ThresholdPivoting parameter to " 
	     << this->thresholdPivoting()
	     << " for future factorizations" << endl;
      }
      this->matrixChanged(); refactorizations++;
      // restore rhs to prepare for the solve in the next loop
      //      for(ii=0; ii<n; ii++) {
      //	resids_ma27[ii] = save_rhs[ii]; drhs[ii] = save_rhs[ii];
      //}
      resid->copyFrom(*rhsSave);
      rhs.copyFrom(*rhsSave);
    }
  }
//  delete dresid;
//  delete drhsSave;
}

void Ma27Solver::copyMatrixElements( double afact[], int lafact ) 
{
  double * M        = mMat->M();
  int i;
  for (i=0; i<nnz; i++) afact[i] = M[i];
  for (i = nnz; i<lafact; i++) afact[i] = 0.0;
}

void Ma27Solver::getIndices( int irow[], int jcol[] )
{
  int * krow= mMat->krowM();
  for( int i = 0; i < n; i++ ) {
    for( int k = krow[i]; k < krow[i+1]; k++ ) {
      irow[k] = i + 1;
    }
  }
  int * jcolMat = mMat->jcolM();
  for( int k = 0; k < nnz; k++ ) {
    jcol[k] = jcolMat[k] + 1;
  }
}

Ma27SolverBase::~Ma27SolverBase()
{
  delete [] irowM;
  delete [] jcolM;
  delete [] fact;
  delete [] ikeep;
  delete [] iw;
  delete [] iw1;
  delete [] iw2;
  delete [] w;
}





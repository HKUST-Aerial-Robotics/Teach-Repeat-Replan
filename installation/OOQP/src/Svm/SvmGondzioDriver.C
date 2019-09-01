/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "SvmData.h"
#include "SvmVars.h"
#include "SvmResiduals.h"
#include "SvmLinsys.h"
#include "GondzioSolver.h"
#include "Svm.h"
#include "SimpleVector.h"
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;
#include <cstdio> 
#include <cstdlib>


int main( int argc, char *argv[] )
{
  Svm     * svm         = new Svm;
  SvmData * prob        = 0;
  int quiet = 0, print_soln = 0;
  
  char    * outfilename = 0;
  int argsOk = 1;
  {
    int iarg;
    
    for( iarg = 1; iarg < argc && argv[iarg][0] == '-'; iarg++ ) {
      // it is a option. Check against recognized options
      if( 0 == strcmp( argv[iarg], "-quiet" ) ||
	  0 == strcmp( argv[iarg], "--quiet" ) ) {
	quiet = 1;
      } else if ( 0 == strcmp( argv[iarg], "-print-solution" ) ||
                  0 == strcmp( argv[iarg], "--print-solution" ) ||
	          0 == strcmp( argv[iarg], "-print_solution" ) ||
		  0 == strcmp( argv[iarg], "--print_solution" ) ) {

	print_soln = 1;
      } else {
	cerr << argv[0] << ": "
	     << argv[iarg] << " is not a recognized option.\n";
	argsOk = 0;
      }
    }
    if( iarg >= argc ) argsOk = 0;  // Not enough arguments 
    if( argsOk ) { // the options were read successfully
      if( 0 == strcmp( argv[iarg] , "random" ) ) {
	int hyperplanedim, nobservations;

	if( iarg + 2 >= argc ) { 
	  // Not enough arguments
	  argsOk = 0;
	} else { // there are enough args
	  hyperplanedim = atoi(argv[iarg + 1]); 
	  nobservations = atoi(argv[iarg + 2]);
	  if(hyperplanedim <= 0) {
	    cerr << " Hyperplane dimension must be positive;"
		 << " hyperplanedim=" <<  hyperplanedim << endl;
	    argsOk = 0;
	  }
	  if(nobservations <= 1) {
	    cerr << " Must have at least two observations;"
		 << " nobservations=" <<  nobservations << endl;
	    argsOk = 0;
	  }
	  if(nobservations < hyperplanedim) {
	    cerr << " Number of observations must be no smaller"
		 << " than hyperplane dimension\n";
	    argsOk = 0;
	  }
	  if( argsOk ) {
	    prob = (SvmData *) svm->makeRandomData(hyperplanedim,
						   nobservations, 1.0);
	  }
	} // there are enough args
      } else { // The data is to be read from a file
	// There can be at most one more argument
	if( argc > iarg + 2 ) { // Too many args
	  argsOk = 0;
	} else { // We have the right number of arguments
	  double penalty  = 1.0;
	  if( iarg + 1 < argc ) { // the penalty parameter was specified
	    char * endptr;
	    char * penstr = argv[iarg+1];
	    penalty = strtod( penstr, &endptr );
	    if( penstr == endptr || // No conversion could be done or
		*endptr != '\0' ) { // We didn't use the whole argument
	      cerr << "Error: I couldn't parse " << penstr 
		   << "as a floating point number\n";
	      argsOk = 0;
	    }
	  } // end if the penalty parameter was specified
	  if( argsOk ) { // syntax of the args is good
	    char * filename = argv[iarg];
	    // Get a name for the output file.
	    outfilename = new char[strlen(filename) + 5]; 
	    strcpy( outfilename, filename );
	    strcat( outfilename, ".out" );
	    // Try to read the input file
	    int iErr;
	    // Note that we, probably incorrectly, use twice the penalty
	    // parameter internally.
	    prob = (SvmData *) 
	      svm->makeDataFromText(filename, 2.0 * penalty, iErr);
	    if(iErr != svminputok) {
	      cerr << " Error reading input file " << filename 
		   <<": TERMINATE\n";
	      return 1; // The args parse, but we can't read the file
	    }	  
	  } // if syntax of the args is good
	} // end else have the right number of arguments
      }  // end else the data is to be read from a file
    } // end if the options were read successfully
  } // end of the scope of iarg
  if( !argsOk ) {
    cerr << "\nUsage: \n\n";
    cerr << "    " << argv[0] << " [ --quiet ] [ --print-solution ]"
	 << " filename [ penalty ] \n\nor\n\n";
    cerr << "    " << argv[0] << " [ --quiet ] [ --print-solution ] "
	 << "random hdim nobs\n\n";
    cerr << "where \"random\" is a literal keyword.\n\n";

    delete svm;
    delete prob;

    return 1;
  }

  GondzioSolver * s     = new GondzioSolver( svm, prob );
  SvmVars       * vars  = (SvmVars *) svm->makeVariables( prob );
  Residuals     * resid = svm->makeResiduals( prob );
  if( !quiet ) {
    s->monitorSelf();
  }
  int status = s->solve(prob, vars, resid);

  // print the interesting variables
  if( (!quiet && vars->hyperplanedim < 20) ||
      !outfilename || print_soln ) {
    cout.precision(4);
    vars->printCoefs();
  }
  if( outfilename ) {
    {
      ofstream outfile( outfilename );
      outfile.precision(16);
      outfile << vars->hyperplanedim << endl;
      vars->w->writeToStream( outfile );
      outfile << vars->beta << endl;
    }
    delete [] outfilename;
  } 

  delete vars;  
  delete resid;
  delete s;
  delete prob;
  delete svm;

  return status;
}



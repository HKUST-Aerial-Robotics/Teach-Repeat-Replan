/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "HuberData.h"
#include "HuberVars.h"
#include "HuberResiduals.h"
#include "HuberLinsys.h"
#include "GondzioSolver.h"
#include "Huber.h"
#include "SimpleVector.h"

#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;
#include <cstdio>
#include <cstdlib>

int main( int argc, char *argv[] )
{
  Huber     * huber = new Huber;
  HuberData * prob  = 0;
  int quiet = 0, print_soln = 0;
  char * outfilename = 0;
  int argsOk = 1;
  { // Scope of iarg
    int iarg;
    for( iarg = 1; iarg < argc && argv[iarg][0] == '-'; iarg++ ) {
      // it is a option. Check against recognized options
      if( 0 == strcmp( argv[iarg], "-quiet" ) ||
	  0 == strcmp( argv[iarg], "--quiet" ) ) {
	quiet = 1;
      } else if ( 0 == strcmp( argv[iarg], "-print_solution" ) ||
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
	if( iarg + 3 != argc ) { // wrong number of input args
	  argsOk = 0;
	} else { // the right number of args for a random problem
	  int nobservations = atoi(argv[iarg+1]);
	  int npredictors   = atoi(argv[iarg+2]);
	  if(nobservations <= 0 || npredictors <= 0) {
	    cerr << " Dimensions of random problem should be positive:"
		 << " nobservations=" << nobservations
		 << ", npredictors=" << npredictors << endl;
	    return 1;// bail out
	  }
	  if( nobservations < npredictors ) {
	    cerr << " The number of observations "
	         << "must be at least as large as\n"
		 << " the number of predictors."
		 << " nobservations=" << nobservations
		 << ", npredictors=" << npredictors << endl;
	    return 1; // bail out
	  }
	  prob    = (HuberData *) huber->makeRandomData( nobservations,
							 npredictors, 1.0 );
	} // end else the right number of args for a random problem
      } else { // data is to be read from a file
	if( iarg + 2 != argc ) { // wrong number of input args
	  argsOk = 0;
	} else { // the right number of args
	  char * filename = argv[iarg];
	  double cutoff   = atof( argv[iarg+1] );

	  int iErr;
	  prob = (HuberData *)
	    huber->makeDataFromText(filename, cutoff, iErr);
	  if(iErr != huberinputok) {
	    cerr << " Error reading input file " << filename
		 << " : TERMINATE\n";
	    return 1;
	  }
	  { // Get a name for the output file
	    int lenname = strlen( filename );
	    outfilename = new char[ lenname + 5 ];
	    strcpy( outfilename, filename );
	    strcat( outfilename, ".out" );
	  } 
	} // else the right number of args
      } // end else data is to be read from a file
    } // end if the options were read sucessfully
  } // end scope of iarg
  if( !argsOk ) {
    cerr << "Usage:\n\n";
    cerr << "    " << argv[0] << " [ --quiet ] [ --print-solution ] "
	 << "filename cutoff\n\nor\n\n";
    cerr << "    " << argv[0] << " [ --quiet ] [ --print-solution ] "
	 << "random nobs npred\n\n";
    cerr << "where \"random\" is a literal keyword.\n\n";

    delete huber;
    delete prob;

    return 1;
  }

  // OK, solve this sucker. 

  GondzioSolver * s       = new GondzioSolver( huber, prob );
  HuberVars     * vars    = (HuberVars * ) huber->makeVariables( prob );
  Residuals     * resid   = huber->makeResiduals( prob );

  if( !quiet ) s->monitorSelf();
  int status = s->solve(prob,vars, resid);

  // print the interesting variables: beta
  if( (!quiet && vars->npredictors < 20) ||
      !outfilename || print_soln ) {
    cout.precision(4);
    vars->printBeta();
  }
  if( outfilename ) {
    {
      ofstream outfile( outfilename );
      outfile.precision(16);
      outfile << vars->npredictors << endl;
      vars->beta->writeToStream( outfile );
    }
    delete [] outfilename;
  }

  delete vars;
  delete resid;
  delete s;
  delete prob;
  delete huber;

  return status;
}

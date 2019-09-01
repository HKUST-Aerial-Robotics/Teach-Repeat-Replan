/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundData.h"
#include "QpBoundVars.h"
#include "QpBoundResiduals.h"
#include "GondzioSolver.h"
#include <iostream>
#include <fstream>
using namespace std;
#include <cstdlib>
#include "QpBoundDense.h"
#include "MpsReader.h"


// void QPDataPrint(QPData *prob);
extern int gOoqpPrintLevel;

int main( int argc, char *argv[] )
{
  char filename[256];
  int goodUsage = 1; // Assume good. Why not be optimistic
  int iargv    = 1;
  int status;

  while( iargv < argc ) {
    if( argv[iargv][0] != '-' ) break; // done with the options
    int iopt = 1; // Assume argv[iargv][1] is the start of the option name
    if( argv[iargv][1] == '-' ) iopt = 2; // it is a --arg
    if( 0 == strcmp( "print-level", &argv[iargv][iopt] ) ) {
      if( ++iargv >= argc ) break;
      char * endptr;
      gOoqpPrintLevel = strtol( argv[iargv], &endptr, 10 );
      if( '\0' != *endptr ) { 
	goodUsage = 0;
	break;
      }
    } else if( 0 == strcmp( "quiet", &argv[iargv][iopt] ) ) {
      gOoqpPrintLevel = 0;
    } else if( 0 == strcmp( "verbose", &argv[iargv][iopt] ) ) {
      gOoqpPrintLevel = 1000;
    } else {
      // Unknown option, bug out of the option parsing loop
      goodUsage = 0;
      break;
    }
    iargv++;
  }
  if( iargv == argc - 1 && goodUsage ) {
    strncpy( filename, argv[iargv], 255 );
    filename[255] = '\0';
  } else {
    cout << "Usage: " << argv[0] << " [ --print-level num ] "
    "[ --quiet ] [ --verbose ] problem.qps\n";
    return 1;
  }

  try {
    int iErr;
    MpsReader * reader  = MpsReader::newReadingFile( filename, iErr );
    if( !reader ) {
      cout << "Couldn't read file " << filename << endl 
	   << "For what it is worth, the error number is " << iErr << endl;
      return 1;
    }    
    int nx, my, mz;
    reader->getSizes( nx, my, mz );
    if( my > 0 || mz > 0 ) {
      cerr << "This program can only solve QP with simple bounds.\n";
      return 1;
    }

    QpBoundDense * qp   = new QpBoundDense( nx );
    QpBoundData * prob  = qp->makeData();
    prob->datainput( reader, iErr );
    if( 0 != iErr ) {
      cout << "Couldn't read file " << filename << endl
	   << "For what it is worth, the error number is " << iErr << endl;
      return 1;
    }
    reader->releaseFile( iErr ); 
    if( 0 != iErr ) {
      cout << "Couldn't close file " << filename << endl 
	   << "For what it is worth, the error number is " << iErr << endl;
      return 1;
    }

    Variables * vars   = qp->makeVariables( prob );
    Residuals * resid  = qp->makeResiduals( prob );
    
    GondzioSolver * s   = new GondzioSolver( qp, prob );
    
    s->monitorSelf();
    status = s->solve(prob,vars,resid);
    cout.precision(5);
    vars->print();
    
    delete s;
    delete vars;  
    delete resid;
    delete prob;
    delete qp;
    delete reader;
  } 
  catch( ... ) {
    cerr << "`nOops, out of memory\n";
    return 1;
  }
  return status;
}







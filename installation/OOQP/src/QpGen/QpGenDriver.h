/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPGENDRIVER
#define QPGENDRIVER

#include <memory>
#include <cstring>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdlib>

#ifdef HAVE_GETRUSAGE
#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>
#endif

#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "MpsReader.h"
#include "SimpleVector.h"
#include "Status.h"
#include "QpGenData.h"
#include "OoqpVersion.h"

extern int gOoqpPrintLevel;
int scale = 0;

template<class SOLVER, class FORMULATION>
int qpgen_solve( int argc, char * argv[],
		 SOLVER * /* dum1 */, FORMULATION * /* dum2 */ )
{
  char filename[256];
  int goodUsage = 1; // Assume good. Why not be optimistic
  int iargv    = 1;

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
    } else if( 0 == strcmp( "version", &argv[iargv][iopt] ) ) {
      printOoqpVersionString();
      exit(0);
    } else if( 0 == strcmp( "scale", &argv[iargv][iopt] ) ) {
      scale = 1;
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
    cerr << "Usage: " << argv[0] << " [ --version ] [ --print-level num ] "
      "[ --quiet ] [ --verbose ] [--scale] problem.qps\n";
    return 1;
  }

  try {
    int iErr;
#ifdef HAVE_GETRUSAGE
    rusage before_read;
    getrusage( RUSAGE_SELF, &before_read );
#endif
    MpsReader * reader  = MpsReader::newReadingFile( filename, iErr );
    if( !reader ) {
      std::cerr << "Couldn't read file " << filename << std::endl
		<< "For what it is worth, the error number is "
		<< iErr << std::endl;
      return 1;
    }
    
    int n1, m1, m2, nnzQ, nnzA, nnzC;
    reader->getSizes( n1, m1, m2 );
    reader->numberOfNonZeros( nnzQ, nnzA, nnzC );
    
    FORMULATION * qp 
      = new FORMULATION( n1, m1, m2, nnzQ, nnzA, nnzC );
    QpGenData * prob = (QpGenData * ) qp->makeData();
 
    prob->datainput( reader, scale, iErr );
    if( 0 != iErr ) {
      std::cerr << "Couldn't read file " << filename << std::endl
	   << "For what it is worth, the error number is " << iErr << std::endl;
      return 1;
    }
    reader->releaseFile( iErr );
#ifdef HAVE_GETRUSAGE
    rusage  after_read;
    getrusage( RUSAGE_SELF, &after_read );
#endif

    if( 0 != iErr ) {
      std::cerr << "Couldn't close file " << filename << std::endl
	   << "For what it is worth, the error number is " << iErr << std::endl;
      return 1;
    }
    
    QpGenVars     * vars  = (QpGenVars * ) qp->makeVariables( prob );
    Residuals     * resid = qp->makeResiduals( prob );
    SOLVER * s            = new SOLVER( qp, prob );
    
    s->monitorSelf();
#ifdef HAVE_GETRUSAGE
    rusage before_solve;
    getrusage( RUSAGE_SELF, &before_solve );
#endif
    int result = s->solve(prob,vars, resid);
    if( 0 == result ) {
      if( gOoqpPrintLevel > 0 ) {
#ifdef HAVE_GETRUSAGE
	rusage  after_solve;
	getrusage( RUSAGE_SELF, &after_solve );
	double read_time =
	  (after_read.ru_utime.tv_sec - before_read.ru_utime.tv_sec)
	  + (after_read.ru_utime.tv_usec - before_read.ru_utime.tv_usec)
	  / 1000000.0;
	double solve_time =
	  (after_solve.ru_utime.tv_sec - before_solve.ru_utime.tv_sec)
	  + (after_solve.ru_utime.tv_usec - before_solve.ru_utime.tv_usec)
	  / 1000000.0;

	cout << " QP read in " << read_time << " seconds.  "
	     << "QP solved in " << solve_time << " seconds.\n";
#endif
      //  vars->print();
      
      double objective = reader->objconst() + prob->objectiveValue(vars);
      
      cout << " " << n1 << " variables, " 
	   << m1  << " equality constraints, " 
	   << m2  << " inequality constraints.\n";
      
      cout << " Iterates: " << s->iter
	   <<",    Optimal Solution:  " << objective << std::endl;
      }
      vars->printSolution(reader, prob, scale, iErr);
    } else {
      if ( gOoqpPrintLevel > 0 ) {
	cout << "Could not solve this QP.\n";
	cout << "Terminated with code " << result;
	if( result > 0 && result <= UNKNOWN ) {
	  cout << " : " << TerminationStrings[result];
	}
	cout << ".\n";
      }
    }
    
    delete s;
    delete vars;  
    delete resid;
    delete prob;
    delete qp;
    delete reader;

    return result;
  } 
  catch(const std::exception & e) {
    std::cerr << "\nOops, fatal error:\n";
    std::cerr << e.what() << std::endl;
    return -1;
  }
}

#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpBoundData.h"
#include "QpBoundVars.h"
#include "QpBoundResiduals.h"
#include "MehrotraSolver.h"
#include <iostream>
#include <cstdlib>
#include "QpBoundPetsc.h"
#include "MpsReader.h"


// void QPDataPrint(QPData *prob);
extern int gOoqpPrintLevel;

int main( int argc, char *argv[] )
{
  PetscTruth quiet, verbose, hasPrintLevel, hasFilename;
  int printLevel, status;

  char filename[256];
  PetscInitialize( &argc, &argv, 0, 0 );
  PetscOptionsGetString(0,"-file",filename,200,&hasFilename);
  PetscOptionsGetInt(0,"-print-level",&printLevel, &hasPrintLevel );
  PetscOptionsHasName(0,"-quiet", &quiet );
  PetscOptionsHasName(0,"-verbose", &verbose );

  if ( !hasFilename ) {
    cout << "Usage: " << argv[0] << " [ -print-level num ] "
    "[ -quiet ] [ -verbose ] -file problem.qps\n";
    return 1;
  }

  if( hasPrintLevel ) gOoqpPrintLevel = printLevel;
  if( quiet ) gOoqpPrintLevel = 0;
  if( verbose ) gOoqpPrintLevel = 1000;

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

    QpBoundPetsc * qp   = new QpBoundPetsc( nx );
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
    
    MehrotraSolver * s   = new MehrotraSolver( qp, prob );
    
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

  PetscFinalize();
  
  return status;
}







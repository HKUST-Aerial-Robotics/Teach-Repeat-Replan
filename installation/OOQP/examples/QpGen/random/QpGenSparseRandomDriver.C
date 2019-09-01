/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "DoubleMatrix.h"
#include "OoqpVector.h"
#include "QpGenData.h"
#include "QpGenVars.h"
#include "GondzioSolver.h"
#include <string.h>
#include <iostream>
// #include "MpsReader.h"

#include <stdlib.h>
#include "QpGenSparseMa27.h"
#include "SimpleVector.h"

using namespace std;

// void QPDataPrint(QPData *prob);
extern int DenseStorageInstances;

int solutionMatches( QpGenVars * vars, QpGenVars * soln, 
		     QpGenVars * temp, double tol );

int main( int argc, char *argv[] )
{
  int n1 = 5, m1 = 2, m2 = 2;
  if(argc >= 4) {
	n1 = atoi(argv[1]);
	m1 = atoi(argv[2]);
	m2 = atoi(argv[3]);
  } else {
    cout << endl 
	 << " Erroneous calling sequence; should be " << endl 
	 << " qpgen-sparse-mehrotra.exe n my mz " << endl
	 << " where n  = # primal variables, " << endl
	 << "       my = # equality constraints, " << endl
	 << "       mz = # inequality constraints " << endl << endl;
    return 1;
  }
  int nnzQ = (int) .20 * (n1 * n1);
  int nnzA = (int) .15 * (m1 * n1);
  int nnzC = (int) .10 * (m2 * n1);

  if( nnzQ < 3 * n1 ) nnzQ = 3 * n1;
  if( nnzA < 3 * m1 ) nnzA = 3 * m1;
  if( nnzC < 3 * m2 ) nnzC = 2 * m2;

  QpGenSparseMa27 * qp   = new QpGenSparseMa27( n1, m1, m2,
						  nnzQ, nnzA, nnzC );
  QpGenData * prob; QpGenVars * soln;
  qp->makeRandomData( prob, soln );
  QpGenVars * vars      = (QpGenVars *) qp->makeVariables( prob );
  Residuals * resid     = qp->makeResiduals( prob );

  GondzioSolver * s   = new GondzioSolver( qp, prob );
  
  s->monitorSelf();
  int result = s->solve(prob,vars, resid);
  delete s;

  if( 0 == result ) {
    cout.precision(4);
    cout << "\nComputed solution:\n\n";
    vars->print();

    QpGenVars * temp = (QpGenVars *) qp->makeVariables( prob );

    cout << "\nChecking the solution...";
    if( solutionMatches( vars, soln, temp, 1e-4 ) ) {
      cout << "The solution appears to be correct.\n";
    } else {
      cout << "\nThe solution may be wrong "
	"(or the generated problem may be ill conditioned.)\n";
    }
    delete temp;
  } else {
    cout << "Could not solve this problem.\n";
  }

  delete vars;  
  delete soln;
  delete prob;
  delete qp;

  return result;
}

int solutionMatches( QpGenVars * vars, QpGenVars * soln, 
		     QpGenVars * temp, double tol )
{
  temp->copy( vars );

  // Only x is  significant
  temp->x->axpy( -1.0, *soln->x );

  if( temp->x->infnorm()/(1 + soln->x->infnorm()) < tol ) {
    return 1;
  } else {
    return 0;
  }
}


/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "DoubleMatrix.h"
#include "OoqpVector.h"
#include "Data.h"
#include "Variables.h"
#include "Residuals.h"
#include "GondzioSolver.h"
#include <cstring>
#include <iostream>
using namespace std;
#include <cstdlib>
#include "QpExampleDense.h"

// void QPDataPrint(QPData *prob);
extern int DenseStorageBodyInstances;

int main( int argc, char *argv[] )
{
  int n1 = 5, m1 = 2, m2 = 2;
  if(argc >= 4) {
	n1 = atoi(argv[1]);
	m1 = atoi(argv[2]);
	m2 = atoi(argv[3]);
  }

  QpExampleDense * qp   = new QpExampleDense( n1, m1, m2 );
  Data * prob        = qp->makeRandomData();
  Variables * vars   = qp->makeVariables( prob );
  Residuals * resid  = qp->makeResiduals( prob );

  GondzioSolver * s   = new GondzioSolver( qp, prob );
  
  s->monitorSelf();
  s->solve(prob, vars, resid );
  vars->print();

  delete s;
  delete vars;  
  delete prob;
  delete qp;
  
}

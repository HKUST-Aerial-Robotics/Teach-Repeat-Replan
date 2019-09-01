/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenDriver.h"
#include "QpGenDense.h"
#include "GondzioSolver.h"

int main( int argc, char *argv[] )
{
  GondzioSolver * solver = 0;
  QpGenDense    * qpgen  = 0;

  int result = qpgen_solve( argc, argv, solver, qpgen );
  
  return result;

}

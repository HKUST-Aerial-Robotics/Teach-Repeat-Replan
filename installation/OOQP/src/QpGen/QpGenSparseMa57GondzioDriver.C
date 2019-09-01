/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenDriver.h"
#include "QpGenSparseMa57.h"
#include "GondzioSolver.h"

int main( int argc, char *argv[] )
{
  GondzioSolver   * solver = 0;
  QpGenSparseMa57 * qpgen = 0;
  int result = qpgen_solve( argc, argv, solver, qpgen );
  
  if (result == 0) {
      return 0;
  } else {
      return 1;
  }
}

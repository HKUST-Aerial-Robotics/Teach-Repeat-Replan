/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "OoqpPetscMonitor.h"
#include "petscmat.h"

OoqpPetscSelfMonitor::OoqpPetscSelfMonitor()
{
  int ierr;
  ierr = MPI_Comm_rank( PETSC_COMM_WORLD, &rank );
  assert( 0 == ierr );
}


void OoqpPetscSelfMonitor::doIt( Solver * solver, Data * data,
				 Variables * vars,
				 Residuals * resids,
				 double alpha, double sigma,
				 int i, double mu,
				 int status_code,
				 int level )
{
  if( rank == 0 ) {
    this->OoqpSelfMonitor::doIt( solver, data, vars, resids,
				 alpha, sigma,
				 i, mu, status_code, level );
  }
}

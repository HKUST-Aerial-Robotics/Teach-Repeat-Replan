/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCMONITOR
#define PETSCMONITOR

#include "OoqpMonitor.h"
#include <cassert>

class OoqpPetscSelfMonitor : public OoqpSelfMonitor {
protected:
  int rank;
public:
  OoqpPetscSelfMonitor();
  virtual void doIt( Solver * solver, Data * data, Variables * vars,
		     Residuals * resids,
		     double alpha, double sigma,
		     int i, double mu,
                     int status_code,
		     int level );
};
 
#endif

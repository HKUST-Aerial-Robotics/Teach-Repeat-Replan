/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPBOUNDPETSCMEHROTRA
#define QPBOUNDPETSCMEHROTRA

#include "petscmat.h"
#include "OoqpMonitorData.h"

#ifdef __cplusplus
extern "C" { 
#endif
  /** A data structure for holding the various bits needed to solve
   *  a simple bound QP. This structure is part of the CInterface
   *  to using QpBound with Petsc */
  typedef struct {
    void * solver;
    void * prob;
    void * qp;
  } QpBoundPetscContext;

  void PetscQpBoundSolve( double alpha, Vec c,    Mat Q,     int nnzQ,
			  Vec xlow, Vec ixlow, Vec xupp, Vec ixupp,
			  Vec x,
			  int * ierr );
  void PetscQpBoundSetup( double alpha, Vec c,    Mat Q,     int nnzQ,
			  Vec xlow, Vec ixlow, Vec xupp, Vec ixupp,
			  QpBoundPetscContext * ctx,
			  int * ierr );
  void PetscQpBoundFinish( QpBoundPetscContext * ctx,
			   Vec x, Vec t, Vec v, Vec tau, Vec nu,
			   int * ierr );
  void PetscQpBoundCleanUp( QpBoundPetscContext * ctx );

  void PetscQpBoundAddMonitor ( QpBoundPetscContext * ctx, DoItCFunc cmon,
				void * monitorContext );
  void PetscQpBoundUseStatus ( QpBoundPetscContext * ctx, int (*sta)(void *),
				void * statusContext );
  void PetscQpBoundMonitorSelf( QpBoundPetscContext * ctx );
#ifdef __cplusplus
}
#endif

#endif

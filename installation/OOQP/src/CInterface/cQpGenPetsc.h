/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCQPGEN
#define PETSCQPGEN

#include "petscmat.h"

#ifdef __cplusplus
extern "C" {
#endif
  
  void newPetscBounds( Vec templateVec, 
		  Vec * low,        Vec * ilow,
		  Vec * upp,        Vec * iupp,
		  int * ierr );
  
  void freePetscBounds( Vec low, Vec ilow,
		   Vec upp, Vec iupp );

  
  void newPetscQpGen( Vec templateXVec,
		 Vec * c,    
		 Mat * Q,    int * nnzQrows,
		 Vec * xlow, Vec * ixlow,     Vec * xupp, Vec * ixupp,
		 Vec templateYVec,
		 Mat * A,    int * nnzArows,  Vec * b,    
		 Vec templateZVec,
		 Mat * C,    int * nnzCrows,
		 Vec * clow, Vec * iclow,     Vec * cupp, Vec * icupp,
		 int * ierr );

  void freePetscQpGen( Vec c,     Mat Q,
		       Vec xlow,  Vec ixlow,  Vec xupp,  Vec ixupp,
		       Mat A,     Vec b,
		       Mat C,     Vec clow,
		       Vec iclow, Vec cupp,  Vec icupp );

  void PetscQPSolve( Vec c,    Mat Q,     int nnzQ,
		     Vec xlow, Vec ixlow, Vec xupp, Vec ixupp,
		     Mat A,    int nnzA,  Vec b,
		     Mat C,    int nnzC,
		     Vec clow, Vec iclow, Vec cupp, Vec icupp,
		     Vec x,    Vec y,     Vec z,
		     int * ierr );

#ifdef __cplusplus
};
#endif

#endif

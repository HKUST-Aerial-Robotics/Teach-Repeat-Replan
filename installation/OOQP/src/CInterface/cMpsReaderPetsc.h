/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/*
 * Edited by: Quan H. Nguyen 
 */
#ifndef PETSCMPSREADER
#define PETSCMPSREADER

#include "cMpsReader.h"
#include "petscmat.h"

#ifdef __cplusplus
extern "C" {
#endif
  void PetscScatterDoubleStar  ( double dc[], Vec c, int * ierr );
  void PetscScatterBooleanStar ( char cc[], Vec c, int * ierr );
  void PetscSymMatrixFromTriple( int m,
				 int irowQ[], int nnzQ, int jcolQ[],
				 double dQ[], Mat Q, int * ierr );
  void PetscGenMatrixFromTriple( int m, int n,
				 int irowA[], int nnzA, int jcolA[],
				 double dA[], Mat A, int  * ierr );
  void CMpsReaderReadPetscQpGen( void * reader,
				 Vec    templateXVec,
				 Vec  c,     Mat Q,
				 Vec  xlow,  Vec ixlow,
				 Vec  xupp,  Vec ixupp,
				 Vec    templateYVec,
				 Mat  A,     Vec b,
				 Vec    templateZVec,
				 Mat  C,
				 Vec  clow,  Vec iclow,
				 Vec  cupp,  Vec icupp,
				 int  * ierr );

#ifdef __cplusplus
};
#endif

#endif

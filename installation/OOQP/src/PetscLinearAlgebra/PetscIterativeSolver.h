/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCITERATIVESOLVER_H
#define PETSCITERATIVESOLVER_H

#include "DoubleLinearSolver.h"
#include "PetscSpSymMatrixHandle.h"
#include "PetscSparseStorageHandle.h"
#include "PetscVectorHandle.h"
#include "petscksp.h"

class PetscIterativeSolver : public DoubleLinearSolver {
protected:
  KSP mKsp;
  int deleteKSP;
  int cits;
  Vec x;
  PetscVectorHandle hx;
  PetscSparseStorageHandle mStorage;
public:
  PetscIterativeSolver( PetscSpSymMatrix * Mat,
			KSPType ksptype, PCType pctype );
  PetscIterativeSolver( PetscSpSymMatrix * Mat, KSP ksp );

  int totalLinearIterations() { return cits; };
  virtual void diagonalChanged( int idiag, int extent );
  virtual void matrixChanged();
  virtual void solve ( OoqpVector&  x );
  KSP ksp() { return mKsp; }; 
  virtual ~PetscIterativeSolver();
};

#endif

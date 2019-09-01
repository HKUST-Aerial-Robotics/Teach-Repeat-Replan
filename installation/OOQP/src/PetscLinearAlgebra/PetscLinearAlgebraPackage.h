/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCLINEARALGEBRA
#define PETSCLINEARALGEBRA

#include "LinearAlgebraPackage.h"

class PetscLinearAlgebraPackage : public LinearAlgebraPackage {
protected:
  PetscLinearAlgebraPackage() {};
  virtual ~PetscLinearAlgebraPackage() {};
public:
  static PetscLinearAlgebraPackage * soleInstance();
  virtual SymMatrix * newSymMatrix( int size, int nnz );
  virtual GenMatrix * newGenMatrix( int m, int n, int nnz );
  virtual OoqpVector * newVector( int n );
  virtual void whatami( char type[32] );
};

#endif

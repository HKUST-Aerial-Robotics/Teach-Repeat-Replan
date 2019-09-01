/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "PetscLinearAlgebraPackage.h"
#include "PetscSpSymMatrix.h"
#include "PetscSpGenMatrix.h"

#include "PetscVector.h"

PetscLinearAlgebraPackage * PetscLinearAlgebraPackage::soleInstance()
{
  static PetscLinearAlgebraPackage la;

  return &la;
}

SymMatrix * PetscLinearAlgebraPackage::newSymMatrix( int size, int nnz )
{
  return new PetscSpSymMatrix( size, nnz );
}

GenMatrix * PetscLinearAlgebraPackage::newGenMatrix( int m, int n,
							 int nnz )
{
  return new PetscSpGenMatrix( m, n, nnz );
}

OoqpVector * PetscLinearAlgebraPackage::newVector( int n )
{
  return new PetscVector( n );
}

void PetscLinearAlgebraPackage::whatami( char type[] )
{
  char type_[] = "PetscLinearAlgebraPackage";

  int i = 0;
  
  type[0] = type_[0];
  while( type[i] != 0 && i < 31 ) {
    ++i;
    type[i] = type_[i];
  }
}


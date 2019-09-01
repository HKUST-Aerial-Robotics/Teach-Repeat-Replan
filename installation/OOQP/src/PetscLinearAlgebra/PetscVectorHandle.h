/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCVECTORHANDLE
#define PETSCVECTORHANDLE

#include "IotrRefCount.h"
#include "SmartPointer.h"

class PetscVector;
typedef SmartPointer<PetscVector> PetscVectorHandle;

#endif

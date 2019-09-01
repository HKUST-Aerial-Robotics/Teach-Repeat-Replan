/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCSPSYMMATRIXHANDLE_H
#define PETSCSPSYMMATRIXHANDLE_H

#include "IotrRefCount.h"
#include "SmartPointer.h"

class PetscSpSymMatrix;

typedef SmartPointer<PetscSpSymMatrix> PetscSpSymMatrixHandle;

#endif

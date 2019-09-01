/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DOUBLEMATRIXHANDLE_H
#define DOUBLEMATRIXHANDLE_H

#include "SmartPointer.h"
#include "IotrRefCount.h"

class DoubleStorage;
typedef SmartPointer<DoubleStorage> DoubleStorageHandle;

class DoubleMatrix;
typedef SmartPointer<DoubleMatrix> DoubleMatrixHandle;

class GenMatrix;
typedef SmartPointer<GenMatrix> GenMatrixHandle;

class SymMatrix;
typedef SmartPointer<SymMatrix> SymMatrixHandle;

#endif

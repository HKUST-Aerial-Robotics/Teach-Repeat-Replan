/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DOUBLEMATRIXTYPES
#define DOUBLEMATRIXTYPES

// Types for dynamic cast (we wouldn't have to do this if more C++ compilers
// supported C++ RTTI). kSquareMatrix = 2 is reserved.
enum {kGenMatrix=0x01, kSymMatrix = 0x03,
      // 0x10-0x1F - Dense matrices
      kDenseMatrix     = 0x10,
      kDenseGenMatrix  = kGenMatrix + kDenseMatrix,
      kDenseSymMatrix  = kSymMatrix + kDenseMatrix,
      // 0x20-0x2F - SparseMatrices
      kSparseMatrix    = 0x20,
      kSparseGenMatrix = kSparseMatrix + kGenMatrix,
      kSparseSymMatrix = kSparseMatrix + kSymMatrix,
      // 0x30-0x3F - PetscMatrices
      kPetscSpMatrix     = 0x30,
      kPetscSpGenMatrix  = kPetscSpMatrix + kGenMatrix,
      kPetscSpSymMatrix  = kPetscSpMatrix + kSymMatrix
};

#endif

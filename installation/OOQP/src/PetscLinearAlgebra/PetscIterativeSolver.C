/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/*
 * Thanks to Quan H. Nguyen for help porting to newer versions of Petsc.
 */

#include "PetscIterativeSolver.h"
#include "PetscSpSymMatrix.h"
#include "PetscSparseStorage.h"
#include "petscksp.h"
#include "PetscVector.h"
#include "PetscVectorHandle.h"
 
PetscIterativeSolver::PetscIterativeSolver( PetscSpSymMatrix * Mat,
					    KSP ksp ) : 
  mKsp(ksp), deleteKSP(0), cits(0)
							
{
  int dummy, n, ierr;

  
  // Create the Petsc vectors used in the solve.
  mStorage = PetscSparseStorageHandle( Mat->getStorage() );
  mStorage->getSize( dummy, n );

  int first, lastp1, m;
  
  ierr = MatGetOwnershipRange( mStorage->M, &first, &lastp1 );
  m = lastp1 - first;
  hx = PetscVectorHandle( new PetscVector( m, n ) );
  x=hx->pv;
}

PetscIterativeSolver::PetscIterativeSolver( PetscSpSymMatrix * Mat,
					    KSPType ksptype,
					    PCType pctype) :
  mKsp(0), deleteKSP(1), cits(0)
{ 
  PC   pc;
  int ierr;
  int dummy, n;

  // Create the Petsc vectors used in the solve.
  mStorage = PetscSparseStorageHandle( Mat->getStorage() );
  mStorage->getSize( dummy, n );

  int first, lastp1, m;

  ierr = MatGetOwnershipRange( mStorage->M, &first, &lastp1 );
  m = lastp1 - first;
  hx = PetscVectorHandle( new PetscVector( m, n ) );
  x=hx->pv;
 
  /* 
     Create linear solver context
  */
  ierr = KSPCreate(PETSC_COMM_WORLD,&mKsp);assert(ierr == 0);
  /* 
     Set operators. Here the matrix that defines the linear system
     also serves as the preconditioning matrix.
  */
  ierr = KSPSetOperators(mKsp, mStorage->M, mStorage->M,
			  DIFFERENT_NONZERO_PATTERN); assert(ierr == 0);

  /* 
     Set linear solver defaults for this problem (optional).
     - By extracting the KSP and PC contexts from the KSP context,
     we can then directly call any KSP and PC routines to set
     various options.
     - The following four statements are optional; all of these
     parameters could alternatively be specified at runtime via
     KSPSetFromOptions();
  */
  ierr = KSPSetType(mKsp, ksptype);assert(ierr == 0);

  ierr = KSPGetPC(mKsp,&pc);assert(ierr == 0);
  ierr = PCSetType(pc, pctype);assert(ierr == 0);

  ierr = KSPSetTolerances(mKsp,1.e-8,
			  PETSC_DEFAULT, PETSC_DEFAULT,
			  PETSC_DEFAULT);assert(ierr == 0);

  /* 
     Set runtime options, e.g.,
     -ksp_type <type> -pc_type <type> -ksp_monitor -ksp_rtol <rtol>
     These options will override those specified above as long as
     KSPSetFromOptions() is called _after_ any other customization
     routines.
  */
  ierr = KSPSetFromOptions(mKsp);assert(ierr == 0);
  cits = 0; // No iterations yet!
}

void PetscIterativeSolver::matrixChanged()
{
  int ierr;

  ierr = KSPSetOperators(mKsp, mStorage->M, mStorage->M,
			 SAME_NONZERO_PATTERN); assert(ierr == 0);
}

void PetscIterativeSolver::solve( OoqpVector& v_in )
{
  assert( v_in.isKindOf( kPetscVector ) );
  PetscVector & v = dynamic_cast<PetscVector &>(v_in);

  int ierr, its;
 
  /* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 
     Solve the linear system
     - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */
  /* 
     Solve linear system
  */
  ierr = KSPSolve(mKsp, v.pv, x); assert( ierr  == 0);
  ierr = KSPGetIterationNumber(mKsp, &its); assert( ierr  == 0);
  cits += its;
  //    cout << "Call: " << ++count << " its: "
  //         << its << " cumulative: " << cits << endl;

  ierr = VecCopy( x, v.pv ); assert( ierr  == 0);
}

void PetscIterativeSolver::diagonalChanged( int /* idiag */, int /* extent */ )
{
  this->matrixChanged();
}

PetscIterativeSolver::~PetscIterativeSolver()
{
  int ierr;

  if( deleteKSP ) { // We made it, we own it.
    ierr = KSPDestroy( mKsp ); assert( ierr  == 0);
  }
}

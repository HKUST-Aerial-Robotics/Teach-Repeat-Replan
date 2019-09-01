/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "PetscQpGen.h"
#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "MehrotraSolver.h"
#include "PetscVector.h"
#include "PetscQpGen.h"

extern "C"
void newPetscBounds( Vec templateVec,
		Vec * low,        Vec * ilow,
		Vec * upp,        Vec * iupp, 
		int * ierr )
{
  *ierr = VecDuplicate( templateVec,  low );
  if( *ierr == 0 ) VecDuplicate( templateVec, ilow );
  if( *ierr == 0 ) VecDuplicate( templateVec,  upp );
  if( *ierr == 0 ) VecDuplicate( templateVec, iupp );

  if( *ierr != 0 ) {
    freePetscBounds( *low, *ilow, *upp, *iupp );
  }
}
  
extern "C"
void freePetscBounds( Vec low, Vec ilow,
		 Vec upp, Vec iupp )
{
  PetscTruth isValid;
  VecValid( low, &isValid); 
  if( isValid ) VecDestroy(  low );
  
  VecValid( ilow, &isValid );
  if( isValid ) VecDestroy( ilow );

  VecValid( upp, &isValid );
  if( isValid ) VecDestroy(  upp );

  VecValid( iupp, &isValid );
  if( isValid ) VecDestroy( iupp );
}

  
extern "C"
void newPetscQpGen( Vec templateXVec,
		    Vec * c,    
		    Mat * Q,    int * nnzQrows,
		    Vec * xlow, Vec * ixlow,     Vec * xupp, Vec * ixupp,
		    Vec templateYVec,
		    Mat * A,    int * nnzArows,  Vec * b,    
		    Vec templateZVec,
		    Mat * C,    int * nnzCrows,
		    Vec * clow, Vec * iclow,     Vec * cupp, Vec * icupp,
		    int * ierr )
{
  int nx, lnx, my, lmy, mz, lmz;

  *ierr = 0;

  if( *ierr == 0 ) *ierr = VecGetSize     ( templateXVec,  &nx );
  if( *ierr == 0 ) *ierr = VecGetLocalSize( templateXVec, &lnx );

  if( *ierr == 0 ) *ierr = VecGetSize     ( templateYVec,  &my );
  if( *ierr == 0 ) *ierr = VecGetLocalSize( templateYVec, &lmy );

  if( *ierr == 0 ) *ierr = VecGetSize     ( templateZVec,  &mz );
  if( *ierr == 0 ) *ierr = VecGetLocalSize( templateZVec, &lmz );

  if( *ierr == 0 ) *ierr = VecDuplicate( templateXVec, c );
  if( *ierr == 0 ) *ierr = MatCreate( PETSC_COMM_WORLD,
				      lnx, lnx, nx, nx, Q);
  if( *ierr == 0 ) *ierr = MatAssemblyBegin( *Q, MAT_FINAL_ASSEMBLY );
  if( *ierr == 0 ) *ierr = MatAssemblyEnd( *Q, MAT_FINAL_ASSEMBLY );

  if( *ierr == 0 ) {
    newPetscBounds( templateXVec, xlow, ixlow, xupp, ixupp, ierr );
  }
  if( *ierr == 0 ) *ierr = MatCreate( PETSC_COMM_WORLD,
				      lmy, lnx, my, nx, A);
  if( *ierr == 0 ) *ierr = MatAssemblyBegin( *A, MAT_FINAL_ASSEMBLY );
  if( *ierr == 0 ) *ierr = MatAssemblyEnd( *A, MAT_FINAL_ASSEMBLY );
  
  if( *ierr == 0 ) *ierr = VecDuplicate( templateYVec, b );

  if( *ierr == 0 ) *ierr = MatCreate( PETSC_COMM_WORLD,
				      lmz, lnx, mz, nx, C);
  if( *ierr == 0 ) *ierr = MatAssemblyBegin( *C, MAT_FINAL_ASSEMBLY );
  if( *ierr == 0 ) *ierr = MatAssemblyEnd( *C, MAT_FINAL_ASSEMBLY );

  if( *ierr == 0 ) {
    newPetscBounds( templateZVec, clow, iclow, cupp, icupp, ierr );
  }

  if( *ierr != 0 ) {
    freePetscQpGen( *c, *Q,     *xlow, *ixlow, *xupp, *ixupp,
	       *A, *b, *C, *clow, *iclow, *cupp, *icupp );
  }
}

extern "C"
void freePetscQpGen( Vec c,     Mat Q,
		     Vec xlow,  Vec ixlow,    Vec xupp,  Vec ixupp,
		     Mat A,     Vec b,
		     Mat C,     Vec clow,  Vec iclow,    Vec cupp,  Vec icupp )
{
  PetscTruth isValid;
  
  VecValid( c, &isValid );
  if( isValid ) VecDestroy( c );
  
  MatValid( Q, &isValid );
  if( isValid ) MatDestroy( Q );
  freePetscBounds( xlow, ixlow, xupp, ixupp );

  MatValid( A, &isValid );
  if( isValid ) MatDestroy( A );

  VecValid( b, &isValid );
  if( isValid ) VecDestroy( b );

  MatValid( C, &isValid );
  if( isValid ) MatDestroy( C );

  freePetscBounds( clow, iclow, cupp, icupp );
}

extern "C"
void PetscQPSolve( Vec c,     Mat Q,      int nnzQ,
		   Vec xlow,  Vec ixlow,  Vec xupp,  Vec ixupp,
		   Mat A,     int nnzA,   Vec b,
		   Mat C,     int nnzC,
		   Vec clow,  Vec iclow,  Vec cupp,  Vec icupp,
		   Vec x,     Vec y,      Vec z,
		   int * ierr )
{
  int nx, my, mz;
  *ierr = VecGetSize( x, &nx ); CHKERRA( *ierr );
  *ierr = VecGetSize( y, &my ); CHKERRA( *ierr );
  *ierr = VecGetSize( z, &mz ); CHKERRA( *ierr );

  PetscQpGen  * qp    = 0;
  QpGenData          * prob  = 0;
  QpGenVars          * vars  = 0;
  QpGenResiduals     * resid = 0;
  MehrotraSolver     * s     = 0;

  try {
    qp    = new PetscQpGen( nx, my, mz, nnzQ, nnzA, nnzC );
    prob  = (QpGenData * ) qp->makeData( c, Q, 
					 xlow, ixlow, xupp, ixupp,
					 A, b, C,
					 clow, iclow, cupp, icupp );
    vars   = (QpGenVars * )     qp->makeVariables( prob );
    resid  = (QpGenResiduals *) qp->makeResiduals( prob );
    s      = new MehrotraSolver( qp, prob );
  
    //    prob->print();
    s->monitorSelf();
    s->solve(prob,vars, resid);
    PetscVectorOld vx( x ), vy( y ), vz( z );
    vx->copyFrom( vars->x );
    vy->copyFrom( vars->y );
    vz->copyFrom( vars->z );

    *ierr = 0;
  }
  catch( ... ) {
    *ierr = 1;
  }
  delete s;
  delete vars;
  delete resid;
  delete prob;
  delete qp;
}

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/*
 * Edited by: Quan H. Nguyen 
 */
//#include "PetscMpsReader.h"
#include "cMpsReaderPetsc.h"
#include "SparseGenMatrix.h"
#include "SparseSymMatrix.h"
#include "PetscSpSymMatrix.h"
#include "PetscSpGenMatrix.h"
#include "cQpGenSparse.h"
#include <cstdlib>

extern "C"
void PetscSymMatrixFromTriple( int m, 
			       int irowQ[], int nnzQ, int jcolQ[],
			       double dQ[], Mat Q, int * ierr )
{
  int * krowQ = NULL;
  *ierr = 0;
  try {
    krowQ = (int *) malloc( (m + 1) * sizeof( int ) );
    if( krowQ ) {
      makehb( irowQ, nnzQ, krowQ, m, ierr );
      if( *ierr == 0 ) {
	SparseSymMatrix M( m, nnzQ, krowQ, jcolQ, dQ );
	PetscSpSymMatrix PM( Q );
	
	PM.symAtPutSubmatrix( 0, 0, M, 0, 0, m, m );
      }
    } else {
      *ierr = 1;
    }
  }
  catch( ... ) {
    *ierr = 1;
  }
  if( krowQ ) free( krowQ );
}


extern "C"
void PetscGenMatrixFromTriple( int m, int n,
			       int irowA[], int nnzA, int jcolA[],
			       double dA[], Mat A, int  * ierr )
{
  int * krowA = NULL;
  *ierr = 0;
  try {
    krowA = (int *) malloc( (m + 1) * sizeof( int ) );
    if( krowA ) {
      makehb( irowA, nnzA, krowA, m, ierr );
      if( *ierr == 0 ) {
	SparseGenMatrix M( m, n, nnzA, krowA, jcolA, dA );
	PetscSpGenMatrix PM( A );
    
	PM.atPutSubmatrix( 0, 0, M, 0, 0, m, n );
      }
    }
  } 
  catch ( ... ) {
    *ierr = 1;
  }
  if( krowA ) free( krowA );
}


extern "C"
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
			       int  * ierr )
{
  int       nnzQ,    nnzA,    nnzC;
  int     *irowQ,  *irowA,  *irowC;
  int     *jcolQ,  *jcolA,  *jcolC;
  double  *dQ,     *dA,     *dC;
  double  *dc;
  double  *dxlow,  *dxupp;
  char    *cxlow,  *cxupp;
  double  *db;
  double  *dclow,  *dcupp;
  char    *cclow,  *ccupp;

  int nx, my, mz;
  double f;
  
  cMpsReaderGetSizes( reader,  &nx,    &my,    &mz );
  cMpsReaderGetNNZ(   reader,  &nnzQ,  &nnzA,  &nnzC );

  newQpGenSparse( &dc,     nx,
	    &irowQ,  nnzQ,    &jcolQ,  &dQ,
	    &dxlow,  &cxlow,  &dxupp,  &cxupp,
	    &irowA,  nnzA,    &jcolA,  &dA,
	    &db,     my,
	    &irowC,  nnzC,    &jcolC,  &dC,
	    &dclow,  mz,      &cclow,  &dcupp,  &ccupp,
	    ierr );
  
  
  if( 0 == *ierr ) {
    cMpsReaderReadQpGen( reader, &f,
			 dc,     irowQ,  jcolQ,  dQ,
			 dxlow,  cxlow,  dxupp,  cxupp,
			 irowA,  jcolA,  dA,     db,
			 irowC,  jcolC,  dC,
			 dclow,  cclow,  dcupp,  ccupp,
			 ierr );
  }
  if( 0 == *ierr ) PetscScatterDoubleStar(  dc, c, ierr );

  if( 0 == *ierr ) PetscScatterDoubleStar(  dxlow,  xlow, ierr );
  if( 0 == *ierr ) PetscScatterBooleanStar( cxlow, ixlow, ierr );
  if( 0 == *ierr ) PetscScatterDoubleStar(  dxupp,  xupp, ierr );
  if( 0 == *ierr ) PetscScatterBooleanStar( cxupp, ixupp, ierr );

  if( 0 == *ierr ) PetscScatterDoubleStar(  db, b, ierr );

  if( 0 == *ierr ) PetscScatterDoubleStar(  dclow,  clow, ierr );
  if( 0 == *ierr ) PetscScatterBooleanStar( cclow, iclow, ierr );
  if( 0 == *ierr ) PetscScatterDoubleStar(  dcupp,  cupp, ierr );
  if( 0 == *ierr ) PetscScatterBooleanStar( ccupp, icupp, ierr );

  if( 0 == *ierr ) {
    PetscSymMatrixFromTriple( nx, irowQ, nnzQ, jcolQ, dQ, Q, ierr );
  }
  if( 0 == *ierr ) {
    PetscGenMatrixFromTriple( my, nx, irowA, nnzA, jcolA, dA, A, ierr );
  }
  if( 0 == *ierr ) {
    PetscGenMatrixFromTriple( mz, nx, irowC, nnzC, jcolC, dC, C, ierr );
  }

  freeQpGenSparse(&dc,     &irowQ,  &jcolQ,  &dQ,
		  &dxlow,  &cxlow,  &dxupp,  &cxupp,
		  &irowA,  &jcolA,  &dA,     &db,
		  &irowC,  &jcolC,  &dC,
		  &dclow,  &cclow,  &dcupp,  &ccupp );
}

void PetscScatterDoubleStar(  double dc[], Vec c, int * ierr )
{
  int n;
  Vec sc;
  IS is;
  VecScatter ctx;
  int derr;

  *ierr = VecGetSize( c, &n ); if( *ierr != 0 ) return;

  *ierr =
    VecCreateSeqWithArray( PETSC_COMM_SELF, n, dc, &sc );
  if( *ierr == 0 ) {
    *ierr = ISCreateStride( PETSC_COMM_SELF, n, 0, 1, &is );
    if( *ierr == 0 ) {
      *ierr = VecScatterCreate( sc, is, c, is, &ctx );
      if( *ierr == 0 ) {
	*ierr =
	  VecScatterBegin( sc, c, INSERT_VALUES, SCATTER_FORWARD, ctx );
	if( *ierr == 0 ) {
	  *ierr =
	    VecScatterEnd( sc, c, INSERT_VALUES, SCATTER_FORWARD, ctx );
	}	
	derr = VecScatterDestroy( ctx ); if( *ierr == 0 ) *ierr = derr;
      }
      derr = ISDestroy( is ); if( *ierr == 0 ) *ierr = derr;
    }
    derr = VecDestroy( sc ); if( *ierr == 0 ) *ierr = derr;
  }
}

extern "C"
void PetscScatterBooleanStar(  char cc[], Vec c, int * ierr )
{
  int n;
  Vec sc;
  IS is;
  VecScatter ctx;
  int derr;
  double *dc;

  *ierr = VecGetSize( c, &n ); if( *ierr != 0 ) return;

  *ierr =
    VecCreateSeq( PETSC_COMM_SELF, n, &sc );
  if( *ierr == 0 ) {
    *ierr = VecGetArray( sc, &dc ); //CHKERRA( ierr ); // fatal
    int i;
    for( i = 0; i < n; i++ ) dc[i] = cc[i];
    *ierr = VecRestoreArray( sc, &dc ); //CHKERRA( ierr ); // fatal
    
    *ierr = ISCreateStride( PETSC_COMM_SELF, n, 0, 1, &is );
    if( *ierr == 0 ) {
      *ierr = VecScatterCreate( sc, is, c, is, &ctx );
      if( *ierr == 0 ) {
	*ierr =
	  VecScatterBegin( sc, c, INSERT_VALUES, SCATTER_FORWARD, ctx );
	if( *ierr == 0 ) {
	  *ierr =
	    VecScatterEnd( sc, c, INSERT_VALUES, SCATTER_FORWARD, ctx );
	}	
	derr = VecScatterDestroy( ctx ); if( *ierr == 0 ) *ierr = derr;
      }
      derr = ISDestroy( is ); if( *ierr == 0 ) *ierr = derr;
    }
    derr = VecDestroy( sc ); if( *ierr == 0 ) *ierr = derr;
  }
}



/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "MpsReader.h"
#include "cMpsReader.h"

extern "C"
void * newCMpsReader( char filename[], int * ierr )
{
  MpsReader * reader = 0;
  try {
    reader = MpsReader::newReadingFile( filename, *ierr );
  } 
  catch( ... ) {
    *ierr = 1;
  }
  if( *ierr == 0 ) {
    return (void *) reader;
  } else {
    return 0;
  }
}

extern "C"
void freeCMpsReader( void ** preader )
{
  MpsReader * reader = (MpsReader *) *preader;
  int ierr;
  reader->releaseFile(ierr);
  delete reader;
  
  *preader = 0;
}

extern "C"
void cMpsReaderGetSizes( void * reader_, int * nx, int * ny, int * nz )
{
  MpsReader * reader = (MpsReader *) reader_;
  
  reader->getSizes( *nx, *ny, *nz );
}

extern "C" 
void cMpsReaderGetNNZ( void * reader_, int * nnzQ, int * nnzA, int * nnzC )
{
  MpsReader * reader = (MpsReader *) reader_;
  
  reader->numberOfNonZeros( *nnzQ, *nnzA, *nnzC );
}

extern "C" 
void cMpsReaderReadQpGen( void * reader_,
			  double *   f  , double     c[],  
			  int    irowQ[],  int  jcolQ[],  double dQ[],
			  double  xlow[],  char ixlow[],
			  double  xupp[],  char ixupp[],
			  int    irowA[],  int  jcolA[],  double dA[],
			  double     b[],
			  int    irowC[],  int  jcolC[],  double dC[],
			  double  clow[],  char  iclow[],
			  double  cupp[],  char  icupp[],
			  int*    ierr )
{
  MpsReader * reader = (MpsReader *) reader_;

  *ierr = 0;
  try {
    reader->readQpGen( c, irowQ, jcolQ, dQ,
		       xlow, ixlow, xupp, ixupp,
		       irowA, jcolA, dA, b,
		       irowC, jcolC, dC,
		       clow, iclow, cupp, icupp, *ierr );
    *f = reader->objconst();
  }
  catch( ... ) {
    *ierr = 1;
  }

}

static void triple2Dense( int   irow[], int  nnz, int  jcol[],
		   double   C[], int    m, int  n )
{
  // This is an in-place conversion, so we must do it in reverse order.
  // kc is the next location to be written in the dense matrix C
  int kc = m * n - 1;
  // k is the current element in the sparse triple
  int k;
  for( k = nnz - 1; k >= 0; k-- ) {
    int i        = irow[k];
    int j        = jcol[k];

    {
      int kctarget = i * n + j; // Where elt k is going in the dense matrix
      // Fill in the intermediate values with zeros
      for( ; kc > kctarget; kc-- ) {
	C[ kc ] = 0.0;
      }  
      // now kc == kctarget
    }

    C[ kc ] = C[k];
    kc--; // Move on to the next elt of the dense matrix
  }
} 


extern "C" 
void cMpsReaderReadQpGenDense( void * reader_,
			       double     c[],  double     Q[],
			       double  xlow[],  char   ixlow[],
			       double  xupp[],  char   ixupp[],
			       double     A[],  double     b[],
			       double     C[],
			       double  clow[],  char  iclow[],
			       double  cupp[],  char  icupp[],
			       int*    ierr )
{
  MpsReader * reader = (MpsReader *) reader_;
  int nnzQ, nnzA, nnzC;
  int nx,   my,   mz;
  
  reader->numberOfNonZeros( nnzQ, nnzA, nnzC );
  reader->getSizes( nx, my, mz );
  int *irowQ = 0, *jcolQ = 0;
  int *irowA = 0, *jcolA = 0;
  int *irowC = 0, *jcolC = 0;
  try {
    irowQ = new int[nnzQ];
    jcolQ = new int[nnzQ];

    irowA = new int[nnzA];
    jcolA = new int[nnzA];
    
    irowC = new int[nnzC];
    jcolC = new int[nnzC];

    *ierr = 0;
  } catch( ... ) {
    *ierr = 1;
  }
  
  if( *ierr == 0 ) {
    try {
      reader->readQpGen( c, irowQ, jcolQ, Q,
			 xlow, ixlow, xupp, ixupp,
			 irowA, jcolA, A, b,
			 irowC, jcolC, C,
			 clow, iclow, cupp, icupp, *ierr );


      triple2Dense( irowQ, nnzQ, jcolQ, Q, nx, nx ); 
      triple2Dense( irowA, nnzA, jcolA, A, my, nx ); 
      triple2Dense( irowC, nnzC, jcolC, C, mz, nx ); 
    }
    catch( ... ) {
      *ierr = 1;
    }
  }
  delete [] irowQ;
  delete [] jcolQ;
  delete [] irowA;
  delete [] jcolA;
  delete [] irowC;
  delete [] jcolC;

}



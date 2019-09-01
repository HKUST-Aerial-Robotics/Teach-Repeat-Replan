/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef CMPSREADER
#define CMPSREADER

#ifdef __cplusplus
extern "C" {
#endif
  void * newCMpsReader( char filename[], int * ierr );
  
  void cMpsReaderGetSizes( void * reader_, int * nx, int * ny, int * nz );

  void cMpsReaderGetNNZ( void * reader_, int * nnzQ, int * nnzA, int * nnzC );

  void cMpsReaderReadQpGen( void * reader_, double * f, 
			    double     c[],  
			    int    irowQ[],  int  jcolQ[],  double dQ[],
			    double  xlow[],  char ixlow[],
			    double  xupp[],  char ixupp[],
			    int    irowA[],  int  jcolA[],  double dA[],
			    double     b[],
			    int    irowC[],  int   jcolC[],  double dC[],
			    double  clow[],  char  iclow[],
			    double  cupp[],  char  icupp[],
			    int*    ierr );

  void cMpsReaderReadQpGenDense( void * reader_,
				 double     c[],  double     Q[],
				 double  xlow[],  char   ixlow[],
				 double  xupp[],  char   ixupp[],
				 double     A[],  double     b[],
				 double     C[],
				 double  clow[],  char  iclow[],
				 double  cupp[],  char  icupp[],
				 int*    ierr );

  void freeCMpsReader( void ** reader );

#ifdef __cplusplus
}
#endif

#endif


/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "mex.h"
#include "mexUtility.h"
#include "cMpsReader.h"

void makehb( int irow[], int nnz, int krow[], int m, int * ierr );

/* ****************************************************************
 * mex stub function: mexFunction
 * **************************************************************** */
void mexFunction( int nlhs, mxArray * plhs[], int nrhs,
    const mxArray * prhs[] ) 
{

    double      *dummyDoublePtr;

    char         filename[256];           /* Input    argument 0   */

    double       f;                       /* Output   argument 0   */
    mxArray     *mexf;                   
    double      *c;                       /* Output   argument 1   */
    mxArray     *mexc;                   
    double      *Q;                       /* Output   argument 2   */
    int         *Q_ir;                   
    int         *Q_rows;                 
    int         *Q_jc;                   
    int         *Q_cols;                 
    int          Q_nz;                   
    mxArray     *mexQ;                   
    double      *xlow;                    /* Output   argument 3   */
    mxArray     *mexxlow;                
    double      *ixlow;                   /* Output   argument 4   */
    mxArray     *mexixlow;               
    double      *xupp;                    /* Output   argument 5   */
    mxArray     *mexxupp;                
    double      *ixupp;                   /* Output   argument 6   */
    mxArray     *mexixupp;               
    double      *A;                       /* Output   argument 7   */
    int         *A_ir;                   
    int         *A_rows;                 
    int         *A_jc;                   
    int         *A_cols;                 
    int          A_nz;                   
    mxArray     *mexA;                   
    double      *dA;                      /* Output   argument 8   */
    mxArray     *mexdA;                  
    double      *C;                       /* Output   argument 9   */
    int         *C_ir;                   
    int         *C_rows;                 
    int         *C_jc;                   
    int         *C_cols;                 
    int          C_nz;                   
    mxArray     *mexC;                   
    double      *clow;                    /* Output   argument 10  */
    mxArray     *mexclow;                
    double      *iclow;                   /* Output   argument 11  */
    mxArray     *mexiclow;               
    double      *cupp;                    /* Output   argument 12  */
    mxArray     *mexcupp;                
    double      *icupp;                   /* Output   argument 13  */
    mxArray     *mexicupp;               


    int         nx;                       /* Array dimension     */
    int         my;                       /* Array dimension     */
    int         mz;                       /* Array dimension     */

    int    i;
    int    ierr;
    void * reader;
    int    nnzQ,    nnzA,   nnzC;
    char   *cxlow, *cxupp, *cclow, *ccupp;

    if ( 1 != nrhs ) {
        mexErrMsgTxt( "Wrong number of input arguments" );
    }
    if ( nlhs > 14 ) {
        mexErrMsgTxt( "Too many output arguments" );
    }

    /* filename : Input argument 0 */
    assertString( prhs[0], 255, "filename" );
    assertRowDim( prhs[0], 1, "filename" );
    {
      int lfilename    = mxGetN(prhs[0]);
      mxChar * mexstr  = (mxChar *) mxGetPr(prhs[0]);
      for( i = 0; i < lfilename; i++ ) {
	filename[i] = mexstr[i];
      }
      filename[i] = '\0';
    }

    reader = newCMpsReader( filename, &ierr );
    if( ierr != 0 ) {
      mexErrMsgTxt( "Trouble reading file.\n" );
    }

    cMpsReaderGetSizes( reader, &nx, &my, &mz );
    cMpsReaderGetNNZ( reader, &Q_nz, &A_nz, &C_nz );
    /* Output argument 0, "f", is scalar */

    /* c : output argument 1 */
    mexc = mxCreateDoubleMatrix( nx, 1, mxREAL );
    c =  mxGetPr( mexc );

    /* Q : output argument 2 */
    mexQ =  mxCreateSparse( nx, nx, Q_nz, mxREAL );
    Q    =  mxGetPr( mexQ );
    Q_ir =  mxGetIr( mexQ );
    Q_jc =  mxGetJc( mexQ );

    /* xlow : output argument 3 */
    mexxlow = mxCreateDoubleMatrix( nx, 1, mxREAL );
    xlow =  mxGetPr( mexxlow );

    /* ixlow : output argument 4 */
    mexixlow = mxCreateDoubleMatrix( nx, 1, mxREAL );
    ixlow =  mxGetPr( mexixlow );

    /* xupp : output argument 5 */
    mexxupp = mxCreateDoubleMatrix( nx, 1, mxREAL );
    xupp =  mxGetPr( mexxupp );

    /* ixupp : output argument 6 */
    mexixupp = mxCreateDoubleMatrix( nx, 1, mxREAL );
    ixupp =  mxGetPr( mexixupp );

    /* A : output argument 7 */
    mexA =  mxCreateSparse( nx, my, A_nz, mxREAL );
    A    =  mxGetPr( mexA );
    A_ir =  mxGetIr( mexA );
    A_jc =  mxGetJc( mexA );


    /* dA : output argument 8 */
    mexdA = mxCreateDoubleMatrix( my, 1, mxREAL );
    dA =  mxGetPr( mexdA );

    /* C : output argument 9 */
    mexC = mxCreateSparse( nx, mz, C_nz, mxREAL );
    C    =  mxGetPr( mexC );
    C_ir =  mxGetIr( mexC );
    C_jc =  mxGetJc( mexC );

    /* clow : output argument 10 */
    mexclow = mxCreateDoubleMatrix( mz, 1, mxREAL );
    clow =  mxGetPr( mexclow );

    /* iclow : output argument 11 */
    mexiclow = mxCreateDoubleMatrix( mz, 1, mxREAL );
    iclow =  mxGetPr( mexiclow );

    /* cupp : output argument 12 */
    mexcupp = mxCreateDoubleMatrix( mz, 1, mxREAL );
    cupp =  mxGetPr( mexcupp );

    /* icupp : output argument 13 */
    mexicupp = mxCreateDoubleMatrix( mz, 1, mxREAL );
    icupp =  mxGetPr( mexicupp );
    
    /* We are transposed */
    Q_cols = Q_ir;
    A_cols = A_ir;
    C_cols = C_ir;
   
    Q_rows = (int *) mxCalloc( Q_nz, sizeof(int) );
    A_rows = (int *) mxCalloc( A_nz, sizeof(int) );
    C_rows = (int *) mxCalloc( C_nz, sizeof(int) );

    cxlow  = (char *) mxCalloc( nx, sizeof(char) );
    cxupp  = (char *) mxCalloc( nx, sizeof(char) );
    cclow  = (char *) mxCalloc( mz, sizeof(char) );
    ccupp  = (char *) mxCalloc( mz, sizeof(char) );

    cMpsReaderReadQpGen( reader,  &f,      c,  
			 Q_rows,  Q_cols,  Q,
			 xlow,    cxlow,
			 xupp,    cxupp,
			 A_rows,  A_cols,  A,     dA,
			 C_rows,  C_cols,  C,
			 clow,    cclow,   cupp,  ccupp,  &ierr );
	
	freeCMpsReader( &reader );
    if( ierr != 0 ) mexErrMsgTxt( "Couldn't read the file.\n" );

    for( i = 0; i < nx; i++ ) {
      ixlow[i] = (double) cxlow[i];
      ixupp[i] = (double) cxupp[i];
    }
    for( i = 0; i < mz; i++ ) {
      iclow[i] = (double) cclow[i];
      icupp[i] = (double) ccupp[i];
    }

    makehb( Q_rows, Q_nz, Q_jc, nx, &ierr );
    if( 0 != ierr ) mexErrMsgTxt( "Internal error" );

    makehb( A_rows, A_nz, A_jc, my, &ierr );
    if( 0 != ierr ) mexErrMsgTxt( "Internal error" );

    makehb( C_rows, C_nz, C_jc, mz, &ierr );
    if( 0 != ierr ) mexErrMsgTxt( "Internal error" );
      
    /* Free any scratch arrays */
    mxFree( ccupp );
    mxFree( cclow );
    mxFree( cxupp );
    mxFree( cxlow );
    mxFree( C_rows );
    mxFree( A_rows );
    mxFree( Q_rows );

    /* f : output argument 0 */
    plhs[0] = mxCreateDoubleMatrix( 1, 1, mxREAL );
    dummyDoublePtr = mxGetPr( plhs[0] );
    *dummyDoublePtr =  f;

    /* c : output argument 1 */
    if ( 1 < nlhs ) {
        plhs[1] = mexc;
    } else {
        mxDestroyArray( mexc );
    }

    /* Q : output argument 2 */
    if ( 2 < nlhs ) {
        plhs[2] = mexQ;
    } else {
        mxDestroyArray( mexQ );
    }

    /* xlow : output argument 3 */
    if ( 3 < nlhs ) {
        plhs[3] = mexxlow;
    } else {
        mxDestroyArray( mexxlow );
    }

    /* ixlow : output argument 4 */
    if ( 4 < nlhs ) {
        plhs[4] = mexixlow;
    } else {
        mxDestroyArray( mexixlow );
    }

    /* xupp : output argument 5 */
    if ( 5 < nlhs ) {
        plhs[5] = mexxupp;
    } else {
        mxDestroyArray( mexxupp );
    }

    /* ixupp : output argument 6 */
    if ( 6 < nlhs ) {
        plhs[6] = mexixupp;
    } else {
        mxDestroyArray( mexixupp );
    }

    /* A : output argument 7 */
    if ( 7 < nlhs ) {
        plhs[7] = mexA;
    } else {
        mxDestroyArray( mexA );
    }

    /* dA : output argument 8 */
    if ( 8 < nlhs ) {
        plhs[8] = mexdA;
    } else {
        mxDestroyArray( mexdA );
    }

    /* C : output argument 9 */
    if ( 9 < nlhs ) {
        plhs[9] = mexC;
    } else {
        mxDestroyArray( mexC );
    }

    /* clow : output argument 10 */
    if ( 10 < nlhs ) {
        plhs[10] = mexclow;
    } else {
        mxDestroyArray( mexclow );
    }

    /* iclow : output argument 11 */
    if ( 11 < nlhs ) {
        plhs[11] = mexiclow;
    } else {
        mxDestroyArray( mexiclow );
    }

    /* cupp : output argument 12 */
    if ( 12 < nlhs ) {
        plhs[12] = mexcupp;
    } else {
        mxDestroyArray( mexcupp );
    }

    /* icupp : output argument 13 */
    if ( 13 < nlhs ) {
        plhs[13] = mexicupp;
    } else {
        mxDestroyArray( mexicupp );
    }

}
/* ****************************************************************
 * end of mex stub: mexFunction
 * **************************************************************** */
void makehb( int irow[], int nnz, int krow[], int m, int * ierr )
{
  int i = 0, k = 0;
  krow[0] = 0;

  while( i < m && k < nnz ) {
    if( irow[k] == i ) {
      /* Element k is in row i */
      k++;
    } else if ( irow[k] > i ) {
      /* Element k belongs to a row greater than i */
      i++;
      krow[i] = k;
    } else { /* irow[k] < i which means the elts weren't sorted */
      *ierr = 1;
      return;
    }
  }
  /* Should have run out of elements in irow */
  if( k != nnz ) {
    *ierr = 1;
    return;
  }
  /* Fill in the remaining rows of krow */
  for( i++; i < m + 1; i++ ) {
    krow[i] = nnz;
  }

  *ierr = 0;
}

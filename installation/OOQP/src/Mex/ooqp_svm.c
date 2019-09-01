/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "mex.h"
#include "mexUtility.h"
#include "OoqpMonitorData.h"
enum TerminationCode 
{
  SUCCESSFUL_TERMINATION = 0,
  NOT_FINISHED,
  MAX_ITS_EXCEEDED,
  INFEASIBLE,
  UNKNOWN
};

#ifdef __cplusplus
extern "C"
#endif
void ooqp_svm( double w[], int n, double * beta, double v[], int m,
	       double z[], double u[], double s[], double Xt[],
	       double d[], double rho, int * status );

/* ****************************************************************
 * mex stub function: ooqp_svm
 * **************************************************************** */
void mexFunction( int nlhs, mxArray * plhs[], int nrhs,
    const mxArray * prhs[] ) {

    double      *dummyDoublePtr;

    double      *X;                       /* Input    argument 0   */
    double      *d;                       /* Input    argument 1   */

    double      *w;                       /* Output   argument 0   */
    mxArray     *mexw;                   
    double       beta;                    /* Output   argument 1   */
    double      *v;                       /* Output   argument 2   */
    mxArray     *mexv;                   
    double      *z;                       /* Output   argument 3   */
    mxArray     *mexz;                   
    double      *u;                       /* Output   argument 4   */
    mxArray     *mexu;                   
    double      *s;                       /* Output   argument 5   */
    mxArray     *mexs;                   

    double       rho;                     /* Optional argument 0   */

    int         m;                        /* Array dimension     */
    int         n;                        /* Array dimension     */

    int         status;

    if ( nrhs < 2 || nrhs > 3  ) {
        mexErrMsgTxt( "Wrong number of input arguments" );
    }
    if ( nlhs > 6 ) {
        mexErrMsgTxt( "Too many output arguments" );
    }

    /* X : Input argument 0 */
    m = mxGetM( prhs[0] );
    n = mxGetN( prhs[0] );
    assertDense( prhs[0], "X" );
    X = mxGetPr( prhs[0] );

    /* d : Input argument 1 */
    assertRowDim( prhs[1], m, "d" );
    assertColDim( prhs[1], 1, "d" );
    assertDense( prhs[1], "d" );
    d = mxGetPr( prhs[1] );

    /* w : output argument 0 */
    mexw = mxCreateDoubleMatrix( n, 1, mxREAL );
    w =  mxGetPr( mexw );

    /* Output argument 1, "beta", is scalar */

    /* v : output argument 2 */
    mexv = mxCreateDoubleMatrix( m, 1, mxREAL );
    v =  mxGetPr( mexv );

    /* z : output argument 3 */
    mexz = mxCreateDoubleMatrix( m, 1, mxREAL );
    z =  mxGetPr( mexz );

    /* u : output argument 4 */
    mexu = mxCreateDoubleMatrix( m, 1, mxREAL );
    u =  mxGetPr( mexu );

    /* s : output argument 5 */
    mexs = mxCreateDoubleMatrix( m, 1, mxREAL );
    s =  mxGetPr( mexs );

    if ( 2 < nrhs ) {
        /* rho : Optional argument 2 */
        assertScalar( prhs[2], "rho" );
        rho =  *mxGetPr ( prhs[2] );
    } else {
      rho = 1;
    }

    if( n > m ) {
      mexErrMsgTxt( "Badly formed problem, X has more columns than rows." );
    }
    
    /* Call function "ooqp_svm" here */
    ooqp_svm( w, n, &beta, v, m, z, u, s, X, d, rho, &status );
    if( 0 != status ) {
      mexErrMsgTxt( "Could not solve the problem. Possibly out of memory." );
    }

    /* Free any scratch arrays */

    /* w : output argument 0 */
    plhs[0] = mexw;

    /* beta : output argument 1 */
    if ( 1 < nlhs ) {
        plhs[1] = mxCreateDoubleMatrix( 1, 1, mxREAL );
        dummyDoublePtr = mxGetPr( plhs[1] );
        *dummyDoublePtr =  beta;
    }

    /* v : output argument 2 */
    if ( 2 < nlhs ) {
        plhs[2] = mexv;
    } else {
        mxDestroyArray( mexv );
    }

    /* z : output argument 3 */
    if ( 3 < nlhs ) {
        plhs[3] = mexz;
    } else {
        mxDestroyArray( mexz );
    }

    /* u : output argument 4 */
    if ( 4 < nlhs ) {
        plhs[4] = mexu;
    } else {
        mxDestroyArray( mexu );
    }

    /* s : output argument 5 */
    if ( 5 < nlhs ) {
        plhs[5] = mexs;
    } else {
        mxDestroyArray( mexs );
    }

}
/* ****************************************************************
 * end of mex stub: ooqp_svm
 * **************************************************************** */


int mexDoMonitor( OoqpMonitorData * data )
{
  switch( data->level ) {
  case 0 : case 1: { 
    mexPrintf( "\nDuality Gap: %g\n", data->gap );
	if( data->i > 1 ) {
	  mexPrintf( " alpha = %g\n", data->alpha );
	}
    mexPrintf( " *** Iteration %d ***\n", data->i );
	mexPrintf( " mu = %9.4e relative residual norm = %9.4e\n",
			   data->mu,
			   data->rnorm / data->dataNorm );

    if( data->level == 1) { 
      /* Termination has been detected by the status check; print
       * appropriate message */
      switch( data->status_code ) {
      case SUCCESSFUL_TERMINATION:
		mexPrintf( "\n *** SUCCESSFUL TERMINATION ***\n" );
		break;
      case MAX_ITS_EXCEEDED:
		mexPrintf( "\n *** MAXIMUM ITERATIONS REACHED *** \n" );
		break;
      case INFEASIBLE:
		mexPrintf( "\n *** TERMINATION: PROBABLY INFEASIBLE *** \n" );
		break;
	  default:
		mexPrintf( "\n *** TERMINATION: STATUS UNKNOWN *** \n" );
		break;
      } /* end switch(statusCode) */
    }
  } break; /* end case 0: case 1: */
  } /* end switch(level) */
  
  return 0;
}

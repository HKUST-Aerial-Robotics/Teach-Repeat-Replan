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
void ooqp_huber( double beta[],    int m, double t[], int n,
		 double lambda1[], double lambda2[],
		 double gamma1[],  double gamma2[], 
		 double Xt[],      double y[],      double cutoff,
		 int * status );

/* ****************************************************************
 * mex stub function: mexFunction
 * **************************************************************** */
void mexFunction( int nlhs, mxArray * plhs[], int nrhs,
    const mxArray * prhs[] ) {

    double      *dummyDoublePtr;

    double      *X;                       /* Input    argument 0   */
    double      *y;                       /* Input    argument 1   */

    double      *beta;                    /* Output   argument 0   */
    mxArray     *mexbeta;                
    double      *t;                       /* Output   argument 1   */
    mxArray     *mext;                   
    double      *gamma1;                  /* Output   argument 2   */
    mxArray     *mexgamma1;              
    double      *gamma2;                  /* Output   argument 3   */
    mxArray     *mexgamma2;              
    double      *lambda1;                 /* Output   argument 4   */
    mxArray     *mexlambda1;             
    double      *lambda2;                 /* Output   argument 5   */
    mxArray     *mexlambda2;             

    double       cutoff;                  /* Optional argument 0   */

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

    /* y : Input argument 1 */
    assertRowDim( prhs[1], m, "y" );
    assertColDim( prhs[1], 1, "y" );
    assertDense( prhs[1], "y" );
    y = mxGetPr( prhs[1] );

    /* beta : output argument 0 */
    mexbeta = mxCreateDoubleMatrix( n, 1, mxREAL );
    beta =  mxGetPr( mexbeta );

    /* t : output argument 1 */
    mext = mxCreateDoubleMatrix( m, 1, mxREAL );
    t =  mxGetPr( mext );

    /* gamma1 : output argument 2 */
    mexgamma1 = mxCreateDoubleMatrix( m, 1, mxREAL );
    gamma1 =  mxGetPr( mexgamma1 );

    /* gamma2 : output argument 3 */
    mexgamma2 = mxCreateDoubleMatrix( m, 1, mxREAL );
    gamma2 =  mxGetPr( mexgamma2 );

    /* lambda1 : output argument 4 */
    mexlambda1 = mxCreateDoubleMatrix( m, 1, mxREAL );
    lambda1 =  mxGetPr( mexlambda1 );

    /* lambda2 : output argument 5 */
    mexlambda2 = mxCreateDoubleMatrix( m, 1, mxREAL );
    lambda2 =  mxGetPr( mexlambda2 );

    if ( 2 < nrhs ) {
        /* cutoff : Optional argument 2 */
        assertScalar( prhs[2], "cutoff" );
        cutoff =  *mxGetPr ( prhs[2] );
    } else {
      cutoff = 1;
    }

    if( m < n ) {
      mexErrMsgTxt( "The number of observations must be at least as large as\n"
		    "the number of predictors.\n" );
    }
    /* Call function "mexFunction" here */
    ooqp_huber( beta, m, t, n,
		lambda1, lambda2, gamma1,  gamma2, 
		X,       y,       cutoff, &status );


    /* Free any scratch arrays */

    /* beta : output argument 0 */
    plhs[0] = mexbeta;

    /* t : output argument 1 */
    if ( 1 < nlhs ) {
        plhs[1] = mext;
    } else {
        mxDestroyArray( mext );
    }

    /* gamma1 : output argument 2 */
    if ( 2 < nlhs ) {
        plhs[2] = mexgamma1;
    } else {
        mxDestroyArray( mexgamma1 );
    }

    /* gamma2 : output argument 3 */
    if ( 3 < nlhs ) {
        plhs[3] = mexgamma2;
    } else {
        mxDestroyArray( mexgamma2 );
    }

    /* lambda1 : output argument 4 */
    if ( 4 < nlhs ) {
        plhs[4] = mexlambda1;
    } else {
        mxDestroyArray( mexlambda1 );
    }

    /* lambda2 : output argument 5 */
    if ( 5 < nlhs ) {
        plhs[5] = mexlambda2;
    } else {
        mxDestroyArray( mexlambda2 );
    }

}
/* ****************************************************************
 * end of mex stub: mexFunction
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

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cQpGenDense.h"
#include "stdio.h"

#define PRINT_LEVEL 0

/**
 * The general quadratic formulation recognized by OOQP is as follows:
 *				min 1/2 x'Qx + c' x subject to
 *					Ax = b, 
 *					d <= Cx <= f,
 *					l <= x <= u
 *
 **/

// nx is the number of primal variables. It is the length of the input vectors
// c, xlow, ixlow, xupp, ixupp, x, gamma, and phi.
const int nx   = 2;

// c is the linear term in the objective function, a vector of length nx
double    c[]  = { 1.5,  -2 };
double    Q[]  = {8, 2, 2, 10};

//xlow, ixlow are the lower bounds on x. These contain the information in the
//lower bounding vector l in the formulation given above. 
//If there is a bound on element k of x (that is, lk > -1), then xlow[k] should
//be set to the value of lk and ixlow[k] should be set to one. 
//Otherwise, element k of both arrays should be set to zero.
double  xlow[] = {  0,   0 };
char   ixlow[] = {  1,   1 };

//xupp, ixupp are the upper bounds on x, that is, the information in the vector
//u in the formulation given above. These should be defined in a similar fashion to 
//xlow and ixlow.
double  xupp[] = { 20,   0 };
char   ixupp[] = {  1,   0 };

// bA contains the right-hand-side vector b for the equality constraints.
// The integer parameter my defines the length of this vector. 
// A contains the coefficients of x for the linear equality constraints.
const int my   = 1;
double A[]	   = {	1,	 1};
double bA[]    = {	1};

// The integer parameter mz defines the number of inequality constraints.
const int mz   = 2;

// clow, iclow are the lower bounds of the inequality constraints.
double clow[]  = {	2,   0 };
char  iclow[]  = {	1,   0 };

// cupp, icupp are the upper bounds of the inequality constraints. 
double cupp[]  = {	0,   6 };
char  icupp[]  = {	0,   1 };

// C contains the coefficients of x for the inequality constraints. 
double   C[]   = { 2,   1,  -1,   2};

int main()
{
  int ierr;

  /* x, y and z are vectors of Lagrange multipliers */ 

  /* double x[nx], gamma[nx], phi[nx]; 
   * gamma and phi contain the multipliers for the lower and upper bounds 
   * x >= l and x <= u, respectively.
   */   
  double x[2], gamma[2], phi[2], ObjVal;

  /* double y[my]; 
   * y contains the multipliers for the equality constraints Ax = b. 
   */
  double y[1]; 

  /* double z[mz], lambda[mz], pi[mz];
   * lambda and pi contain the multipliers for the inequality constraints
   * Cx >= d and Cx <= f, respectively. 
   */
  double z[2], lambda[2], pi[2];
  
  qpsolvede( c, nx, Q, xlow, ixlow, xupp, ixupp,
	         A, my, bA,
	         C, mz, clow, iclow, cupp, icupp,
	         x, gamma, phi,
	         y, 
	         z, lambda, pi, &ObjVal, PRINT_LEVEL, &ierr );

  if( ierr != 0 ) {
    fprintf( stderr, "Couldn't solve it.\n" );
    return 1;
  } else {
    int i;

    printf(" Final Objective: %g\n\n", ObjVal);
    printf(" Solution:...\n" );
    for( i = 0; i < nx; i++ ) {
      printf( "x[%2d] = %g\n", i, x[i] );
    }
    return 0;
  }
}

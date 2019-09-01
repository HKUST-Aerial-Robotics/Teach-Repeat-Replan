/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "cQpGenSparse.h"
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

// Harwell Boeing format:
// In this format, the nonzeros are stored in row-major form, with jcolA[l] and dA[l]
// containing the column index and value of the l nonzero element, respectively. 
// The integer vector krowA[k] indicates the index in jcolA and dA at which the first
// nonzero element for row k is stored; its final element krowA[my+1] points to the 
// index in jcolA and dA immediately after the last nonzero entry.

//krowQ, jcolQ, dQ hold the nnzQ lower triangular elements of the quadratic
//term of the objective function in the Harwell Boeing format.
const int nnzQ = 3;
int    krowQ[] = {  0,   1,   3 }; 
int    jcolQ[] = {  0,   0,   1 };
double    dQ[] = {  8,   2,  10 };

//bA contains the right-hand-side vector b for the equality constraints given
//in the above formulation. The integer parameter my defines the length of this vector.
const int my   = 1;
double bA[]    = {  1  };

//irowA, jcolA, dA are the nnzA nonzero elements of the matrix A of linear
//equality constraints in the Harwell Boeing format.
const int nnzA = 2;
int krowA[]    = {  0,  2};
int jcolA[]    = {  0,  1};
double dA []   = {  1,  1};

/*
//Here is an example of how to specify set up zero equality constraints in the
//Harwell Boeing format.

const int my = 0;
double *bA;
const int nnzA = 0;
int krowA[] = {	 0  };
int *jcolA; 
double *dA;
*/

// The integer parameter mz defines the number of inequality constraints.
const int mz   = 2;

// clow, iclow are the lower bounds of the inequality constraints.
double clow[]  = { 2,   0 };
char  iclow[]  = { 1,   0 };

// cupp, icupp are the upper bounds of the inequality constraints. 
double cupp[]  = { 0,   6 };
char  icupp[]  = { 0,   1 };

//irowC, jcolC, dC are the nnzC nonzero elements of the matrix C of linear
//inequality constraints in the Harwell Boeing format.
const int nnzC = 4;
int   krowC[]  = { 0,   2,   4};
int   jcolC[]  = { 0,   1,   0,   1};
double   dC[]  = { 2,   1,  -1,   2};

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


  qpsolvehb( c, nx,
   	     krowQ, jcolQ, dQ,
	     xlow, ixlow,
	     xupp, ixupp,
	     krowA, my, jcolA, dA,
	     bA,
	     krowC, mz, jcolC, dC,
	     clow, iclow,
	     cupp, icupp,
		 x, gamma, phi,
         y,
	     z, lambda, pi,
	     &ObjVal, PRINT_LEVEL, &ierr);
  
  if( ierr != 0 ) {
    fprintf( stderr, "Couldn't solve it.\n" );
    return 1;
  } else {
    int i;

    printf(" Final Objective: %g\n\n", ObjVal);
    printf( "Solution:...\n" );
    for( i = 0; i < nx; i++ ) {
      printf( "x[%2d] = %g\n", i, x[i] );
    }
    return 0;
  }
}

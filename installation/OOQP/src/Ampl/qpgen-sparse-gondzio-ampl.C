/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <cstdio>
#include <cassert>
#include "asl.h"
#include "getstub.h"
#include "cQpGenSparse.h"
#include "Status.h"

enum { kMinimize = 0, kMaximize = 1 } ;

void doubleLexSort( int first[], int n, int second[], double data[] );
void new_qpgen_variables( double ** x, int nx, double ** gamma, double ** phi,
			  double ** y, int my,
			  double ** z, int mz,
			  double ** lambda, double ** pi, int * ierr );
void free_qpgen_variables( double ** x, double ** gamma, double ** phi,
			   double ** y,  
			   double ** z, double ** lambda, double ** pi );
void  ampl_count_sizes( ASL * asl, fint irow[], fint kcol[],
			int & nx, int & nnzQ,
			int & my, int & nnzA, int & mz, int & nnzC, 
			int * rowMap );
void ampl_get_bounds( ASL * asl, int rowMap[],
		      double xlow[], int nx, char ixlow[],
		      double xupp[], char ixupp[],
		      double b[], int my,
		      double clow[], int mz, char iclow[],
		      double cupp[], char icupp[] );

void ampl_get_c( ASL * asl, double c[], int nx );

void ampl_get_matrices( ASL * asl, fint irow[], fint kcol[], double elts[],
			int rowMap[],
			int nx, int nnzQ,
			int my, int nnzA,
			int mz, int nnzC,
			int irowQ[], int jcolQ[], double dQ[],
			int irowA[], int jcolA[], double dA[],
			int irowC[], int jcolC[], double dC[] );
void ampl_recover_mults( ASL * asl, int rowMap[], double y[], double z[],
			 double mults[] );

static Long lprintLevel = 0;
const int nk = 1;
static keyword keywds[] = {      /* must be in alphabetical order */
  KW((char *) "print_level",    L_val, &lprintLevel,  
	 (char *) "Amount of output" )
};

void negate(double * x, int nx)
{
  for (int i = 0;  i < nx;  i++)
    x[i] = -x[i];
}

int main( int /* argc */, char *argv[])
{
  double  *c     = NULL,  *xlow   = NULL,  *xupp = NULL;
  int      nx;
  int     *irowQ = NULL,  *jcolQ  = NULL;
  int      nnzQ;
  int     *irowA = NULL,  *jcolA  = NULL;
  int      nnzA, my;
  int     *irowC = NULL,  *jcolC  = NULL;
  int      nnzC, mz;

  double  *dQ    = NULL,  *dA     = NULL,  *dC   = NULL;
  double  *b     = NULL,  *clow   = NULL,  *cupp = NULL;

  char    *ixlow = NULL,  *ixupp  = NULL;
  char    *iclow = NULL,  *icupp  = NULL;

  double  *x     = NULL,  *gamma  = NULL,  *phi  = NULL;
  double  *y     = NULL;
  double  *z     = NULL,  *lambda = NULL,  *pi   = NULL;
  int ierr;

  // Allocate the ampl solver library (ASL) context.
  ASL * asl = ASL_alloc( ASL_read_fg );
  // Set up a mostly-empty Option_info structure
  Option_Info oi = { (char*)"OOQP", (char*) "OOQP", (char*)"ooqp_options",
  keywds, nk };
  // Read the stub file containing the problem to solve. 

  // Parse the command line to find the name of the stub file
  char ** av = argv;
  char * stub = getstub(&av, &oi);
  if (!stub) { // Couldn't find the name of the stub file
    printf("Usage: %s stub\n", argv[0]);
    return 1;
  }
  
  if (getopts(av, &oi))
    return 1;
// Open the stub file and initialize the asl data structure
  FILE * nl = jac0dim( argv[1], (fint) strlen( stub ) );
  // Create a double array for the real part of the Jacobian array. 
  // This tells ampl that the constraints *must* be linear. Ampl will
  // create the index arrays A_colstarts and A_rownos automatically.
  A_vals = (double *) malloc( nzc * sizeof( real ) );
  if( NULL == A_vals ) {
    fprintf( Stderr, "Couldn't allocate enough memory.\n" );
    exit( 1 );
  }
  // read the stub file( evidently the read routine closes the file
  // when it is done ). 
  qp_read( nl, 0 );

  // Now read the quadratic Hessian. This will print an error message and
  // quit if the objective is not quadratic (or linear).
  fint *irow, *kcol;
  double * elts;
  int nnzfullQ = qpcheck( &irow, &kcol, &elts );
  if( nnzfullQ == 0 ) {
    // The Q matrix is empty. Ampl doesn't allocate kcol, so all code
    // that would work properly on an empty matrix segfaults. Let's fix
    // that.
    kcol = (fint *) calloc( n_var + 1,  sizeof(fint) );
  }

  // Note that n_con and n_var are MACROS the translate to
  // asl->n_con and asl->n_var. Arghh....

  // Count the problem sizes, and create a map between the rows of the
  // Jacobian and A and C (A is the Jacobian of the equality constraints,
  // C is the Jacobian of the inequality constraints.)
  int * rowMap = new int[n_con];
  ampl_count_sizes( asl, irow, kcol, nx, nnzQ, my, nnzA, mz, nnzC, rowMap );
//    printf( "Number of non-zeros in upper triangle of the Hessian"
//  	  "  : %d\n\n\n", nnzQ );

  assert( my + mz == n_con );
//    printf( "my: %d nnzA %d\n", my, nnzA );
//    printf( "mz: %d nnzC %d\n", mz, nnzC );

  // Allocate the structure of the QP
  newQpGenSparse( &c,     nx,
		  &irowQ, nnzQ,  &jcolQ,  &dQ,
		  &xlow,         &ixlow,  &xupp, &ixupp,
		  &irowA, nnzA,  &jcolA,  &dA,
		  &b,     my,
		  &irowC, nnzC,  &jcolC,  &dC,
		  &clow,  mz,    &iclow,  &cupp, &icupp,
		  &ierr );
  if( ierr != 0 ) {
    fprintf( Stderr, "Couldn't allocate enough memory\n" );
    exit( 1 );
  }
  ampl_get_c( asl, c, nx );
  ampl_get_bounds( asl, rowMap,
		   xlow, nx, ixlow, xupp, ixupp,
		   b, my,
		   clow, mz, iclow, cupp, icupp );
  ampl_get_matrices( asl, irow, kcol, elts, rowMap,
		     nx, nnzQ, my, nnzA, mz, nnzC,
		     irowQ, jcolQ, dQ,
		     irowA, jcolA, dA,
		     irowC, jcolC, dC );

  double objectiveConstant = objconst(0);

  // We no longer need AMPL's version of Q and the Jacobian
  if( irow ) free( irow ); 
  if( kcol ) free( kcol ); 
  if( elts ) free( elts );
  free( A_vals );      A_vals = 0;
  free( A_rownos );    A_rownos = 0;
  free( A_colstarts);  A_colstarts = 0;

  new_qpgen_variables( &x, nx, &gamma, &phi, &y, my,
		       &z, mz, &lambda, &pi, &ierr );
  if( ierr != 0 ) {
    fprintf( Stderr, "Couldn't allocate enough memory\n" );
    exit( 1 );
  }

  if (objtype[0] == kMaximize) {
    negate(c, nx);
    negate(dQ, nnzQ);
  }

  double objectiveValue;
  qpsolvesp( c, nx,  irowQ,  nnzQ,   jcolQ,  dQ,
	     xlow,   ixlow,  xupp,   ixupp,
	     irowA,  nnzA,   jcolA,  dA,
	     b,      my,
	     irowC,  nnzC,   jcolC,  dC,
	     clow,   mz,     iclow,  cupp,   icupp,
	     x,      gamma,  phi,
	     y,      z,      lambda, pi,
	     &objectiveValue,
	     (int) lprintLevel,
	     &ierr );

  if (objtype[0] == kMaximize)
    objectiveValue = -objectiveValue;

  objectiveValue += objectiveConstant;

  freeQpGenSparse( &c, 
		   &irowQ,  &jcolQ,  &dQ,
		   &xlow,   &ixlow,  &xupp,  &ixupp,
		   &irowA,  &jcolA,  &dA,    &b,
		   &irowC,  &jcolC,  &dC,
		   &clow,   &iclow,  &cupp,  &icupp );


//    printf( "And the verdict is...\n" );
//    for( int i = 0; i < nx; i++ ) {
//      printf( "x[%3d] = %g\n", i, x[i] );
//    }
  switch( ierr ) {
  case SUCCESSFUL_TERMINATION: {
    double * mults = new double[n_con];
    ampl_recover_mults( asl, rowMap, y, z, mults );

    char buff[64];
    snprintf( buff, 64, "OOQP completed sucessfully.  "
	      "Objective value: %14.8g.", objectiveValue );
    
    write_sol( buff, x, mults, &oi );
    delete [] mults;
  } break;
  case MAX_ITS_EXCEEDED:
    fprintf( Stderr,
	     "OOQP could not solve the problem. Max iterations exceeded." );
    break;
  case INFEASIBLE:
    fprintf( Stderr,
	     "OOQP could not solve the problem. "
	     "This problem is probably infeasible." );
    break;
  default:
    fprintf( Stderr,
	     "Could not solve the problem, due to an unknown error.\n" );  
    break;
  };

  if( ierr != 0 ) {
    fprintf( Stderr,
	     "Could not solve the problem, due to an unknown error.\n" );
  } else {
  }  free_qpgen_variables( &x, &gamma, &phi, &y, &z, &lambda, &pi );
  delete [] rowMap;

  return 0;
}

////////////////////////////////////////////////////////////////
void new_qpgen_variables( double ** x, int nx, double ** gamma, double ** phi,
			  double ** y, int my, 
			  double ** z, int mz,
			  double ** lambda, double ** pi, int *ierr )
{
  *x = 0; *gamma = 0; *phi = 0;
  *y       = 0;
  *z = 0; *lambda = 0; *pi = 0;

  try {
    *x       = new double[nx];
    *gamma   = new double[nx];
    *phi     = new double[nx];
    if( my > 0 ) {
      *y       = new double[my];
    }
    if( mz > 0 ) {
      *z       = new double[mz];
      *lambda  = new double[mz];
      *pi      = new double[mz];
    }
    *ierr = 0;
  } catch( ... ) {
    free_qpgen_variables( x, gamma, phi, y, z, lambda, pi );
    *ierr = 1;
  }
}

////////////////////////////////////////////////////////////////
void free_qpgen_variables( double ** x, double ** gamma, double ** phi,
			   double ** y,  
			   double ** z, double ** lambda, double ** pi )
{
  delete [] *x;       *x      = 0;
  delete [] *gamma;   *gamma  = 0;
  delete [] *phi;     *phi    = 0;
  delete [] *y;       *y      = 0;
  delete [] *z;       *z      = 0;
  delete [] *lambda;  *lambda = 0;
  delete [] *pi;      *pi     = 0;
}

////////////////////////////////////////////////////////////////
void  ampl_count_sizes( ASL * asl, fint irow[], fint kcol[],
			int & nx, int & nnzQ,
			int & my, int & nnzA, int & mz, int & nnzC, 
			int * rowMap )
{
  int i, j, k;

  // Our internal data structures need only a triangle of Q.  Count
  // the number of non-zeros in this triangle.
  nnzQ = 0;
  for( j = 0; j < n_var; j++ ) {
    for( k = kcol[j]; k < kcol[j+1]; k++ ) {
      i = irow[k];
      // count only the upper triangle
      if( i > j ) break;
      nnzQ++;
    }
  }
  
  // Count the number of rows in A and C and create a map between the 
  // rows of ampl's version of the Jacobian, and the rows of A and C.
  nx = n_var;
  my =0; mz = 0;

  // Count the number of nonzeros in A, the Jacobian of the equality
  // constraints, and the number of nonzeros in C, the Jacobian of the
  // inequality constraints.
  nnzA = 0; nnzC = 0;
  double (*Fbnds)[2] = (double(*)[2]) LUrhs;
  for( i = 0; i < n_con; i++ ) {
    double lowerb = Fbnds[i][0];
    double upperb = Fbnds[i][1];
    if ( lowerb == upperb ) {
      rowMap[i] = - (my + 1); // Negative values indicate an equality
      my++;
    } else {
      rowMap[i] = mz;
      mz++;
    }
  }
  for( j = 0; j < n_var; j++ ) {
    for( k = A_colstarts[j]; k < A_colstarts[j+1]; k++ ) {
      int iampl = A_rownos[k];
      if ( rowMap[iampl] < 0 ) {
	nnzA++;
      } else {
	nnzC++;
      }
    }
  }
}


////////////////////////////////////////////////////////////////
void ampl_get_c( ASL * asl, double c[], int nx )
{
  for( int j = 0; j < nx; j++ ) c[ j ] = 0;
  ograd * og;
  for( og = Ograd[0]; og; og = og->next ) {
    assert( og->varno < nx );
    c[og->varno] = og->coef;
  }
}

////////////////////////////////////////////////////////////////
void ampl_get_bounds( ASL * asl, int rowMap[],
		      double xlow[], int nx, char ixlow[],
		      double xupp[], char ixupp[],
		      double b[], int /* my */,
		      double clow[], int /* mz */, char iclow[],
		      double cupp[], char icupp[] )
{
  double (*xbnd)[2] = (double (*)[2]) LUv;
  // Get xlow, xupp
  for( int i = 0; i < nx; i++ ) {
    double lowerb = xbnd[i][0];
    double upperb = xbnd[i][1];
    if( lowerb > -1e20 ) {
      xlow[i]  = lowerb;
      ixlow[i] = 1;
    } else {
      xlow[i] = 0; ixlow[i] = 0;
    }
    if( upperb < 1e20 ) {
      xupp[i]  = upperb;
      ixupp[i] = 1;
    } else {
      xupp[i] = 0; ixupp[i] = 0;
    }
  }
  // Get b, clow, cupp
  double (*Fbnds)[2] = (double (*)[2]) LUrhs;
  for( int iampl = 0; iampl < n_con; iampl++ ) {
    double lowerb = Fbnds[iampl][0];
    double upperb = Fbnds[iampl][1];
    if( lowerb == upperb ) {
      int i = - ( rowMap[iampl] + 1 ); // Recover i from the negative value
      // in rowMap

      b[i] = lowerb;
    } else {
      int i = rowMap[iampl];

      if( lowerb > -1e20 ) {
	clow[i]  = lowerb;
	iclow[i] = 1;
      } else {
	clow[i] = 0; iclow[i] = 0;
      }
      if( upperb < 1e20 ) {
	cupp[i]  = upperb;
	icupp[i] = 1;
      } else {
	cupp[i] = 0; icupp[i] = 0;
      }
    }
  }
}

////////////////////////////////////////////////////////////////
void ampl_get_matrices( ASL * asl, fint irow[], fint kcol[], double elts[],
			int rowMap[],
			int /* nx */, int nnzQ,
			int /* my */, int nnzA,
			int /* mz */, int nnzC,
			int irowQ[], int jcolQ[], double dQ[],
			int irowA[], int jcolA[], double dA[],
			int irowC[], int jcolC[], double dC[] )
{			
  int i, j, k;
  // Q
  int kQ = 0;
  for( j = 0; j < n_var; j++ ) {
    for( k = kcol[j]; k < kcol[j+1]; k++ ) {
      i = irow[k];
      // use only the lower triangle, but transpose
      if( i <= j ) {
	assert( kQ < nnzQ );
	irowQ[kQ] = j; jcolQ[kQ] = i; dQ[kQ] = elts[k];
	kQ++;
      }
    }
  }

  int kA = 0, kC = 0;
  for( j = 0; j < n_var; j++ ) {
    for( k = A_colstarts[j]; k < A_colstarts[j+1]; k++ ) {
      int iampl = A_rownos[k];
      if ( rowMap[iampl] < 0 ) { // A negative value in rowMap indicates
	// and equality constraint.
	i = - ( rowMap[iampl] + 1 );

	assert( kA < nnzA );
	irowA[kA] = i;  jcolA[kA] = j;  dA[kA] = A_vals[k];

	kA++;
      } else {
      i = rowMap[iampl];

	assert( kC < nnzC );
	irowC[kC] = i;  jcolC[kC] = j;  dC[kC] = A_vals[k];

	kC++;
      }
    }
  }

  // Transpose everything
  doubleLexSort( irowA, nnzA, jcolA, dA );
  doubleLexSort( irowC, nnzC, jcolC, dC );
}

////////////////////////////////////////////////////////////////
void ampl_recover_mults( ASL * asl, int rowMap[], double y[], double z[],
			 double mults[] )
{
  for( int iampl = 0; iampl < n_con; iampl++ ) {
    if( rowMap[iampl] < 0 ) { // This is an equality
      int i = -( rowMap[iampl] + 1 );
      mults[iampl] = y[i];
    } else {
      int i = rowMap[iampl];
      mults[iampl] = z[i];
    }
  }
}

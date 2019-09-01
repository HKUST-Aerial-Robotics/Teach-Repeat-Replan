/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/* Mike Gertz 2-Aug-98 */
/* Correction: 3-Oct-98 */

#include "mexUtility.h"

int iequal( int * a, int * b, int n )
{ 
  int i;
  for ( i = 0; i < n; i++ ) {
	if ( a[i] != b[i] ) break;
  }
  return i == n;
}

/* **************************************************************** *
 * void int2integer
 * **************************************************************** */
void int2integer( int n, int * i, int * ii )
{
  int j;
  for ( j = 0; j < n; j++ ) {
	ii[j] = i[j];
  }
}

/* ****************************************************************
 * void jc2cols
 * **************************************************************** */
void jc2cols( int n, int * jc, int * cols )
{
  int i, j;

  for ( i = 0; i < n; i++ ) {
	/* mexPrintf( "%d\t", jc[i] ); */
 	for( j =  jc[i]; j < jc[i+1]; j++ ) { 
 	  cols[j] = i + 1; 
    }
  }
  /*  mexPrintf( "%d\n", jc[n] ); */
}
	
/* **************************************************************** *
 * void mx2char
 *
 * Converts a matrix of mxChar with dimension rows * lenMexStr to a
 * matrix of char with dimension rows * cols. The input matrix is in
 * column major order, but the output matrix is in row-major order. If
 * lenMexStr > cols then only the first cols columns of mexstr will be
 * copied into string. If lenMexStr < cols the rows of string will be
 * padded with blanks (character ' ').
 *
 * None of the rows of string[] will be null-terminated! 
 * **************************************************************** */
void mx2char ( int rows, int cols, int lenMexStr,
			   mxChar * mexstr, char string[] )
{
  int i,j;

  if ( 1 == rows ) {
	for( j = 0; j < lenMexStr; j++ ) {
	  string[j] = (char) mexstr[j];
	}
	for( ; j < cols; j++ ) {
	  string[j] = ' ';
	}
  } else {
	/* The characters in dp are in column-major order */
	/* but the ones in string are row-major */
	char * sp = string;
	for( i = 0; i < rows; i++ ) {
	  mxChar * mp = &mexstr[i];
	  for ( j = 0; j < lenMexStr; j++, sp++, mp += rows ) {
		/* 		string[i*cols + j] = (char) mexstr[j*rows + i]; */
		*sp = (char) *mp;
	  } /* end for j */
	  for( /* j */; j < cols; j++, sp++ ) {
		/* 		string[i*cols + j] = ' '; */
		*sp = ' ';
	  }
	} /* end for i */
  } /* end else */
}

/* **************************************************************** *
 * void dble2int ( int n, double * da, int * ia )
 *     Copy n items from the array double precision array da
 *     to the int array ia.
 * **************************************************************** */
void dble2int ( int n, double * da, int * ia )
{
	int i;
	if ( sizeof(int) <= sizeof(double) ) {
		for ( i = 0; i < n; i++ ) {
			ia[i] = (int) da[i];
		}
	} else {
		for ( i = n - 1; i >= 0; i-- ) {
			ia[i] = (int) da[i];
		}
	}
}

/* **************************************************************** *
 * void int2dble( int n, int * ia, double * da )
 *     Copy n items from the int array ia to the double precision
 *     array da.
 * **************************************************************** */ 
void int2dble( int n, int * ia, double * da )
{
	int i;
	if ( sizeof(int) <= sizeof(double) ) {
	  for ( i = n - 1; i >= 0; i-- ) {
		da[i] = ia[i];
	  }
	} else {
	  for ( i = 0; i < n; i++ ) {
		da[i] = ia[i];
	  }
	}  
}

/* ****************************************************************
 * static void errNotString( char name[8] )
 *     The argument with name "name" should have contained a string,
 *     but did not. Print an error message and abort the mex
 *     function. 
 *
 *     This function is only used by assertString(), and exists 
 *     to keep assertString as lightweight as possible. assertString
 *     is also a good candidate for conversion to an inline function
 *     or macro.
 * **************************************************************** */
static void errNotString( char name[8], int maxl )
{
  char errStr[24];

  sprintf( errStr, 
		   "'%.8s' must be a dense string or a dense array of strings\n"
		   "of length at most %d.\n", name, maxl );
  mexErrMsgTxt( errStr );
}

/* ****************************************************************
 * void assertString( const mxArray * mex, char name[8] )
 *     If "mex" does not contain the MATLAB representation of a string,
 *     prints an error message and aborts the mex function. "name" is
 *     a symbolic name for the argument that should have contained a
 *     string. "name" will be used in displaying the error message.
 * **************************************************************** */
void assertString( const mxArray * mex, int maxl, char name[8] )
{
  if( !mxIsChar( mex ) || mxGetN( mex ) > maxl || mxIsSparse( mex ) ) {
    errNotString( name, maxl );
  }
}

/* **************************************************************** *
 * static void errNotScalar( char name[8] )
 *     The argument with name "name" should have contained a scalar
 *     value, but did not. Print an error message and abort the mex
 *     function. 
 * **************************************************************** */
static void errNotScalar( char name[8] )
{
  char errStr[40];
  
  sprintf ( errStr, "Parameter '%.8s' must be a dense scalar", name );
  mexErrMsgTxt( errStr );
}

/* **************************************************************** *
 * void assertScalar( const mxArray * mex, char name[8] )
 *     If "mex" does not contain the MATLAB representation of a scalar
 *     value, prints an error message and aborts the mex
 *     function. "name" is a symbolic name for the argument that
 *     should have contained a scalar value. "name" will be used in
 *     displaying the error message.
 * **************************************************************** */
void assertScalar( const mxArray * mex, char name[8] )
{
  if ( 1 != mxGetM( mex ) || 1 != mxGetN( mex ) || mxIsSparse( mex ) ) {
    errNotScalar( name );
  }
}

/* **************************************************************** *
 * static void errRowDim( char name[8] )
 *     The argument with name "name" should have had dim rows, but
 *     instead had m rows. Display an error message and abort the mex
 *     function.
 * **************************************************************** */

static void errRowDim( int dim, int m, char name[8] )
{
  char errStr[64];

  sprintf( errStr, "Expected %4d rows for parameter '%.8s'. Got %4d",
	   dim, name, m );
  mexErrMsgTxt( errStr );
}

/* **************************************************************** *
 * void assertRowDim( const mxArray * mex, int dim, char name[8] ) 
 *     If "mex" does not contain a matrix with "dim" rows, prints an
 *     error message and aborts the mex function. "name" is a symbolic
 *     name for the argument that should have contained a scalar
 *     value. "name" will be used in displaying the error message.
 * **************************************************************** */
void assertRowDim( const mxArray * mex, int dim, char name[8] ) 
{
  int m;

  m = mxGetM( mex );
  if ( dim != m ) {
    errRowDim( dim, m, name );
  }
}

/* **************************************************************** *
 * static void errColDim( int dim, int m, char name[8] )
 *     See errRowDim()
 * **************************************************************** */
static void errColDim( int dim, int m, char name[8] )
{
  char errStr[64];

  sprintf( errStr, "Expected %4d columns for parameter '%.8s'. Got %4d",
	   dim, name, m );
  mexErrMsgTxt( errStr );
}

/* **************************************************************** *
 * void assertColDim( const mxArray * mex, int dim, char name[8] )
 *     See assertRowDim()
 * **************************************************************** */
void assertColDim( const mxArray * mex, int dim, char name[8] )
{
  int n;
  n = mxGetN( mex );
  if ( dim != n ) {
    errColDim( dim, n, name );
  }
}

/* **************************************************************** *
 * static void errNotSparse( char name[8] )
 *     The argument with name "name" should have contained a sparse
 *     value, but did not. Print an error message and abort the mex
 *     function. 
 * **************************************************************** */
static void errNotSparse( char name[8] )
{
  char errStr[64];
  sprintf( errStr, "%.8s must be a sparse matrix.\n", name );

  mexErrMsgTxt( errStr );
}

/* ****************************************************************
 * void assertSparse( const mxArray * mex, char name[8] )
 *     If "mex" does not contain the MATLAB representation of a sparse
 *     array, prints an error message and aborts the mex
 *     function. "name" is a symbolic name for the argument that
 *     should have contained a sparse array. "name" will be used in
 *     displaying the error message.
 * **************************************************************** */
void assertSparse( const mxArray * mex, char name[8] )
{
  if ( !mxIsSparse( mex ) ) errNotSparse( name );
}

/* **************************************************************** *
 * static void errNotDense( char name[8] )
 *     The argument with name "name" should have contained a dense
 *     value, but did not. Print an error message and abort the mex
 *     function. 
 * **************************************************************** */
static void errNotDense( char name[8] )
{
  char errStr[64];
  sprintf( errStr, "%.8s must be a dense matrix.\n", name );

  mexErrMsgTxt( errStr );
}

/* ****************************************************************
 * void assertDense( const mxArray * mex, char name[8] )
 *     If "mex" does not contain the MATLAB representation of a dense
 *     array, prints an error message and aborts the mex
 *     function. "name" is a symbolic name for the argument that
 *     should have contained a Dense array. "name" will be used in
 *     displaying the error message.
 * **************************************************************** */
void assertDense( const mxArray * mex, char name[8] )
{
  if ( mxIsSparse( mex ) ) errNotDense( name );
}
/* **************************************************************** *
 * int  overlaySparse( int m, int n, 
 *                     int nza, int ira[], int jca[], double a[],
 *   				   int nzb, int irb[], int jcb[], double b[] )
 *
 * Copy the sparse matrix a to b. The sparsity structure of a should be 
 * a subset of the sparsity structure of b. Wherever there is an element
 * in b without a corresponding element in a, that element of b will be 
 * set to 0.0.
 *
 * The innermost loop has been hand-optimized (try not to laugh too hard)
 * **************************************************************** */

int overlaySparse( int m, int n, int nza, int ira[], int jca[], double a[],
					int nzb, int irb[], int jcb[], double b[] )
{
  int j, indexa, indexa_jp1, indexb, indexb_jp1, ia, ib;
  int *pira = ira, *pirb = irb; /* Baby the compiler by not modifying */
  double *pa = a, *pb = b;      /* parameters.  This can encourage the */
  int *pjca = jca, *pjcb = jcb; /* use of registers */
  enum {Ok = 0, structureChanged = 1};

  if ( nza > nzb ) return structureChanged;


  indexa = jca[0];
  indexb = jcb[0];

  for ( j = 0; j < n; j++ ) {
	/* indexa = jca[j] - assignment unnecessary, already true */
	/* indexb = jcb[j] - assignment unnecessary, already true */
	indexa_jp1 = *++pjca; /* indexa_jp1 = jca[j+1]; */
	indexb_jp1 = *++pjcb; /* indexb_jp1 = jcb[j+1]; */
	if ( indexa_jp1 - indexa > indexb_jp1 - indexb ) {
	  /* There are more non-zeros in a(:,j) than in b(:,j), */
	  /* which is not allowed */
	  return structureChanged;
	}
	for ( /* indexa and indexb initialized above */;
		  indexa < indexa_jp1; /* indexa_jp1 = jca[j+1] */
		  indexa++, indexb++, pira++, pirb++, pa++, pb++ ) {

	  ia = *pira; /* ia = ira[indexa]; */
	  ib = *pirb; /* ib = irb[indexb]; */
/* 	  mexPrintf( "in loop indexa: %d indexb: %d\n", indexa, indexb); */
/* 	  mexPrintf( "a(%d, %d) b(%d, %d)\n", ia, j, ib, j); */
	  

	  if ( ib != ia ) {
		/* ib == ia is by far the most common case, so we test */
		/* that first. */		
        /* When it doesn't occur, it means that there are */
		/* "accidental" zeros which need to be included in b. */
		/* These are entries whose value happens to be zero although */
		/* the sparsity structure would allow them to be non-zero. */
		/* MATLAB doesn't allow accidental zeros in its sparse matricies */

		if ( ib > ia ) return structureChanged;
		/* ib can't be greater than ia, because b(:,j) can only have MORE */
		/* non-zeros than a(:,j) (counting accidental zeros as non-zeros) */
		while( ib < ia ) { 
		  /* Loop over entries in the structure of b(:,j) which are not */
		  /* in the structure of a(:,j) */
		  
		  /* Add an "accidental" zero to b */
		  *pb = 0.0; pb++; indexb++; pirb++; /* b[indexb++] = 0.0; */
		  if ( indexb >= jcb[j+1] ) return structureChanged;
		  ib = *pirb; /* ib = irb[indexb]; */
/* 		  mexPrintf( "in loop indexa: %d indexb: %d\n", indexa, indexb); */
/* 		  mexPrintf( "a(%d, %d) b(%d, %d)\n", ia, j, ib, j); */
		} /* end while ( ib > ia ) */
	  } /* End if ( ib != ia ) */
	  *pb = *pa; /* b[indexb] = a[indexa]; */
	} /* End for( indexa = ... */
	/* Any remaining entries in b(:,j) not in a(:,j) are accidental zeros */
	/* Add them in. */
	for( ; indexb < indexb_jp1; indexb++, pb++, pirb++ ) {
/* 	  mexPrintf( "past loop indexa: %d indexb: %d\n", indexa, indexb); */
	  *pb = 0.0;; /* b[indexb] = 0.0; */
	}
  } /* End for ( j = 0; ... */
  return Ok;
} /* end function */



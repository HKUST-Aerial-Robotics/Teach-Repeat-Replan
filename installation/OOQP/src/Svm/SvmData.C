/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "SvmData.h"
// #include "math.h"
#include <cstring>
#include <cstdio>
#include <iostream>
#include <fstream>
using namespace std;

#include <cassert>

// include for IO purposes
#include <fstream>

#include "DenseGenMatrix.h"
#include "SimpleVector.h"
#include "LinearAlgebraPackage.h"   

//define a constructor

SvmData::SvmData(int hyperplanedim_in, int nobservations_in, 
		 double penalty /* = 1.0 */ )
{
  hyperplanedim = hyperplanedim_in;
  nobservations = nobservations_in;
  mPenalty      = penalty;

  assert ( nobservations_in >= hyperplanedim_in);
  assert ( nobservations_in >= 2);
  assert ( hyperplanedim_in >= 1);

  Yt = DenseGenMatrixHandle( new DenseGenMatrix(hyperplanedim,
						    nobservations ) );
  categories = SimpleVectorHandle( new SimpleVector( nobservations ) );
}

SvmData::SvmData(int hyperplanedim_in, int nobservations_in,
		 double * X_in, double * d,
		 double penalty /* = 1.0 */ )
{
  hyperplanedim = hyperplanedim_in;
  nobservations = nobservations_in;
  mPenalty      = penalty;

  assert ( nobservations_in >= hyperplanedim_in);
  assert ( nobservations_in >= 2);
  assert ( hyperplanedim_in >= 1);

  Yt = DenseGenMatrixHandle( new DenseGenMatrix( X_in, hyperplanedim,
						     nobservations ) );
  categories = SimpleVectorHandle( new SimpleVector( d, nobservations ) );
}

/* The textinput routine is called when we read input from a file
 * 
 * The file must start with two integers that denote
 *
 * - nobservations : number of observations
 * - hyperplanedim : hyperplane dimension (that is, the size of each vector)
 *
 * must satisfy nobservations >= 2, hyperplanedim >= 1, nobservations
 * >= hyperplanedim.
 *
 * Then for each observation one must list the "hyperplanedim" entries
 * in each vector, followed by a label for that vector. The labels
 * must take two distinct values (not necessarily +1 and -1).  */

SvmData * SvmData::textInput( char filename[], double penalty, int& iErr)
{
  int i;
  int nobs_file, ndim_file;
  double label1, label2;
  FILE * file;
  SvmData * result;

  // assume that the input will be OK
  iErr = svminputok;

  //open the input file; bomb out if it can't be found
  file = fopen( filename, "r");
  if(!file) {
    fprintf( stderr, " textInput: Error reading %s: fopen failed\n", filename);
    iErr = svmfileopenerror;
    return 0;
  }

  // read dimensions : first the number of observations
  if ( fscanf(file, "%d", &nobs_file) != 1) {
    fprintf( stderr, " textInput: Error reading %s:"
	     " couldn't find nobservations\n",  filename);
    iErr = svmfileinputerror;
    return 0;
  }
  if ( fscanf(file, "%d", &ndim_file) != 1) {
    fprintf( stderr, " textInput: Error reading %s:"
	     " couldn't find hyperplanedim\n", filename);
    iErr = svmfileinputerror;
    return 0;
  }
  if(nobs_file <= 1) {
    fprintf( stderr, " Error reading %s:"
	    " Must have at least two observations\n", filename);
    fprintf( stderr, " nobservations=%d\n", nobs_file);
    iErr = svmfileinputerror;
    return 0;
  }
  if(ndim_file <= 0) {
    fprintf( stderr, " Error reading %s:"
	     " Data space dimension must be at least 1\n", filename);
    fprintf( stderr, " hyperplanedim=%d\n", ndim_file);
    iErr = svmfileinputerror;
    return 0;
  }
  // create a data structure with these dimensions
  result = new SvmData(ndim_file, nobs_file, penalty );          
  SimpleVector & categories = *result->categories;
            
  // and temporary storage for each row of the matrix;
  { 
    SimpleVectorHandle hTempRowY( new SimpleVector( result->hyperplanedim ) );
    SimpleVector & tempRowY = *hTempRowY;
    
    // read the matrix and labels
    for(i=0; i<result->nobservations; i++) {

      // read a row
      for(int j=0; j<result->hyperplanedim; j++) {
	if(fscanf(file, "%le", &tempRowY[j]) != 1) {
	  fprintf( stderr, " Error reading %s at observation %d\n",
		   filename, i);
	  fprintf( stderr, " Apparently too little data in the file\n");
	  iErr = svmfileinputerror;
	  return 0;
	}
      }
      // stuff the row into a column of Yt
      result->Yt->atPutDense(0, i, &tempRowY[0], 1, result->hyperplanedim, 1);

      //read the right-hand side element
      {
	double tlabel;
	if(fscanf(file, "%le", &tlabel) != 1) {
	  fprintf( stderr,
		   " Error reading %s at observation %d\n", filename, i);
	  fprintf( stderr, " Apparently too little data in the file\n");
	  iErr = svmfileinputerror;
	  return 0;
	}
	categories[i] = tlabel;
      }
    } // end for "read the matrix
  } // end scope of tempRowY
  fclose(file);

  // now check that there are just two distinct labels;

  label1 = categories[0];
  for (i = 0;
	 i < result->nobservations && categories[i] == label1;
	 i++ ) 
    ;

  if(i < result->nobservations) 
    label2 = categories[i];
  else {
    // only found one label
    fprintf(stderr, " Error reading %s: Found only one label: %g\n",
	    filename, label1);
    iErr = svmlabelerror;
    return 0;
  }

  for(i=0; i<result->nobservations; i++) {
    if(categories[i] != label1 && categories[i] != label2) {
      // found a third label
      fprintf( stderr, " Error reading %s: "
	       "Found more than two labels: %g, %g, %g,...\n", 
	       filename, label1, label2, categories[i]);
      iErr = svmlabelerror;
      return 0;
    }
  }

  //printf(" Found exactly two labels: %g, %g\n", label1, label2);

  // Now, make the two labels be exactly 1, and -1
  // Make the greater of the two labels be 1 and the lesser -1.
  if( label1 < label2 ) {
    double temp;
    temp = label1; label1 = label2; label2 = temp;
  }

  for(i=0; i<result->nobservations; i++) {
    if(categories[i] == label2) {
      categories[i] = -1.0;
    } else {
      categories[i] =  1.0;
    }
  }
  return result;
}

SvmData::~SvmData()
{
}

// calculate the norm of the data for the SvmData class

double SvmData::datanorm()
{
  return Yt->abmaxnorm();
}

  
void SvmData::YMult( double beta, SimpleVector& y,
		     double alpha, SimpleVector& x )
{
  SimpleVectorHandle svtemp( new SimpleVector(nobservations) );
  SimpleVector & temp = *svtemp;

  Yt->transMult( 0, temp, alpha, x );
  temp.componentMult( *categories );

  int i;
  for( i = 0; i < nobservations; i++ ) {
    y[i] = temp[i] + beta * y[i];
  }
}

void SvmData::YTransMult( double beta, SimpleVector& y,
			  double alpha, SimpleVector& x )
{
  // Probably resonable here to use a temp.
  SimpleVectorHandle svtemp( new SimpleVector( nobservations) );
  SimpleVector & temp = *svtemp;

  temp.copyFrom(x);
  temp.componentMult(*categories);

  Yt->mult( beta, y, alpha, temp );
}

void SvmData::datarandom()
{
  double drand(double *), ix;
  int i;

  ix = 89176823.0;

  // fill out the matrix Y with random numbers
  Yt->randomize( -1.0, 1.0, &ix );
  categories->randomize( -1.0, 1.0, &ix );
  // set elements of the "categories" to -1 or 1
  double * pcat = &(*categories)[0];
  for(i=0; i<nobservations; i++) {
    if( pcat[i] < 0 ) {
      pcat[i] = -1.0;
    } else {
      pcat[i] =  1.0;
    }
  }
}

double SvmData::dotCategories( SimpleVector & v )
{
  return categories->dotProductWith( v );
}







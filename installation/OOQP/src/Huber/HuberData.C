/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "HuberData.h"
// #include "math.h"
#include <cstring>
#include <iostream>
#include <fstream>
using namespace std;

#include <cassert>
#include <cstdio>

// include for IO purposes
#include "DenseGenMatrix.h"
#include "SimpleVector.h"

//define a constructor

HuberData::HuberData(int nobservations_in, int npredictors_in,
		     double cutoff_in, double * dXt, double * dY) :
  cutoff(cutoff_in), nobservations(nobservations_in),
  npredictors(npredictors_in)
{
  assert( nobservations_in >= npredictors_in );

  m = nobservations+npredictors;
  n = 2*nobservations;
  if( dXt ) {
    Xt = DenseGenMatrixHandle( new DenseGenMatrix( dXt,
						   npredictors,
						   nobservations ) );
  } else {
    Xt = DenseGenMatrixHandle( new DenseGenMatrix( npredictors,
						   nobservations ) );
  }
  if( dY ) {
    Y = SimpleVectorHandle( new SimpleVector( dY, nobservations ) );
  } else {
    Y = SimpleVectorHandle( new SimpleVector( nobservations ) );
  }
}


// this one called when we choose to read input from a file

HuberData * HuberData::textInput( char filename[], double cutoff, int& iErr)
{
  int nobservations_file, npredictors_file;
  FILE * file;
  HuberData * result;

  // assume that the input will be OK  
  iErr = huberinputok;

  //open the input file; bomb out if it can't be found
  file = fopen( filename, "r");
  if(!file) {
    cerr << " textInput: Error reading " << filename
	 << " fopen failed\n";
    iErr = huberfileopenerror;
    return 0;
  }
  
  // read dimensions (first two integers in the input file)
  if ( fscanf(file, "%d", &nobservations_file) != 1) {
    cerr << " textInput: Error reading " << filename
	 << " : couldn't find nobservations\n";
    iErr = huberfileinputerror;
    return 0;
  }
  if ( fscanf(file, "%d", &npredictors_file) != 1) {
    cerr << " textInput: Error reading " << filename 
	 << ": couldn't find npredictors\n";
    iErr = huberfileinputerror;
    return 0;
  }
  if(nobservations_file <= 0 || npredictors_file <= 0) {
    cerr << " Error reading " << filename
	 << ": Dimensions of data matrix must be positive\n";
    cerr << " nobservations=" << nobservations_file
	 << " npredictors="   <<  npredictors_file << endl;
    iErr = huberfileinputerror;
    return 0;
  }
  
  // create data structure with these dimensions
  result = new HuberData(nobservations_file, npredictors_file, cutoff);
  SimpleVector & resultY = *result->Y;

  // and temporary storage for each row of the matrix;
  SimpleVectorHandle hTempRowX( new SimpleVector(result->npredictors) );
  SimpleVector & tempRowX = *hTempRowX;
  
  // read the matrix and right-hand side
  for(int i=0; i<result->nobservations; i++) {

    //read a row
    for(int j=0; j<result->npredictors; j++) {
      if(fscanf(file, "%le", &tempRowX[j]) != 1) {
	iErr = huberfileinputerror;
	cerr << "Not enough data in file: " << filename << endl;
	return 0;
      }
    }
    // stuff the row into X (i.e. stuff column i of Xt)
    result->Xt->atPutDense(0, i, &tempRowX[0], 1, result->npredictors, 1);

    //read the right-hand side element
    if(fscanf(file, "%le", &resultY[i] ) != 1) {
      iErr = huberfileinputerror;
      cerr << "Not enough data in file: " << filename << endl;
      return 0;
    }
  }
  fclose(file);
  return result;
}

//  the destructor

HuberData::~HuberData()
{
}

// calculate the norm of the data for the HuberData class

double HuberData::datanorm()
{
  double norm = Xt->abmaxnorm();
  double temp = Y->infnorm();

  return (norm > temp) ? norm : temp;
}

void HuberData::XtMult( double beta, SimpleVector& y,
			double alpha, SimpleVector& x )
{
  Xt->mult( beta, y, alpha, x );
}

void HuberData::XtTransMult( double beta, SimpleVector& y,
			     double alpha, SimpleVector& x )
{
  Xt->transMult( beta, y, alpha, x );
}

void HuberData::datarandom()
{
  double drand(double *), ix;
  int i;

  ix = 89176823.0;

  SimpleVectorHandle svbeta( new SimpleVector( npredictors ) );
  SimpleVector & beta = *svbeta;

  // assign a value for "cutoff" (a critical parameter)
  cutoff=1.0;

  // fill out the matrix Xt

  Xt->randomize( -1.0, 1.0, &ix );

  // the dummy solution OoqpVector beta
  for(i=0; i<npredictors; i++)
    beta[i] = drand(&ix);

  // the right-hand side Y. Find X* beta
  this->XtTransMult(0.0, *Y, 1.0, beta);
  // now add some random noise
  double * pY = Y->elements();
  for(i=0; i<nobservations; i++) {
	pY[i] += 4.0*cutoff*(drand(&ix) - 0.5);
  }

  return;
}


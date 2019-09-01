/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SVMVARS_H
#define SVMVARS_H

#include "SimpleVectorHandle.h"
#include <iostream>
#include <fstream>
using namespace std;

#include "Variables.h"

/**
 * @ingroup Svm
 *
 * Variables class for Svm.
 *
 */

class SvmVars : public Variables
{
public:

  /** dimension of Euclidean space in which each observation resides */
  int hyperplanedim;

  /** number of observations */
  int nobservations;

  /** vectors and the scalar that make up the Svm variables; see the
      paper by E. M. Gertz and S. J. Wright for details of the
      formulation */
  SimpleVectorHandle z, w, u, v, s;
  double beta;

  /** constructor to make a Svm variables object of specified
      dimensions */
  SvmVars(int hyperplanedim_in, int nobservations_in);

  /** constructor to make a Svm variables object of specified
      dimensions for which storage has already been allocated for the
      five individual vector components */
  SvmVars(int hyperplanedim_in, int nobservations_in,
	  double w[], double v[], double z[], double u[], double s[] );

  /** destructor for Svm variables object */
  virtual ~SvmVars();

  /** performs saxpy operation: add alpha*b to current variables */
  virtual void saxpy( Variables *b, double alpha );

  virtual void negate();

  virtual double mu();

  virtual double mustep(Variables *step, double alpha);

  virtual double stepbound( Variables *b );

  virtual void interiorPoint( double alpha, double beta );
  virtual void shiftBoundVariables( double alpha, double beta );
  virtual double violation();

  virtual void print();

  /** print only the interesting parts of the SVM variable vector
      (that is, the coefficient that define the separating hyperplane) */
  virtual void printCoefs();

  virtual void copy(Variables *b);

  virtual double findBlocking( Variables * step, 
			       double & primalValue,
			       double & primalStep,
			       double & dualValue,
			       double & dualStep,
			       int& firstOrSecond );

  /** write current set of variables to given output stream */
  virtual void asMfile( ostream& os );

  virtual double onenorm();
  virtual double infnorm();
};

#endif





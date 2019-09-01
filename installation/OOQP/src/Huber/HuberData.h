/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef HuberDATA_H
#define HuberDATA_H

#include "Data.h"
#include "DenseGenMatrixHandle.h"
#include "DenseSymMatrixHandle.h"
#include "SimpleVectorHandle.h"

/**
 * @ingroup Huber
 *
 * Data class for Huber.
 *
 */

class HuberData : public Data
{
 public:

  /** dimensions of the equivalent LCP reformulation of the Huber
      regression problem */
  int m, n;

  /** make data object using data structures already allocated */
  HuberData(int nobservations_in, int npredictors_in,  double cutoff,
	    double * X = 0, double * y = 0);

  /** destructor */
  ~HuberData();

  /** store the transpose of the matrix X as a dense
      matrix. Dimensions of Xt are npredictors x nobservations */
  DenseGenMatrixHandle Xt;

  /** right-hand side (targets) */
  SimpleVectorHandle Y;

  /** cutoff parameter: value at which the loss function turns from a
      least-squares function into an absolute-value function */
  double cutoff;

  /** number of observations */
  int nobservations;

  /** number of predictors */
  int npredictors;

  /** reads problem data from a text file.
   *
   * @param filename name of input file. First entry of file is the
   * integer "nobservations" representing number of observations.
   * Second entry "npredictors" represents number of predictr
   * variables. Subsequent entries represent successive rows of the
   * predictor matrix X, with each row followed by the value of the
   * corresponding target variable Y.  */

  static HuberData * textInput(char filename[], double cutoff, int &iErr);

  /** performs operation y <- beta*y + alpha*Xt*x */
  virtual void XtMult( double beta, SimpleVector& y,
		      double alpha, SimpleVector& x );

/** performs operation y <- beta*y + alpha*Xt^T*x; that is, y <-
 *  beta*y + alpha*X*x */
  virtual void XtTransMult( double beta, SimpleVector& y,
			   double alpha, SimpleVector& x );

  virtual double datanorm();
  virtual void datarandom();
  virtual void print() {};
};


enum { huberinputok = 0,
       huberfileopenerror = -1, // couldn't find the designated input file 
       huberfileinputerror = -2 // not enough input, or error on input
};

#endif

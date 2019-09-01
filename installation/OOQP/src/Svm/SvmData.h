/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SVMDATA_H
#define SVMDATA_H

#include "Data.h"
#include "DenseGenMatrixHandle.h"
#include "DenseSymMatrixHandle.h"
#include "SimpleVectorHandle.h"

/**
 * @ingroup Svm
 *
 * Data class for Svm.
 *
 */

class SvmData : public Data
{
 public:

  /** Constructor that makes object with specified dimensions
   *
   * @param hyperplanedim_in dimenion of Euclidean space in which each
   * obseration resides
   *
   * @param nobservations_in number of observations
   *
   * @param penalty penalty parameter for violation term in objective
   * */
  SvmData(int hyperplanedim_in, int nobservations_in,
	  double penalty  );

  /** Constructor that makes object with specified dimensions, using
   * storage already allocated
   *
   * @param hyperplanedim_in dimension of Euclidean space in which
   * each observation resides
   *
   * @param nobservations_in number of observations
   *
   * @param X array of doubles containing observation points stored
   * one after another
   *
   * @param d array of doubles containing list of labels for the points
   *
   * @param penalty penalty parameter for violation term in objective
   * */
  SvmData(int hyperplanedim_in, int nobservations_in, double * X, double * d,
	  double penalty );

  /** Destructor for data class */
  ~SvmData();

  /** Reads Svm data  from given input file 
   *
   * @param filename name of input file. First entry of file is an
   * integer "nobservations" representing number of
   * observations. Second entry "hyperplanedim" is an integer
   * representing dimension of the Euclidean space in which each point
   * resides. Subsequent entries consist of sequences of
   * (hyperplanedim) doubles representing one of the observation
   * points, following by a single double representing teh label for
   * that point. The labels must take on two distinct values.
   *
   * @param penalty  penalty parameter for violation term in objective
   *
   * @param iErr returns 0 if OK, otherwise indicates error if file
   * not found or input faulty */
  static SvmData * textInput(char filename[], double penalty, int &iErr);

  /** penalty parameter for violation term in objective */
  double mPenalty;

  /** dimension of Euclidean space in which each observation resides */
  int hyperplanedim;    

  /** number of observations */
  int nobservations;  

  /** Y is an N x (t+1) matrix in which each row consists of t entries
   * representing a point, followed by a "1". Each row corresponding
   * to one of the two distinct labels is then multiplied by -1. Yt
   * stores the the transpose of Y.  */
  DenseGenMatrixHandle Yt; 

  /** vector of dimension nobservations containing the labels of the
      points */
  SimpleVectorHandle  categories; 

  /** Perform a saxpy operation with matrix Y
   *
   * y <- beta y + alpha Yx
   */
  virtual void YMult( double beta, SimpleVector& y,
		      double alpha, SimpleVector& x );
  
  /** Perform a saxpy operation with matrix Yt
   *
   * y <- beta y + alpha Y^T x
   */
  virtual void YTransMult( double beta, SimpleVector& y,
			   double alpha, SimpleVector& x );

  /** form inner product of v with the categories vector */
  virtual double dotCategories( SimpleVector & v );

  virtual double datanorm();
  virtual void datarandom();
  virtual void datainput() { };
  virtual void print() {};
  double penalty() { return mPenalty; }
};

enum { svminputok = 0,
       svmfileopenerror = -1,  // couldn't find the designated input file
       svmfileinputerror = -2, // not enough input, or error on input
       svmlabelerror = -3    // there were not exactly two distinct labels
}; 

#endif



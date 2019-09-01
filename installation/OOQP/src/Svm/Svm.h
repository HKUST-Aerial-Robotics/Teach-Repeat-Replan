/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SVM1FACTORY
#define SVM1FACTORY

#include "ProblemFormulation.h"

/** 
 * @defgroup Svm
 *
 * Solve the QP arising in support vector machine with a linear
 * kernel. 
 *
 * Input is a list of points in N-space, each point labelled with one
 * of two possible labels. Output is a hyperplane that separates the
 * two labelled sets of points, if possible, or "nearly separates"
 * them according to minimizing an objective function in which
 * violations (distance of points on the wrong side from the plane)
 * are weighed against a norm of the hyperplane gradient. 
 *
 * For further information on this formulation, see Chapter 5 of
 * V. Vapnik: "The Nature of Statistical Learning Theory", 2nd
 * edition, Springer, 1999.
 *
 */

class Svm : public ProblemFormulation {

public:

  /** constructor for Svm factory class */
  Svm() {};

  /** make data object with specified dimensions and given penalty
      parameter */
  virtual Data          * makeData(int hyperplanedim, int nobservations,
				   double penalty);

  /** make data object using given structures already allocated
   *
   * @param X array of doubles containing observation points stored
   * one after another
   *
   * @param d array of doubles containing list of labels for the points 
   *
   * @param penalty penalty parameter for violation term in objective */
  virtual Data          * makeData(int hyperplanedim, int nobservations,
				   double * X, double * d,
				   double penalty );

  /** make a data object with specified dimensions and fill it with
      random data */
  virtual Data          * makeRandomData(int hyperplanedim, int nobservations,
					 double penalty );

  /** make data object and read input from a text file in specified
      format */
  virtual Data          * makeDataFromText(char  filename[], double penalty,
					   int &iErr); 

  /** make Svm residuals object */
  virtual Residuals     * makeResiduals( Data * prob_in );

  /** make Svm linear system object */
  virtual LinearSystem  * makeLinsys( Data * prob_in );

  /** make Svm variables object */
  virtual Variables     * makeVariables( Data * prob_in );

  /** make Svm variables object, building it up from the input arrays
      of doubles which are already allocated */
  virtual Variables     * makeVariables( Data * prob_in, 
					 double w[], double v[], 
					 double z[], double u[],
					 double s[] );

  /** destructore for Svm factory class */
  virtual ~Svm();
};

#endif


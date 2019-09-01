/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef HUBERFACTORY
#define HUBERFACTORY

#include "ProblemFormulation.h"

/** 
 * @defgroup Huber
 *
 * Solve the linear Huber  regression problem.
 *
 * Input consists of dimensions "nobservations" (number of
 * observations) and "npredictors" (number of predictor variables); a
 * matrix of dimensions nobservations x npredictors containing values
 * of the predictors, a right-hand side vector of dimension
 * "nobservations"; and a cutoff value. The cutoff value represents
 * the point at which the loss function turns from being a
 * least-squares function to an absolute-value function.
 *
 */

class Huber : public ProblemFormulation {
public:

  /** constructor for the Huber factory class */
  Huber() {};

  /** make data object using data structures already allocated */
  virtual Data          * makeData( int nobservations, int npredictors,
				    double cutoff,
				    double * dXt = 0, double * dY = 0 );

  /** make data object and fill with random data */
  virtual Data          * makeRandomData( int nobservations, int npredictors,
					  double cutoff );

  /** make data object and read data from a text file in specified
      formay */
  virtual Data          * makeDataFromText(char  filename[],
					   double cutoff, int &iErr);

  /** make Huber residuals object */
  virtual Residuals     * makeResiduals( Data * prob_in );

  /** make Huber linear system object */
  virtual LinearSystem  * makeLinsys( Data * prob_in );

  /** make Huber variables obejct */
  virtual Variables     * makeVariables( Data * prob_in );

  /** make Huber variables object, building it up from input arrays
      for which storage has been allocated already */
  virtual Variables     * makeVariables( Data * prob_in,
					 double * beta, double * t,
					 double * lambda1, double * lambda2,
					 double * gamma1,  double * gamma2 );

  /** destructor for Huber factory class */
  virtual ~Huber();
};

#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef HuberVARS_H
#define HuberVARS_H

#include "SimpleVectorHandle.h"
#include "Variables.h"

/**
 * @ingroup Huber
 *
 * Variables class for Huber
 *
 */

class HuberVars : public Variables
{
public:

  /** number of observations */
  int nobservations;

  /** number of predictor variables */
  int npredictors;

  /** vectors that make up the Huber variables; see the paper by
      E. M. Gertz and S. J. Wright for details of the formulation */
  SimpleVectorHandle beta, t, gamma1, gamma2, lambda1, lambda2;

  /** constructor to make a Huber variables object of specified
      dimensions */
  HuberVars(int nobservations_in, int npredictors_in);

/** constructor to make a Huber variables object of specified
 *  dimensions for which storage has already been allocated for the
 *  five individual vector components */
  HuberVars(int nobservations_in, int npredictors_in,
	    double * beta, double * t,
	    double * lambda1, double * lambda2,
	    double * gamma1,  double * gamma2 );
    
  /** destructor for Huber variables object */
  virtual ~HuberVars();

  virtual void saxpy( Variables *b, double alpha );
  virtual void negate();

  virtual double mu();
  virtual double mustep(Variables *step, double alpha);

  virtual double stepbound( Variables *b );
  virtual double findBlocking( Variables * step, 
			       double & primalValue,
			       double & primalStep,
			       double & dualValue,
			       double & dualStep,
			       int& firstOrSecond );

  virtual void interiorPoint( double alpha, double beta );

  /** print all variables in the Svm variables structure */
  virtual void print();

  /** print only the interesting variables; namely, the coefficients
   *  of the predictor variables */
  virtual void printBeta();

  virtual void copy(Variables *b);

  virtual void shiftBoundVariables( double alpha, double beta );
  virtual double violation();
  virtual double onenorm();
  virtual double infnorm();
};

#endif


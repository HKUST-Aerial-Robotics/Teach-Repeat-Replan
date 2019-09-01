/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SVMRESIDUALS_H
#define SVMRESIDUALS_H

#include "Residuals.h"
#include "SimpleVectorHandle.h"
#include <iostream>
#include <fstream>
using namespace std;

class SvmData;
class SvmVars;

/**
 * @ingroup Svm
 *
 * Residuals class for Svm.
 *
 */

class SvmResiduals : public Residuals
{
 public:

  /** storage for the various residual components */
  SimpleVectorHandle wRes, sRes, zRes, rSV, rUZ;
  double betaRes;

  /** dimension of Euclidean space in which each observation resides */
  int hyperplanedim;

  /** number of observations */
  int nobservations;

  /** construct Svm object with specified dimensions */
  SvmResiduals(int hyperplanedim_in, int nobservations_in);
  virtual ~SvmResiduals();

  virtual void calcresids( Data *prob, Variables *vars );
  virtual void add_r3_xz_alpha(Variables *vars, double alpha);
  virtual void set_r3_xz_alpha(Variables *vars, double alpha);
  virtual void clear_r1r2();
  virtual void clear_r3();
  virtual void project_r3(double rmin, double rmax);

  /** print residuals to a specified output stream */
  virtual void asMfile( ostream& os );
};

#endif










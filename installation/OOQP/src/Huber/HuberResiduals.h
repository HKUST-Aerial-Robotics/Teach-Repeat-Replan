/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef HuberRESIDUALS_H
#define HuberRESIDUALS_H

#include "Residuals.h"
#include "SimpleVectorHandle.h"

class HuberData;
class HuberVars;

/**
 * @ingroup Huber
 *
 * Residuals class for Huber
 *
 */

class HuberResiduals : public Residuals
{
 public:

  /** storage for various residual components */
  SimpleVectorHandle Xtimest, Yresid, gam1, gam2, lamgam1, lamgam2;

  /** number of observations */
  int nobservations;

  /** number of predictors */
  int npredictors;

  /** construct residuals object with specified dimensions */
  HuberResiduals(int nobservations_in, int npredictors_in);
  virtual ~HuberResiduals();

  virtual void calcresids( Data *prob, Variables *vars );

  virtual void add_r3_xz_alpha(Variables *vars, double alpha);
  virtual void set_r3_xz_alpha(Variables *vars, double alpha);
  virtual void clear_r1r2();
  virtual void clear_r3();
  virtual void project_r3(double rmin, double rmax);
};

#endif



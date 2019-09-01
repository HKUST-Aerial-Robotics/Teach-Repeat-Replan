/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPBOUNDRESIDUALS_H
#define QPBOUNDRESIDUALS_H

#include "Residuals.h"

class QpBoundData;
class Variables;
class QpBound;
#include "OoqpVectorHandle.h"

/** 
 * Residuals for the bound-constrained QP formulation
 *
 * @ingroup QpBound */

class QpBoundResiduals : public Residuals
{
protected:
  QpBound * factory;
public:
  int nx;

  OoqpVectorHandle rc;
  OoqpVectorHandle rl;
  OoqpVectorHandle ru;
  OoqpVectorHandle rttau;
  OoqpVectorHandle rvnu;
  OoqpVectorHandle index_lower;
  OoqpVectorHandle index_upper;

  QpBoundResiduals( QpBound * f, QpBoundData * prob, int nx_);
  QpBoundResiduals( QpBound * f, OoqpVector * rc_,
		    QpBoundData * prob, int nx_);
  virtual ~QpBoundResiduals();

  QpBoundResiduals *clone();

  virtual void calcresids( Data *problem, Variables *vars );
  virtual void add_r3_xz_alpha(Variables *vars, double alpha);
  virtual void set_r3_xz_alpha(Variables *vars, double alpha);

  virtual void clear_r1r2();
  virtual void clear_r3();
  virtual void project_r3(double rmin, double rmax);
};

#endif

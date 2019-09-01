/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPEXAMPLERESIDS_H
#define QPEXAMPLERESIDS_H

#include "Residuals.h"

class Data;
class Variables;
class LinearAlgebraPackage;
class OoqpVector;

/** 
 * Residuals for the example QP formulation 
 *
 * @ingroup QpExample
 */
class QpExampleResids : public Residuals
{
protected:
  /** number of components in x */
  int nx;

  /** number of equality constraints */
  int my;

  /** number of inequality constraints */
  int mz;
public:

  /** residual for KKT condition involving Q, A, C, c */
  OoqpVector * rQ;

  /** residual for KKT condition involving A, b */
  OoqpVector * rA;

  /** residual for KKT condition involving C, d */
  OoqpVector * rC;

  /** residual for complementarity condition */
  OoqpVector * r3;

  QpExampleResids(LinearAlgebraPackage * la,
		  int nx_in, int my_in, int mz_in );
  virtual ~QpExampleResids();

  virtual void calcresids( Data *problem, Variables *vars);
  virtual void add_r3_xz_alpha(Variables *vars, double alpha);
  virtual void set_r3_xz_alpha(Variables *vars, double alpha);

  virtual void clear_r1r2();
  virtual void clear_r3();
  virtual void project_r3(double rmin, double rmax);
};

#endif

/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef RESIDUALS_H
#define RESIDUALS_H

/**
 * @file Residuals.h
 * @ingroup AbstractProblemFormulation.
 */

class Data;
class Variables;

/**
 * Represents the residuals of a QP when solved by an interior point
 * QP solver. In terms of our abstract QP formulation, these residuals
 * are rQ, rA, rC and r3.
 * @ingroup AbstractProblemFormulation
 */
class Residuals
{
protected:
  double mResidualNorm;
  double mDualityGap;
public:
  int m, n;

  /** The norm of the residuals, ommiting the complementariy conditions */
  double residualNorm() { return mResidualNorm; }

  /** A quantity that measures progress toward feasibility. IN terms
   *  of the abstract problem formulation, this quantity is defined as
   *  @code 
   * x' * Q * x - b' * y + c' * x - d' * z
   *  @endcode 
   */
  double dualityGap() { return mDualityGap; };

  /** calculate residuals, their norms, and duality/complementarity
   * gap, given a problem and variable set.  */
  virtual void calcresids(Data *problem, Variables *vars) = 0;

  /** Modify the "complementarity" component of the residuals, by
   * adding the pairwise products of the complementary variables plus
   * a constant alpha to this term.  
   */
  virtual void add_r3_xz_alpha(Variables *vars, double alpha) = 0;

  /** Set the "complementarity" component of the residuals to the
   * pairwise products of the complementary variables plus a constant
   * alpha 
   */
  virtual void set_r3_xz_alpha(Variables *vars, double alpha) = 0;

  /** set the complementarity component of the residuals to 0. */
  virtual void clear_r3() = 0;

  /** set the noncomplementarity components of the residual (the terms
   *  arising from the linear equalities in the KKT conditions) to 0.  */
  virtual void clear_r1r2(){}

  /** perform the projection operation required by Gondzio algorithm:
   * replace each component r3_i of the complementarity component of
   * the residuals by r3p_i - r3_i, where r3p_i is the projection of
   * r3_i onto the box [rmin, rmax]. Then if the resulting value is
   * less than -rmax, replace it by -rmax.
   *
   * @see SimpleVector::gondzioProjection */
  virtual void project_r3(double rmin, double rmax) = 0;

  virtual ~Residuals() {};
};

#endif




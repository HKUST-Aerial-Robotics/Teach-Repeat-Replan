/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef LINEARSYSTEM_H
#define LINEARSYSTEM_H

class Data;
class Variables;
class Residuals;

/** Implements the main solver for linear systems that arise in
 * primal-dual interior-point methods for QP
 *
 * @ingroup AbstractProblemFormulation 
 */
class LinearSystem
{
public:
  /** factorizes the matrix, stores data related to the factorization
   * to prepare for later calls to "solve"
   */
  virtual void factor(Data *prob, Variables *vars) = 0;
  
  /** assuming the "factor" call was successful, supplies the
   * right-hand side and solves the system.
   */
  virtual void solve(Data *prob, Variables *vars, Residuals *resids,
		     Variables *step) = 0;
  
  virtual ~LinearSystem() {};
};
  

#endif



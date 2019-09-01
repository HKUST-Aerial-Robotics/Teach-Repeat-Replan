/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef OPTIMIZATIONFACTORY
#define OPTIMIZATIONFACTORY

/**
 *  @defgroup AbstractProblemFormulation
 *
 *  Abstract base classes for defining a problem formulation.
 *
 * A quadratic program QP takes the form
 * @code
 * minimize    c'* x + (1/2) * x' * Q * x
 * subject to  A x  = b
 *             C x >= d
 * @endcode
 *
 * However, for many (possibly most) QP's, the matrices in the
 * formulation have structure that may be exploited to solve the
 * problem more efficiently. The AbstractProblemFormulation module
 * contains abstract base classes upon which these specialized
 * formulations are based. The optimality conditions of the simple QP
 * defined above as are follows:
 *
 * @code
 * rQ  = c + Q * x - A' * y - C' * z = 0
 * rA  = A * x - b                   = 0
 * rC  = C * x - s - d               = 0
 * r3  = S * z                       = 0
 * s, z >= 0
 * @endcode
 *
 * Where rQ, rA, rC and r3 newly defined quantities known as residual
 * vectors and x, y, z and s are variables of used in the solution of
 * the QPs.  
 * @{
 */
class Data;
class Residuals;
class LinearSystem;
class Variables;

/**
 * Creates a compatible set of components representing a problem formulation
 * specialized by structure.
 */
class ProblemFormulation {
public:

  /** create the Residuals class for the relevant formulation */
  virtual Residuals     * makeResiduals( Data * prob_in ) = 0;

  /** creates the LinearSystem class for the relevant formulation */
  virtual LinearSystem  * makeLinsys( Data * prob_in ) = 0;

  /** creates the Variables class for the relevant formulation */
  virtual Variables     * makeVariables( Data * prob_in ) = 0;
  virtual ~ProblemFormulation() {};
};

//@}
#endif



/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef STATUS_H
#define STATUS_H

class Solver;
class Data;
class Variables;
class Residuals;

enum TerminationCode 
{
  SUCCESSFUL_TERMINATION = 0,
  NOT_FINISHED,
  MAX_ITS_EXCEEDED,
  INFEASIBLE,
  UNKNOWN
};

extern const char * TerminationStrings[]; 

/**
 * Class for representing termination conditions of a QP solver.
 *
 * @ingroup QpSolvers
 */
class Status
{
public:
  virtual int doIt(  Solver * solver, Data * data, Variables * vars, 
		     Residuals * resids,
		     int i, double mu,
		     int level ) = 0;
  virtual ~Status();
};

/** structure to store data needed by termination condition checks
 * 
 * @ingroup QpSolvers */
struct StatusData {
  void * solver;
  void * data;
  void * vars;
  void * resids;
  int i;
  double mu;
  double dnorm;
  int level;
  void * ctx;
};

extern "C" {
  typedef int (*StatusCFunc)( void * data );
}


/**
 * Class that uses a C function to check for termination of a QP solver.
 *
 * @ingroup QpSolvers
 */
class CStatus : public Status {
protected:
  StatusCFunc doItC;
  void * ctx;
public:
  CStatus( StatusCFunc doItC_, void * ctx );
  virtual int doIt(  Solver * solver, Data * data, Variables * vars, 
		     Residuals * resids,
		     int i, double mu, 
		     int level );
};

#endif











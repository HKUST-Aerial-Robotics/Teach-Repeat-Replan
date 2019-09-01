/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DATA_H
#define DATA_H

class Variables;

/**
 * Stores the data of the the QP in a format appropriate for the problem 
 * structure.
 *
 * @ingroup AbstractProblemFormulation
 */
class Data
{
public:
  
  Data();    
  virtual ~Data();
  
  /** compute the norm of the problem data */
  virtual double datanorm() = 0;

  /** print the problem data */
  virtual void print() = 0;
};

#endif

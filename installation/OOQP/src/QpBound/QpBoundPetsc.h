/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPBOUNDPETSC
#define QPBOUNDPETSC

#include "QpBound.h"
#include "petscmat.h"
#include "petscksp.h"

class QpBoundData;

class QpBoundPetsc : public QpBound {
protected:
  int m;
  KSP mKsp;
public:
  virtual QpBoundData     *makeData( double alpha, Vec g, Mat Q,
				     Vec l, Vec il, Vec u, Vec iu );
  virtual QpBoundData     *makeData();

  virtual Variables       *makeVariables(Data * prob_in);
  virtual Variables       *makeVariables( Data * problem, 
					  Vec x, 
					  Vec t,   Vec v, 
					  Vec tau, Vec nu );
  virtual Residuals       *makeResiduals(Data * prob_in);
  virtual Residuals       *makeResiduals( Data * problem, 
					  Vec rg );
  QpBoundPetsc( int nx );
  QpBoundPetsc( int m, int n );
  QpBoundPetsc( int m, int n, KSP ksp );

  virtual LinearSystem   * makeLinsys( Data *prob );
  virtual OoqpVector * newPrimalVector();

};

#endif

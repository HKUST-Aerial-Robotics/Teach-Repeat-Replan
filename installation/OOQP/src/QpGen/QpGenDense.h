/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef DEQPGENFACTORY
#define DEQPGENFACTORY

#include "QpGen.h"
#include "OoqpVectorHandle.h"
class QpGenData;
class QpGenVars;

class QpGenDense : public QpGen {

public:
  QpGenDense( int nx_, int my_, int mz_ );
  // Include a constructor that takes nnzQ,A,C as input, although
  // these values are ignored. Allows us to substitute a QpGenDense
  // for a QpGenSparseXXX without changing the code.
  QpGenDense( int nx_, int my_, int mz_,
	      int nnzQ, int nnzA, int nnzC);
  virtual LinearSystem  * makeLinsys( Data * prob_in );
  virtual QpGenData     * makeData( double    c[],  double   Q[],
				    double xlow[],  char ixlow[], 
				    double xupp[],  char ixupp[],
				    double    A[],  double  bA[],
				    double    C[],  
				    double clow[],  char iclow[],
				    double cupp[],  char icupp[] );
  virtual QpGenData     * makeData();
  void makeRandomData(QpGenData *& data, QpGenVars *& soln);

  virtual void joinRHS( OoqpVector& rhs_in,  OoqpVector& rhs1_in,
			OoqpVector& rhs2_in, OoqpVector& rhs3_in );

  virtual void separateVars( OoqpVector& x_in, OoqpVector& y_in,
			     OoqpVector& z_in, OoqpVector& vars_in );
};

#endif

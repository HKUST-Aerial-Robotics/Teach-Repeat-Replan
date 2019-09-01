/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef SPSEQQPGENFACTORY
#define SPSEQQPGENFACTORY

#include "QpGen.h"
class QpGenData;
class QpGenVars;

class QpGenSparseSeq : public QpGen {
protected:
  int nnzQ;
  int nnzA;
  int nnzC;
public:
  QpGenSparseSeq( int nx_, int my_, int mz_,
		  int nnzQ_, int nnzA_, int nnzC_ )
    : QpGen( nx_, my_, mz_ ),
      nnzQ(nnzQ_), nnzA(nnzA_), nnzC(nnzC_)
  {}

  Data  * makeData();
  Data  * makeData( double    c[],
			 int    krowQ[],  int  jcolQ[],  double dQ[],
			 double  xlow[],  char ixlow[],
			 double  xupp[],  char ixupp[],
			 int    krowA[],  int  jcolA[],  double dA[],
			 double     b[],
			 int    krowC[],  int  jcolC[],  double dC[],
			 double  clow[],  char iclow[],
			 double  cupp[],  char icupp[] );

  void makeRandomData( QpGenData *& prob, QpGenVars *& soln );

  Data   *
  copyDataFromSparseTriple( double c[],
			    int irowQ[], int nnzQ,  int jcolQ[],  double dQ[],
			    double xlow[],  char ixlow[],
			    double xupp[],  char ixupp[],
			    int irowA[], int nnzA,  int jcolA[],  double dA[],
			    double   bA[],
			    int irowC[],  int nnzC,  int jcolC[], double dC[],
			    double clow[], char iclow[],
			    double cupp[], char icupp[] );

  virtual void joinRHS( OoqpVector& rhs_in,  OoqpVector& rhs1_in,
			OoqpVector& rhs2_in, OoqpVector& rhs3_in );

  virtual void separateVars( OoqpVector& x_in, OoqpVector& y_in,
			     OoqpVector& z_in, OoqpVector& vars_in );
};

#endif

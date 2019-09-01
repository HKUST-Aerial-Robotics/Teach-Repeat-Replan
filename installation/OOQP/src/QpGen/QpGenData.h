/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef QPGENDATA
#define QPGENDATA

#include "Data.h"
#include "OoqpVectorHandle.h"
#include "OoqpVector.h"
#include "DoubleMatrixHandle.h"

class MpsReader;
class LinearAlgebraPackage;
class QpGenVars;

#ifdef TESTING
class QpGenDataTester;
#endif

/**
 * Data for the general QP formulation.
 * 
 * @ingroup QpGen
 */

class QpGenData : public Data {
#ifdef TESTING
  friend QpGenDataTester;
#endif
private:

  /** as part of setting up a random test problem, generate a random
   *  set of upper, lower, and two-sided bounds */
  void randomlyChooseBoundedVariables( OoqpVector& x, OoqpVector& dualx,
				       OoqpVector& blx, OoqpVector& ixlow,
				       OoqpVector& bux, OoqpVector& ixupp,
				       double * ix,
				       double percentLowerOnly,
				       double percentUpperOnly,
				       double percentBound );
protected:
  LinearAlgebraPackage * la;

public:
  SymMatrixHandle Q;
  GenMatrixHandle A;
  GenMatrixHandle C;
  OoqpVectorHandle    g;
  OoqpVectorHandle    bA;
  OoqpVectorHandle    bux;
  OoqpVectorHandle    ixupp;
  OoqpVectorHandle    blx;
  OoqpVectorHandle    ixlow;
  OoqpVectorHandle    bu;
  OoqpVectorHandle    icupp;
  OoqpVectorHandle    bl;
  OoqpVectorHandle    iclow;
  OoqpVectorHandle    sc;

  int nx, my, mz;

  /** constructor that makes data objects of the specified dimensions */
  QpGenData(LinearAlgebraPackage * la,
	    int nx_, int my_, int mz_,
	    int nnzQ, int nnzA, int nnzC);

  /** constructor that sets up pointers to the data objects that are
      passed as arguments */
  QpGenData( LinearAlgebraPackage * la,
	     OoqpVector * c, SymMatrix * Q,
	     OoqpVector * xlow, OoqpVector * ixlow,
	     OoqpVector * xupp, OoqpVector * ixupp,
	     GenMatrix * A, OoqpVector * bA,
	     GenMatrix * C,
	     OoqpVector * clow, OoqpVector * iclow,
	     OoqpVector * cupp, OoqpVector * ciupp );

  /** insert the Hessian Q into the matrix M for the fundamental
      linear system, where M is stored as a GenMatrix */
  virtual void putQIntoAt( GenMatrix& M, int row, int col );

  /** insert the constraint matrix A into the matrix M for the
      fundamental linear system, where M is stored as a GenMatrix */
  virtual void putAIntoAt( GenMatrix& M, int row, int col );

  /** insert the constraint matrix C into the matrix M for the
      fundamental linear system, where M is stored as a GenMatrix */
  virtual void putCIntoAt( GenMatrix& M, int row, int col );

  /** insert the Hessian Q into the matrix M for the fundamental
      linear system, where M is stored as a SymMatrix */
  virtual void putQIntoAt( SymMatrix& M, int row, int col );

  /** insert the constraint matrix A into the matrix M for the
      fundamental linear system, where M is stored as a SymMatrix */
  virtual void putAIntoAt( SymMatrix& M, int row, int col );

  /** insert the constraint matrix C into the matrix M for the
      fundamental linear system, where M is stored as a SymMatrix */
  virtual void putCIntoAt( SymMatrix& M, int row, int col );

  /** y = beta * y + alpha * Q * x */
  virtual void Qmult( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x );

  /** y = beta * y + alpha * A * x */
  virtual void Amult( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x);

  /** y = beta * y + alpha * C * x   */
  virtual void Cmult( double beta,  OoqpVector& y,
		      double alpha, OoqpVector& x );

  /** y = beta * y + alpha * A\T * x */
  virtual void ATransmult( double beta,  OoqpVector& y,
			   double alpha, OoqpVector& x );

  /** y = beta * y + alpha * C\T * x */
  virtual void CTransmult( double beta,  OoqpVector& y,
			   double alpha, OoqpVector& x );

  //  virtual void addSymProdCRowToAt(double alpha, int i, 
  //				  SymMatrix& M, int rowcol );

  virtual void getg(  OoqpVector& cout );
  virtual void getbA( OoqpVector& bout );

  /** extract the diagonal of Q and put it in the OoqpVector dQ */
  virtual void getDiagonalOfQ( OoqpVector& dQ );

  virtual OoqpVector&  xupperBound() { return *bux; };
  virtual OoqpVector& ixupperBound() { return *ixupp; };
  virtual OoqpVector&  xlowerBound() { return *blx; };
  virtual OoqpVector& ixlowerBound() { return *ixlow; };
  virtual OoqpVector&  supperBound() { return *bu; };
  virtual OoqpVector& isupperBound() { return *icupp; };
  virtual OoqpVector&  slowerBound() { return *bl; };
  virtual OoqpVector& islowerBound() { return *iclow; };
  virtual OoqpVector& scale(){ return *sc; };

  virtual void createScaleFromQ();
  virtual void scaleQ();
  virtual void scaleA();
  virtual void scaleC();
  virtual void scaleg();
  virtual void scalexupp();
  virtual void scalexlow();

  virtual void flipg();
  virtual void flipQ();

  virtual double datanorm();
  virtual void datainput() {};
  virtual void datainput( MpsReader * reader, int scale, int& iErr );
  /** Create a random problem 
   *  @param (x,y,z,s) the solution to the random problem
   */
  virtual void datarandom( OoqpVector  & x, OoqpVector  & y,
			    OoqpVector & z, OoqpVector & s );
  virtual void print();

  virtual double objectiveValue( QpGenVars * vars );

  virtual ~QpGenData();
};

#endif

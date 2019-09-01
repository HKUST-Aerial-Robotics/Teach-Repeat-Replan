/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef PETSCVECTOR_H
#define PETSCVECTOR_H

#include <cstring>
#include <iostream>
#include "PetscVectorHandle.h"
#include "assert.h"
#include "OoqpVector.h"
#include "petscvec.h"

class PetscVector : public OoqpVector {
protected:
  int m;
  int preserveVec;
public:
  static int instances;
  Vec pv;

  PetscVector( int n_ );
  PetscVector( Vec v );
  PetscVector( int m_, int n_ );

  virtual ~PetscVector();

  virtual int isKindOf( int kind );
  virtual void setToZero();
  virtual void setToConstant( double c );
  virtual void randomize( double alpha, double beta, double *ix );
  virtual void copyFrom( OoqpVector& v );
  virtual double infnorm();
  virtual double onenorm();
  virtual void min( double& m, int& index );
  virtual double stepbound(OoqpVector & v, double maxStep );
  virtual double findBlocking(OoqpVector & wstep_vec, 
			      OoqpVector & u_vec, 
			      OoqpVector & ustep_vec, 
			      double maxStep,
			      double *w_elt, 
			      double *wstep_elt,
			      double *u_elt, 
			      double *ustep_elt,
			      int& first_or_second);

  virtual void componentMult( OoqpVector& v );
  virtual void componentDiv ( OoqpVector& v );
  virtual void scalarMult( double num);
  virtual void writeToStream(ostream& out) const;
  virtual void writefToStream( ostream& out,
			       const char format[] ) const;

  virtual void scale( double alpha );

  virtual void axpy  ( double alpha, OoqpVector& x );
  virtual void axzpy ( double alpha, OoqpVector& x, OoqpVector& z );
  virtual void axdzpy( double alpha, OoqpVector& x, OoqpVector& z );

  virtual void addConstant( double c );
  virtual void gondzioProjection( double rmin, double rmax );
  virtual double dotProductWith( OoqpVector& v );
  virtual double shiftedDotProductWith( double alpha, OoqpVector& mystep,
					OoqpVector& yvec,
					double beta,  OoqpVector& ystep );
  virtual void negate();
  virtual void invert();
  virtual int allPositive();

  virtual int matchesNonZeroPattern( OoqpVector& select );
  virtual void selectNonZeros( OoqpVector& select );
  virtual int numberOfNonzeros();
  virtual void addSomeConstants( double c, OoqpVector& select );
  virtual void writefSomeToStream( ostream& out,
				   const char format[],
				   OoqpVector& select ) const;
  virtual void axdzpy( double alpha, OoqpVector& x,
		       OoqpVector& z, OoqpVector& select );

  virtual int somePositive( OoqpVector& select );
  virtual void divideSome( OoqpVector& div, OoqpVector& select );
  virtual void copyIntoArray( double v[] ) const;
  virtual void copyFromArray( double v[] );
  virtual void copyFromArray( char v[] );

  int getSize() { return n; };
  int getLocalSize() { return m; };
};

#endif


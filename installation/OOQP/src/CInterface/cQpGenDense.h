/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef CQPGENDENSE
#define CQPGENDENSE

#include "cQpGen.h"

#ifdef __cplusplus
extern "C" {
#endif

  void newQpGenDense( double ** c,      int nx,    double ** Q,
		      double ** xlow,              char   ** ixlow,
		      double ** xupp,              char   ** ixupp,
		      double ** A,      int my,    double ** b,
		      double ** C,      int mz,
		      double ** clow,              char   ** iclow,
		      double ** cupp,              char   ** icupp,
		      int    *  ierr );

  void freeQpGenDense( double ** c,      double ** Q,
		       double ** xlow,   char   ** ixlow,
		       double ** xupp,   char   ** ixupp,
		       double ** A,      double ** b,
		       double ** C,
		       double ** clow,   char ** iclow,
		       double ** cupp,   char ** icupp );

  void qpsolvede( double    c[],  int  nx,       double Q[],
		double xlow[],  char ixlow[], 
		double xupp[],  char ixupp[],
		double    A[],  int  my,       double   bA[],
		double    C[],  int  mz,
		double clow[],  char iclow[],
		double cupp[],  char icupp[],
		double    x[],  double gamma[],     double phi[],
		double    y[],
		double    z[],  double lambda[],  double pi[],
		double   *objectiveValue,
		int print_level, int * ierr );

  void QpGenDenseGondzioSetup( double    c[],   int  nx,       double Q[],
			     double xlow[],   char ixlow[], 
			     double xupp[],   char ixupp[],
			     double    A[],   int  my,       double   bA[],
			     double    C[],   int  mz,   
			     double clow[],   char iclow[],
			     double cupp[],   char icupp[],
			     QpGenContext * ctx,
			     int * ierr );

#ifdef __cplusplus
}
#endif

#endif





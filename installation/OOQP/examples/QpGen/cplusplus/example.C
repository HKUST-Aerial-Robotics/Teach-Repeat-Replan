/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenData.h"
#include "QpGenVars.h"
#include "QpGenResiduals.h"
#include "GondzioSolver.h"
#include "QpGenSparseMa27.h"

#include <iostream>
#include <string.h>

using namespace std;

const int nx   = 2;
double    c[]  = { 1.5,  -2 };

double  xupp[] = { 20,   0 };
char   ixupp[] = {  1,   0 };

double  xlow[] = {  0,   0 };
char   ixlow[] = {  1,   1 };

const int nnzQ = 3;
int    irowQ[] = {  0,   1,   1 }; 
int    jcolQ[] = {  0,   0,   1 };
double    dQ[] = {  8,   2,  10 };


int my         = 0;
double * b     = 0;
int nnzA       = 0;
int * irowA    = 0;
int * jcolA    = 0;
double * dA    = 0;

const int mz   = 2;
double clow[]  = { 2,   0 };
char  iclow[]  = { 1,   0 };

double cupp[]  = { 0,   6 };
char  icupp[]  = { 0,   1 };

const int nnzC = 4;
int   irowC[]  = { 0,   0,   1,   1};
int   jcolC[]  = { 0,   1,   0,   1};
double   dC[]  = { 2,   1,  -1,   2};

int main( int argc, char * argv[] )
{
  int usage_ok = 1, quiet = 0;

  if( argc > 2 ) usage_ok = 0;
  if( argc == 2 ) {
    if( 0 == strcmp( "--quiet", argv[1] ) ) {
      quiet = 1;
    } else {
      usage_ok = 0;
    }
  } 
  if( !usage_ok ) {
    cerr << "Usage: " <<  argv[0] << " [ --quiet ]\n";
    return 1;
  }
    
  QpGenSparseMa27 * qp 
    = new QpGenSparseMa27( nx, my, mz, nnzQ, nnzA, nnzC );
  
  QpGenData      * prob = (QpGenData * ) qp->copyDataFromSparseTriple(
        c,      irowQ,  nnzQ,   jcolQ,  dQ,
        xlow,   ixlow,  xupp,   ixupp,
        irowA,  nnzA,   jcolA,  dA,     b,
        irowC,  nnzC,   jcolC,  dC,
        clow,   iclow,  cupp,   icupp );

  QpGenVars      * vars 
    = (QpGenVars *) qp->makeVariables( prob );
  QpGenResiduals * resid 
    = (QpGenResiduals *) qp->makeResiduals( prob );
  
  GondzioSolver  * s     = new GondzioSolver( qp, prob );
  
  if( !quiet ) s->monitorSelf();
  int ierr = s->solve(prob,vars, resid);
  
  if( ierr == 0 ) {
    cout.precision(4);
    cout << "Solution: \n";
    vars->x->writefToStream( cout, "x[%{index}] = %{value}" );
  } else {
    cout << "Could not solve the problem.\n";
  }
  return ierr;
}

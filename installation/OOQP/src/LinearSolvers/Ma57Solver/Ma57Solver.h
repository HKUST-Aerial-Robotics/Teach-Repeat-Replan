/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef MA57LINSYS_H
#define MA57LINSYS_H

#include "DoubleLinearSolver.h"
#include "SparseSymMatrixHandle.h"
#include "SparseStorageHandle.h"
#include "OoqpVectorHandle.h"
#include "SparseStorage.h"

extern "C" {
  void ma57id_( double cntl[],  int icntl[] );

  void ma57ad_( int * n,        int * ne,       int irn[],     
		int jcn[],      int * lkeep,    int keep[],
		int iwork[],    int icntl[],    int info[],
		double rinfo[] );

  void ma57bd_( int * n,        int * ne,       double a[],
		double fact[],  int * lfact,    int ifact[],
		int * lifact,   int * lkeep,    int keep[],
		int ppos[],     int * icntl,    double cntl[],
		int info[],     double rinfo[] );
  void ma57cd_( int * job,      int * n,        double fact[],
		int * lfact,    int ifact[],    int * lifact,
		int * nrhs,     double rhs[],   int * lrhs,
		double w[],     int * lw,       int iw1[],
		int icntl[],    int info[]);
  void ma57dd_( int * job,      int * n,        int * ne,
		double a[],     int irn[],      int jcn[],
		double fact[],  int * lfact,    int ifact[],
		int * lifact,   double rhs[],   double x[],
		double resid[], double w[],     int iw[],
		int icntl[],    double cntl[],  int info[],
		double rinfo[] );
  void ma57ed_( int * n,        int * ic,       int keep[],
		double fact[],  int * lfact,    double * newfac,
		int * lnew,     int  ifact[],   int * lifact,
		int newifc[],   int * linew,    int * info );
}

/** implements the linear solver class using the HSL MA57 solver
 *
 * @ingroup LinearSolvers 
 */
class Ma57Solver : public DoubleLinearSolver {
protected:
  int     icntl[20];
  int     info[40];
  double  cntl[5];
  double  rinfo[20];

  /** the Threshold Pivoting parameter, stored as U in the ma27dd
   *  common block. Takes values in the range [0,1]. Larger values
   *  enforce greater stability in the factorization as they insist on
   *  larger pivots. Smaller values preserve sparsity at the cost of
   *  using smaller pivots.  */
  double   kThresholdPivoting;

  /** the Threshold Pivoting parameter may need to be increased during
   * the algorithm if poor precision is obtained from the linear
   * solves.  kThresholdPivoting indicates the largest value we are
   * willing to tolerate.  */
  double   kThresholdPivotingMax;

  /** the factor in the range (1,inf) by which kThresholdPivoting is
   * increased when it is found to be inadequate.  */
  double   kThresholdPivotingFactor;

  /** the "Treat As Zero" parameter, stored as pivtol in the common
   * block ma27td. The factorization will not accept a pivot whose
   * absolute value is less than this parameter as a 1x1 pivot or as
   * the off-diagonal in a 2x2 pivot.  */
  double   kTreatAsZero;

  /** precision we demand from the linear system solver. If it isn't
   * attained on the first solve, we use iterative refinement and
   * possibly refactorization with a higher value of
   * kThresholdPivoting. */
  double  kPrecision;

  /** index array for the factorization */
  int     *irowM,    *jcolM;

  /** storage for the original matrix */
  double  *M;

  /** dimension of the whole matrix */
  int      n;

  /** number of nonzeros in the matrix */
  int      nnz;

  /** temporary storage */
  int     lkeep, *keep;

  /** temporary storage for the factorization process */
  int     lifact, *ifact, lfact;

  /* storage for the factors */
  double *fact;

  /** amounts by which to increase allocated factorization space when
   * inadequate space is detected. ipessimism is for array "iw",
   * rpessimism is for the array "fact". */
  double  ipessimism, rpessimism;

  /** used to indicate when we need a fresh factorization (when
   * iterative refinement has failed to improve the precision of the
   * computed solution satisfactorily */
  int     freshFactor;

  /** store as a sparse symmetric matrix */
  SparseStorageHandle mStorage;

  /** called the very first time a matrix is factored. Allocates space
   * for the factorization and performs ordering */
  virtual void firstCall();
public:
  /** sets mStorage to refer to the argument sgm */
  Ma57Solver( SparseSymMatrix * sgm );
  
  virtual void diagonalChanged( int idiag, int extent );
  virtual void matrixChanged();
  virtual void solve( OoqpVector& rhs );

  /** set the Treat As Zero parameter in the MA27 data structures to
   *  the current value of kTreatAsZero */
  double setTreatAsZero() { return cntl[1] = kTreatAsZero; }

  /** set the Pivoting Threshold parameter in the MA27 data structures
   *  to the current value of kThresholdPivoting */
  double setThresholdPivoting() { return cntl[0] = kThresholdPivoting; }  

  /** destructor */
  virtual ~Ma57Solver();
};

#endif

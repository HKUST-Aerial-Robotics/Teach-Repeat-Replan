/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#ifndef MA27LINSYS_H
#define MA27LINSYS_H

#include "DoubleLinearSolver.h"
#include "SparseSymMatrixHandle.h"
#include "SparseStorageHandle.h"
#include "OoqpVectorHandle.h"



extern "C" {
  void ma27id_( int icntl[],    double cntl[] );

  void ma27ad_( int * n,        int * nz,       
		int irn[],      int icn[],      
		int iw[],       int * liw,
		int ikeep[],    int iw1[],      
		int * nsteps,   int * iflag,
		int icntl[],    double cntl[],
		int info[],     double * ops );

  void ma27bd_( int * n,        int * nz,
		int irn[],      int icn[],
		double a[],     int * la,
		int iw[],       int * liw,
		int ikeep[],    int * nsteps,
		int * maxfrt,   int iw1[],
		int icntl[],    double cntl[], 
		int  info[] );

  void ma27cd_( int * n,        
		double a[],     int * la,
		int iw[],       int * liw,
		double w[],     int * maxfrt,
		double rhs[],
		int iw2[],      int * nsteps,
		int icntl[],    int info[] );
}

/** implements the linear solver class using the HSL MA27 solver
 *
 * @ingroup LinearSolvers 
 */
class Ma27SolverBase : public DoubleLinearSolver {
protected:
  int     icntl[30];
  int     info[20];
  double  cntl[5];


  /** ierror, used by ma27 to send context-specific supplimental
   *  info to the user */
  int ierror() { return info[1]; }
  int minimumRealWorkspace() { return info[4]; }
  int minimumIntWorkspace() { return info[5]; }
  int ma27ErrFlg() { return info[0]; }

  /** precision we demand from the linear system solver. If it isn't
   * attained on the first solve, we use iterative refinement and
   * possibly refactorization with a higher value of
   * kThresholdPivoting. */
  double  precision;

  /** index array for the factorization */
  int     *irowM,    *jcolM;

  /** nonzero element of the factors */
  double  *fact;

  /** dimension of the whole matrix */
  int      n;

  /** number of nonzeros in the matrix */
  int      nnz;

  /** length of the array containing factors; may be increased during
   * the numerical factorization if the estimate obtained during the
   * symbolic factorization proves to be inadequate. */
  int la;

  /** pivot sequence and temporary storage information */
  int     *ikeep, *iw, liw, *iw1, *iw2, nsteps, maxfrt;

  /** temporary storage for the factorization */
  double  *w;

  /** amounts by which to increase allocated factorization space when
   * inadequate space is detected. ipessimism is for array "iw",
   * rpessimism is for the array "fact". */
  double  ipessimism, rpessimism;

  /** called the very first time a matrix is factored. Allocates space
   * for the factorization and performs ordering */
  virtual void firstCall();
public:

  /** base class constructor. Allocates values for kTreatAsZero,
   * kThresholdPivoting, kThresholdPivotingMax,
   * kThresholdPivotingFactor, kPrecision, ipessimism,rpessimism */
  Ma27SolverBase( int n, int nnz );
  
  /** the Threshold Pivoting parameter, stored as U in the ma27dd
   *  common block. Takes values in the range [0,1]. Larger values
   *  enforce greater stability in the factorization as they insist on
   *  larger pivots. Smaller values preserve sparsity at the cost of
   *  using smaller pivots.  */
  double thresholdPivoting() { return cntl[0]; }
  void   setThresholdPivoting( double piv  ) { cntl[0] = piv; }

  /** the "Treat As Zero" parameter, stored as pivtol in the common
   * block ma27td. The factorization will not accept a pivot whose
   * absolute value is less than this parameter as a 1x1 pivot or as
   * the off-diagonal in a 2x2 pivot.  */
  double   treatAsZero() { return cntl[2]; }
  void     setTreatAsZero( double tol ) { cntl[2] = tol; }

  /** copy elements from matrix into the fact data structure, in
   * preparation for factorization (or refactorization).  */
  virtual void copyMatrixElements( double fact[], int lfact ) = 0;

  /** change format for row/column index matrices, in preparation for
   * call to MA27 FORTRAN routines
   * 
   * @param irowM array of nnz elements indicating row index (in range
   * 1..n) of the corresponding matrix element
   *
   * @param jcolM array of nnz elements indicating col index (in range
   * 1..n) of the corresponding matrix element */
  virtual void getIndices( int irowM[], int jcolM[] ) = 0;

  virtual void diagonalChanged( int idiag, int extent );
  virtual void matrixChanged();

  /** version of the main solve routine that takes argument as an
   * array of doubles
   *
   * @param drhs on input contains the right-hand side; on output
   * contains the solution
   *
   * @param n dimension of the system
   */
  virtual void basicSolve( double drhs[], int n );

  /** version of the main solve routine that takes argument as an
   * OoqpVector
   *
   * @param drhs on input contains the right-hand side; on output
   * contains the solution
   *
   * @param n dimension of the system 
   */
  virtual void solve( OoqpVector& rhs ) = 0;

  /** destructor */
  virtual ~Ma27SolverBase();
};

/** implements the linear solver class using the HSL MA27 solver
 *
 * @ingroup LinearSolvers 
 */
class Ma27Solver : public Ma27SolverBase {
protected:
  /** store as a sparse symmetric matrix */
  SparseSymMatrixHandle mMat;

public:
  /** sets mMat to refer to the argument sgm */
  Ma27Solver( SparseSymMatrix * sgm );

  /** copy the contents of the matrix into fact 
   *
   * @param fact on exit, contains the nonzero elements of the
   * original matrix, stored as an array of doubles 
   */
  virtual void copyMatrixElements( double fact[], int lfact );

/** change format for row/column index matrices, in preparation for
   * call to MA27 FORTRAN routines
   * 
   * @param irowM array of nnz elements indicating row index (in range
   * 1..n) of the corresponding matrix element
   *
   * @param jcolM array of nnz elements indicating col index (in range
   * 1..n) of the corresponding matrix element */
  virtual void getIndices( int irowM[], int jcolM[] );

  /** version of the main solve routine that takes argument as an
   * OoqpVector
   *
   * @param drhs on input contains the right-hand side; on output
   * contains the solution
   *
   * @param n dimension of the system */
  virtual void solve( OoqpVector& rhs );
};

#endif




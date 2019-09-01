/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include <cmath>
#include <cstring>
#include <cassert>
#include <ostream>

#include "SparseStorage.h"
#include "OoqpVector.h"
#include "SimpleVector.h"

int SparseStorage::instances = 0;

SparseStorage::SparseStorage( int m_, int n_, int len_ )
{
  int i;
  
  neverDeleteElts = 0;
  m      = m_;
  n      = n_;
  len    = len_;
  jcolM  = new int[len];
  krowM  = new int[m+1];
  for( i = 0; i <= m; i++ ) {
    krowM[i] = 0;
  }
  M      = new double[len];

  SparseStorage::instances++;
}

SparseStorage::SparseStorage( int m_, int n_, int len_,
				      int * krowM_, int * jcolM_,
				      double * M_ )
{
  neverDeleteElts = 1;
  m               = m_;
  n               = n_;
  len             = len_;
  jcolM           = jcolM_;
  krowM           = krowM_;
  M               = M_;

  SparseStorage::instances++;
}

SparseStorage::~SparseStorage()
{
  if ( !neverDeleteElts ) {
    delete [] jcolM;
    delete [] krowM;
    delete [] M;
  }

  SparseStorage::instances--;
}

void SparseStorage::getSize( int& m_, int& n_ )
{
  m_ = m;
  n_ = n;
}


void SparseStorage::fromGetDiagonal( int idiag, OoqpVector& vec_in )
{
  SimpleVector & vec = dynamic_cast<SimpleVector &>(vec_in);
  int extent = vec.length();

  assert( idiag + extent <= m );
  assert( idiag + extent <= n );
  
  int i, j, k;

  for ( i = idiag; i < idiag + extent; i++ ) {
    // Loop over all rows in range

    // Set the diagonal element to zero in case we don't find 
    // one in the matrix
    vec[i-idiag] = 0.0;
    for( k = krowM[i]; k < krowM[i+1]; k++ ) {
      // Loop over the elements of the sparse row
      j = jcolM[k];
      if ( i == j ) {
	vec[i-idiag] = M[k];
      }
    } // End loop over the elements of the sparse row
  } // End loop over all rows in range

}

void SparseStorage::ColumnScale( OoqpVector& scale_in )
{
  SimpleVector & scale = dynamic_cast<SimpleVector &>(scale_in);
  int extent = scale.length();

  assert( extent == n );
 
  int i, j, k;

  for ( i = 0; i < m; i++ ) {
    // Loop over all rows in the sparse matrix
    for( k = krowM[i]; k < krowM[i+1]; k++ ) {
      // Loop over the elements of the sparse row
      j = jcolM[k];
      M[k] = M[k] * scale[j];
    } // End loop over the elements of the sparse row
  } // End loop over all rows in the sparse matrix

}

void SparseStorage::SymmetricScale( OoqpVector& scale_in )
{
  SimpleVector & scale = dynamic_cast<SimpleVector &>(scale_in);
  int extent = scale.length();

  assert( extent == n );
  assert( extent == m );

  int i, j, k;

  for ( i = 0; i < m; i++ ) {
    // Loop over all rows in the sparse matrix
    for( k = krowM[i]; k < krowM[i+1]; k++ ) {
      // Loop over the elements of the sparse row
      j = jcolM[k];
      M[k] = M[k] * scale[j] * scale[i];
    } // End loop over the elements of the sparse row
  } // End loop over all rows in the sparse matrix

}


void SparseStorage::scalarMult( double num )
{
  int i, k;

  for ( i = 0; i < m; i++ ) {
    // Loop over all rows in the sparse matrix
    for( k = krowM[i]; k < krowM[i+1]; k++ ) {
      // Loop over the elements of the sparse row
      M[k] = M[k] * num;
    } // End loop over the elements of the sparse row
  } // End loop over all rows in the sparse matrix

}

void SparseStorage::getDiagonal( OoqpVector& vec_in )
{
  this->fromGetDiagonal( 0, vec_in );
}

void SparseStorage::setToDiagonal( OoqpVector& vec_in )
{
  SimpleVector & vec = dynamic_cast<SimpleVector &>(vec_in);
  int diagExtent = (m <= n) ? m : n; 

  assert( diagExtent == vec.length() );

  int i;
  for( i = 0; i <= diagExtent; i++ ) {
    krowM[i] = i; // Initialize to the diagonal matrix of all zeros
  }
  for( ; i <= m; i++ ) {
    krowM[i] = diagExtent;
  }
  
  double * v = &vec[0];
  for( i = 0; i < diagExtent; i++ ) {
    jcolM[i] = i;
    M[i]     = v[i];
  }
}

void SparseStorage::fromGetDense( int row, int col, double * A, int lda,
				      int rowExtent, int colExtent )
{
  int i, j, k;
  
  assert( row >= 0 && row + rowExtent <= m );
  assert( col >= 0 && col + colExtent <= n );
  
  for ( i = row; i < row + rowExtent; i++ ) {
    // Loop over all rows in range
    int jcurrent = col - 1;
    for( k = krowM[i]; k < krowM[i+1]; k++ ) {
      // Loop over the elements of the sparse row
      j = jcolM[k];
      if ( j >= col ) {
	// j is big enough to be within range
	if ( j < col + colExtent ) {
	  // j is small enough to be within range
	  for ( jcurrent++; jcurrent < j; jcurrent++ ) {
	    A[(i - row) * lda + jcurrent - col] = 0.0;
	  }
	  jcurrent = j;
	  A[(i - row) * lda + j - col] = M[k];
	} else { // j is too big. 
	  // There will be no more interesting elements in this row
	  break;
	} // End else j is too big
      } // End if j is big enough
    } // End loop over element of the sparse row
    for( jcurrent++; jcurrent < n; jcurrent++ ) {
      A[(i - row) * lda + jcurrent - col] = 0.0;
    }
  } // End loop over all rows in range
}


void SparseStorage::atPutSpRow( int row, double A[], int lenA,
				int jcolA[], int& info )
{
  int ka    = lenA - 1;
  int km_f  = krowM[row + 1] - 1;
  int km    = km_f;
  int km_s  = krowM[row];
  int count = 0;

  assert( row >= 0 && row < m );

  while( ka >= 0 ) {
    if ( km < km_s ) {
      // There are no more elements in M. All the rest of A must be
      // inserted.
      count += ka + 1;
      break;
    } else if ( jcolM[km] == jcolA[ka] ) {
      // The element in A will replace an element in M
      km--; ka--;
    } else if ( jcolM[km] > jcolA[ka] ) {
      assert( jcolA[ka] >= 0 );
      // This element is in M but not in A
      km--;
    } else {
      // The element is in A, but not in M and so must be inserted.
      assert( jcolA[ka] < n );
      ka--;
      count++;
    }
  }

  if ( count > 0 ) {
    this->shiftRows( row + 1, count, info );
  } else {
    info = 0;
  }

  if ( 0 == info ) {
    ka    = lenA - 1;
    km    = km_f;
    int ik    = krowM[row + 1] - 1;

    while ( ka >= 0 ) {
      if ( km < km_s ) {
	// There are no more elements in M. All the rest of A must be
	// inserted.
	for( ; ka >= 0; ka--, ik-- ) {
	  jcolM[ik] = jcolA[ka];
	  assert( jcolM[ik] >= 0 && jcolM[ik] < n );
	  M[ik]     = A[ka];
	}
	break;
      } else if ( jcolM[km] == jcolA[ka] ) {
	// The element in A will replace an element in M
	jcolM[ik] = jcolM[km];
	M[ik]     = A[ka];
	km--; ka--; ik--;
      } else if ( jcolM[km] > jcolA[ka] ) {
	// This element is in M but not in A
	jcolM[ik] = jcolM[km];
	M[ik]     = M[km];
	km--; ik--;
      } else {
	// The element is in A, but not in M.
	jcolM[ik] = jcolA[ka];
	assert( jcolM[ik] >= 0 && jcolM[ik] < n );
	M[ik]     = A[ka];
		
	ka--; ik--;
      }
    }
  }
}

void SparseStorage:: atPutDense( int row, int col, double * A, int lda,
				     int rowExtent, int colExtent )
{ 
  int info;
  int i;

  assert( row >= 0 && row + rowExtent <= m );
  assert( col >= 0 && col + colExtent <= n );

  for ( i = row; i < row + rowExtent; i++) {
    // Loop over all rows in range.
    int km_f = krowM[i + 1] - 1;
    int km   = km_f;
    int ka   = colExtent - 1;
    int count = 0;
    while( km >= krowM[i] || ka >= 0 ) {
      // The current row in M and the current row in A are not
      // both empty
      if ( ka < 0 ) {
	// The current row of A is empty. Insert an element from M
	km--;
      } else if ( km < krowM[i] ) {
	// The current row of M is empty. Insert an element from A.
	if ( A[(i - row) * lda + ka] == 0 ) {
	  // The current element of A is zero. Skip it.
	  ka--;
	} else {
	  count++; ka --;
	}
      } else if ( ka + col > jcolM[km] ) {
	// The current element in A comes first.
	if ( A[(i - row) * lda + ka] == 0 ) {
	  // The current element of A is zero. Skip it.
	  ka--;
	} else {
	  count++; ka--;
	}
      } else if ( ka + col < jcolM[km] ) {
	// The current element in M comes first.
	km--;
      } else {
	// The current element in M is overwritten by the element in A.
	km--; ka--;
      }
    } // End while the current row...are not both empty
    if ( count > 0 ) {
      this->shiftRows( i + 1, count, info );
    } else {
      info = 0;
    }
    if ( info != 0 ) { 
      cout << "bing\n"; return;
    }
	
    km = km_f;
    int k  = krowM[i + 1] - 1;
    ka = colExtent - 1;

    while( km >= krowM[i] || ka >= 0 ) {
      // The current row in M and the current row in A are not
      // both empty
      if ( ka < 0 ) {
	// The current row of A is empty. Insert an elemnt from M
	M[k]     = M[km];
	jcolM[k] = jcolM[km];
	k--; km--;
      } else if ( km < krowM[i] ) {
	// The current row of M is empty. Insert an element from A.
	if ( A[ (i - row) * lda + ka] == 0 ) {
	  // The current element of A is zero. Skip it.
	  ka--;
	} else {
	  M[k]     = A[(i - row) * lda + ka];
	  jcolM[k] = ka + col;
	  k--; ka --;
	}
      } else if ( ka + col > jcolM[km] ) {
	// The current element in A comes first.
	if ( A[ (i - row) * lda + ka] == 0 ) {
	  // The current element of A is zero. Skip it.
	  ka--;
	} else {
	  M[k]     = A[(i - row) * lda + ka];
	  jcolM[k] = ka + col;
	  k--; ka--;
	}
      } else if ( ka + col < jcolM[km] ) {
	// The current element in M comes first.
	M[k]      = M[km];
	jcolM[k]  = jcolM[km];
	k--; km--;
      } else {
	// The current element in M is overwritten by the element in A.
	M[k]       = A[(i - row) * lda + ka];
	jcolM[k]   = ka + col;  
	k--; km--; ka--;
      }
    } // End while the current row...are not both empty
  } // End loop over all rows in range.
}

void SparseStorage::shiftRows( int row, int shift, int& info )
{
  if ( shift == 0 ) {
    info = 0;
  } else if ( krowM[m] + shift > len ) {
    // Insufficient space
    info = krowM[m] + shift - len;
  } else {
    // We perform the copy
    info = 0;
    int lcopy = krowM[m] - krowM[row];
    if ( lcopy > 0 ) {
      // There is anything to copy
      // As a consequence of lcopy > 0, col !== n
      memmove( &jcolM[ krowM[row] + shift ], 
	       &jcolM[ krowM[row] ], lcopy * sizeof(int) );
      memmove( &M[ krowM[row] + shift ],
	       &M[ krowM[row] ], lcopy * sizeof(double) );
      int i;
      for ( i = row; i <= m; i++ ) {
	krowM[i] += shift;
      }    
    } else {
      // Still adjust the starts of the rows
      int i;
      int rowStart = krowM[m] + shift; 
      for ( i = row; i <= m; i++ ) {
	krowM[i] = rowStart;
      }    
    }
  } // end else we perform the copy
}

void SparseStorage::putSparseTriple( int irow[], int lenA,
					 int jcol[], double A[], 
					 int& info )
{
  if( len < lenA ) {
    info = 1;
    return;
  }
  info = 0;
  int i, k;
  krowM[0] = 0;
  i = 0;
  for( k = 0; k < lenA; k++ ) {
    while( i < irow[k] ) {
      i++;
      krowM[i] = k;
    }
    // now i == irow[k] because irow is sorted
    jcolM[k] = jcol[k];
    M[k]     = A[k];
  }
  for( i++; i <= m; i++ ) {
    krowM[i] = lenA;
  }
}

void SparseStorage::fromGetSpRow( int row, int col,
				      double A[], int lenA, int jcolA[],
				      int& nnz,
				      int colExtent, int& info )
{
  assert( col >= 0 && col < n );
  assert( row >= 0 && row < m );
  assert( col + colExtent <= n );
  int km;
  int ka = 0;
  int lastCol = col + colExtent - 1;

  info = 0;
  
  for ( km = krowM[row]; km < krowM[row+1]; km++ ) {
    int colm = jcolM[km];
    if ( colm >= col ) {
      if ( colm <= lastCol ) {
	if( ka < lenA ) {
	  A[ka]     = M[km];
	  jcolA[ka] = colm;
	  ka++;
	} else {
	  // Count the number of aditional elements needed in A
	  info++;
	}
      } else {
	break;
      }
    }
  }
  nnz = ka;
}

void SparseStorage::writeToStream(std::ostream& out) const
{
  int i, k;

  for( i = 0; i < m; i++ ) {
    for ( k = krowM[i]; k < krowM[i+1]; k++ ) {
      out << i << '\t' << jcolM[k] << '\t' << M[k] << std::endl;
    }
  }
}

void indexedLexSort( int first[], int n, int swapFirst,
		     int second[], int swapSecond, int index[] )
{
  int fi, se, j, k, kinc, ktemp;
  const int nincs = 12;
  const int incs[]  = {1, 5, 19, 41, 109, 209, 505,
		       929, 2161, 3905, 8929, 16001};
  
  kinc = 0;
  for ( k = 0; k < nincs; k++ ) {
    kinc = k;
    if ( incs[kinc] > n/2 ) {
      kinc--;
      break;
    }
  }
  // incs[kinc] is the greatest value in the sequence that is also less
  // than n/2.

  //for( k = 0; k < n; k++ ) index[k] = k;

  for( ; kinc >= 0; kinc-- ) {
    // Loop over all increments
    int inc = incs[kinc];

    if ( !swapFirst && !swapSecond ) {
      for ( k = inc; k < n; k++ ) {
	// loop over all subarrays defined by the current increment
	ktemp = index[k];
	fi = first[ ktemp ];
	se = second[ ktemp ];
	// Insert element k into the sorted subarray
	for( j = k; j >= inc; j -= inc ) {
	  // Loop over the elements in the current subarray
	  if ( fi < first[ index[j - inc] ] ||
	       ( fi == first[ index[j - inc] ] &&
		 se < second[ index[j - inc] ] ) ) {
	    // Swap elements j and j - inc, implicitly use the fact
	    // that ktemp hold element j to avoid having to assign to 
	    // element j - inc
	    index[j]        = index[j - inc];
	  } else {
	    // There are no more elements in this sorted subarray which
	    // are less than element j
	    break;
	  }
	} // End loop over the elements in the current subarray
	// Move index[j] out of temporary storage
	index[j] = ktemp;
	// The element has been inserted into the subarray.
      } // End loop over all subarrays defined by the current increment
    } else if ( swapSecond && !swapFirst ) {
      for ( k = inc; k < n; k++ ) {
	ktemp = index[k];
	fi = first[ ktemp ];
	se = second[ k ];
	for( j = k; j >= inc; j -= inc ) {
	  if ( fi < first[ index[j - inc] ] ||
	       ( fi == first[ index[j - inc] ] &&
		 se < second[ j - inc ] ) ) {
	    index[j]        = index[j - inc];
	    second[j]       = second[j - inc];
	  } else {
	    break;
	  }
	} 
	index[j]   = ktemp;
	second[j]  = se;
      } 
    } else if ( swapFirst  && !swapSecond ) {
      for ( k = inc; k < n; k++ ) {
	ktemp = index[k];
	fi = first[ k ];
	se = second[ ktemp ];
	for( j = k; j >= inc; j -= inc ) {
	  if ( fi < first[j - inc] ||
	       ( fi == first[j - inc] &&
		 se < second[ index[j - inc ] ]) ) {
	    index[j]        = index[j - inc];
	    first[j]        = first[j - inc];
	  } else {
	    break;
	  }
	} 
	index[j]   = ktemp;
	first[j]   = fi;
      } 
    } else { // Swap both
      for ( k = inc; k < n; k++ ) {
	ktemp = index[k];
	fi = first[ k ];
	se = second[ k];
	for( j = k; j >= inc; j -= inc ) {
	  if ( fi < first[j - inc] ||
	       ( fi == first[j - inc] &&
		 se < second[ j - inc ]) ) {
	    index[j]        = index[j - inc];
	    first[j]        = first[j - inc];
	    second[j]       = second[j - inc];
	  } else {
	    break;
	  }
	} 
	index[j]   = ktemp;
	first[j]   = fi;
	second[j]  = se;
      } 
    }
  } // End loop over all increments
}


void SparseStorage::mult( double beta,  double y[], int incy,
			      double alpha, double x[], int incx )
{
  int i, j, k;
  double temp;
  if (beta == 0) {
    if (incx == 1) {
      for( i = 0; i < m; i++ ) {
	temp = 0;
	for( k = krowM[i]; k < krowM[i+1]; k++ ) {
	  j = jcolM[k];
	  temp += M[k] * x[j];
	}
	y[i * incy] = alpha * temp;
      }
    } else {
      for( i = 0; i < m; i++ ) {
	temp = 0;
	for( k = krowM[i]; k < krowM[i+1]; k++ ) {
	  j = jcolM[k];
	  temp += M[k] * x[j * incx];
	}
	y[i * incy] = alpha * temp;
      }
    }
  } else {
    if (incx == 1) {
      for( i = 0; i < m; i++ ) {
	temp = 0;
	for( k = krowM[i]; k < krowM[i+1]; k++ ) {
	  j = jcolM[k];
	  temp += M[k] * x[j];
	}
	y[i * incy] = beta * y[i * incy] + alpha * temp;
      }
    } else {
      for( i = 0; i < m; i++ ) {
	temp = 0;
	for( k = krowM[i]; k < krowM[i+1]; k++ ) {
	  j = jcolM[k];
	  temp += M[k] * x[j * incx];
	}
	y[i * incy] = beta * y[i * incy] + alpha * temp;
      }
    }
  }
}

void SparseStorage::transMult( double beta,  double y[], int incy,
				   double alpha, double x[], int incx )
{
  int i, j, k;
  if (beta == 0) {
    for( j = 0; j < n; j++ ) {
      y[j * incy] = 0;
    }
  } else {
    for( j = 0; j < n; j++ ) {
      y[j * incy] *= beta;
    }
  }
  if (incy == 1) {
    for( i = 0; i < m; i++ ) {
      double axi = alpha * x[i * incx];
      for( k = krowM[i]; k < krowM[i+1]; k++ ) {
	j = jcolM[k];
	y[j] += M[k] * axi;
      }
    }
  } else {
    for( i = 0; i < m; i++ ) {
      double axi = alpha * x[i * incx];
      for( k = krowM[i]; k < krowM[i+1]; k++ ) {
	j = jcolM[k];
	y[j * incy] += M[k] * axi;
      }
    }
  }
}

void doubleLexSort( int first[], int n,
		    int second[], double data[] );

void SparseStorage::symmetrize( int& info)
{
  int i, k, ku;

  int nnz = krowM[m];
  int * irowM = new int[ 2 * nnz ];
  
  info = 0;

  ku = nnz;
  for ( i = 0; i < m; i++ ) {
    // For all rows
    for( k = krowM[i]; k < krowM[i + 1]; k++ ) {
      // Loop over elements of row i
      irowM[k] = i;
      if ( i != jcolM[k] ) {
	// Not a diagonal element
	if ( ku >= len ) {
	  info++;
	} else {
	  // Add the element transpose to the scrambled matrix
	  irowM[ku] = jcolM[k];
	  jcolM[ku] = i;
	  M[ku]     = M[k];
	  ku++;
	}
      } // End not a diagonal element
    } // End loop over elements of column j
  } // End for all columns
  if( info != 0 ) {
    delete [] irowM;
    return;
  }
  nnz = ku;

  doubleLexSort( irowM, nnz, jcolM, M );
  i = 0;
  krowM[0] = 0;
  for( k = 0; k < nnz; k++ ) {
    for( ; i < irowM[k]; i++ ) {
      krowM[i + 1] = k;
    }
  }
  for( i++; i <= m; i++ ) {
    krowM[i] = nnz;
  }
  delete [] irowM;
}

void SparseStorage::getTransposePat( int row, int col,
					 int rowExtent, int colExtent,
					 int kpat[], int kcolM[], int irowM[] )
{
  int i, j, k, kout;
  const int dontPermuteCols = 0, doPermuteRows = 1;
  
  kout = 0;
  for( i = row; i < row + rowExtent; i++ ) {
    int kin = krowM[i];
    while( kin < krowM[i+1] && jcolM[kin] < col ) {
      kin++;
    }
	
    for( ; kin < krowM[i+1] && jcolM[kin] < col + colExtent; kin++ ) {
      irowM[kout] = i;
      kpat[kout] = kin;
      kout++;
    }
  }
  
  indexedLexSort( jcolM, kout, dontPermuteCols, irowM, 
		  doPermuteRows, kpat );

  j = 0;
  kcolM[0] = 0;
  for ( k = 0; k < kout; k++ ) {
    for( ; j + col < jcolM[ kpat[k] ]; j++ ) {
      assert( j + 1 < colExtent );
      kcolM[j + 1] = k;
    }
  }
  for( j++; j <= colExtent; j++ ) {
    kcolM[j] = kout;
  }
}

void SparseStorage::getFromPat( double data[], int ldata, int kpat[] )
{
  int k;
  for ( k = 0; k < ldata; k++ ) {
    data[k] = M[ kpat[k] ];
  }
}

void SparseStorage::randomize( double alpha, double beta, double * seed )
{
  int  i, k, NN, chosen, icurrent;

  double scale = beta - alpha;
  double shift = alpha/scale;

  double drand( double * );

  // Knuth's algorithm for choosing length elements out of NN elts.
  NN        = m * n;
  int length = (len <= NN) ? len : NN;
  chosen    = 0;
  icurrent  = 0;
  krowM[0]  = 0;
  for ( k = 0; k < NN; k++ ) {
    double r = drand( seed );
	
    if( (NN - k) * r < length - chosen ) {
      jcolM[chosen] = k % n;
      i             = k / n;

      if ( i > icurrent ) {
	for ( ; icurrent < i; icurrent++ ) {
	  krowM[icurrent + 1] = chosen;
	}
      }
      M[chosen]     = scale * (drand(seed) + shift);
      chosen++;
    }  	
  }
  for ( ; icurrent < m; icurrent++ ) {
    krowM[icurrent + 1] = length;
  }

  assert( chosen == length );

}

double SparseStorage::abmaxnorm()
{
  double norm = 0.0;
  int nnz = this->numberOfNonZeros();
  
  int i;
  for( i = 0; i < nnz; i++ ) {
    double fabsMi = fabs( M[i] );
    if ( fabsMi > norm ) norm = fabsMi;
  }
  return norm;
}


void SparseStorage::atPutDiagonal( int idiag, OoqpVector& vvec )
{
  SimpleVector & v = dynamic_cast<SimpleVector &>(vvec);
  this->atPutDiagonal( idiag, v.elements(), 1, v.n );
}

void SparseStorage::atPutDiagonal( int idiag,
				       double x[], int incx, int extent )
{
  int i;
  int info;
  for( i = idiag; i < idiag + extent; i++ ) { // Loop over elts to be put
    // Search for the diagonal elt.
    int lastk;
    lastk = krowM[i+1];
    for( int k = krowM[i]; k < lastk; k++ ) { // Loop over all elts in row
      if ( i >= jcolM[k] ) { // Found or past the diagonal
	if( i == jcolM[k] ) { // Found it, overwrite it.
	  M[k] = x[incx*(i - idiag)];
	} else { // Didn't find it, so insert it
	  this->atPutSpRow( i, &x[incx*(i - idiag)], 1, &i, info );
	}
	// Either way, bug out of the loop
	break;
      } // end if found or past the diagonal
    } // end loop over all elts
  } // end loop over elts to be put
}

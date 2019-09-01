/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "SimpleVector.h"
#include "SimpleVectorHandle.h"
#include "VectorUtilities.h"
#include "OoqpBlas.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int SimpleVector::numberOfNonzeros()
{
  int i, count = 0;
  for( i = 0; i < n; i++ ) {
    if( v[i] != 0 ) count++;
  }
  return count;
}

void SimpleVector::min( double& m, int& index )
{
  index = 0;
  m     = v[0];
  for( int i = 0; i < n; i++ ) {
    if( v[i] < m ) {
      m     = v[i];
      index = i;
    }
  }
}

int SimpleVector::isKindOf( int kind )
{
  return (kind == kSimpleVector);
}

void SimpleVector::copyIntoArray( double w[] ) const
{
  memcpy( w, this->v, n * sizeof( double ) );
}

void SimpleVector::copyFromArray( double w[] )
{
  memcpy( this->v, w, n * sizeof( double ) );
}

void SimpleVector::copyFromArray( char w[] )
{
  int i;
  for( i = 0; i < n; i++ ) {
    this->v[i] = w[i];
  }
}

SimpleVector::SimpleVector( int n_ ) : OoqpVector( n_ )
{
  preserveVec = 0;
  v = new double[n];
}

SimpleVector::SimpleVector( double * v_, int n_ )
  : OoqpVector( n_ )
{
  preserveVec = 1;
  v = v_;
}

SimpleVector::~SimpleVector()
{
  if( !preserveVec ) {
    delete [] v;
  }
}

void SimpleVector::setToZero()
{
  int i;
  for( i = 0; i < n; i++ ) v[i] = 0.0;
}

void SimpleVector::setToConstant( double c)
{
  int i;
  for( i = 0; i < n; i++ ) v[i] = c;
}

void SimpleVector::randomize( double alpha, double beta, double *ix )
{
  assert( beta > alpha);

  double drand(double *);
  double scale = beta - alpha;
  double shift = alpha/scale;

  int i;
  for( i = 0; i < n; i++ ) {
    v[i] = scale * (drand(ix) + shift);
  }
}

  
void SimpleVector::copyFrom( OoqpVector& vec )
{
  assert( vec.length() == n );

  vec.copyIntoArray( this->v );
}

double SimpleVector::infnorm()
{
  double norm = 0;
  int i;
  for( i = 0; i < n; i++ ) {
    double temp = fabs( v[i] );
    // Subtle reversal of the logic to handle NaNs
    if( ! ( temp <=  norm ) ) norm = temp;
  }

/*
  if(norm > 1.e-8) {
    for(int j=0; j<n; j++) {
      if(fabs(v[j]) > .1*norm) 
	cout << " element " << j << " is " << v[j] << endl;
    }
  }
*/

  return norm;
}

double SimpleVector::onenorm()
{
  double norm = 0;
  int i;
  for( i = 0; i < n; i++ ) {
    double temp = fabs( v[i] );
    norm += temp;
  }
  return norm;
}    

void SimpleVector::componentMult( OoqpVector& vec )
{
  assert( n == vec.length() );
  SimpleVector & sv = dynamic_cast<SimpleVector &>(vec);
  double * y = sv.v;
  int i;
  for( i = 0; i < n; i++ ) v[i] *= y[i];
}

void SimpleVector::scalarMult( double num)
{
int i;
for( i = 0; i < n; i++) v[i] *= num;
}

void SimpleVector::componentDiv ( OoqpVector& vec )
{
  assert( n == vec.length() );
  double * pv = v, *lv = v + n;

  SimpleVector & sv = dynamic_cast<SimpleVector &>(vec);
  double * y = sv.v;

  for( ; pv < lv; pv++, y++ ) *pv /= *y;
}

void SimpleVector::writeToStream(ostream& out) const
{
  this->writefToStream( out, "%{value}" );
}

void SimpleVector::writefToStream( ostream& out,
				       const char format[] ) const
{
  SimpleVectorHandle empty( new SimpleVector(0) );
  this->writefSomeToStream( out, format, *empty );
}

void SimpleVector::writefSomeToStream( ostream& out,
					   const char format[],
					   OoqpVector& select ) const
{
  SimpleVector & sselect = dynamic_cast<SimpleVector &>(select);
  double * s = 0;
  if( select.length() > 0 ) {
    s = sselect.v;
  } 
  int i;

  for( i = 0; i < n; i++ ) {
    if( !s || s[i] != 0.0 ) {
      int j = 0;
      char c;
      while( (c = format[j]) != 0 ) {
	if( c != '%' ) {
	  out << c;
	} else {
	  // Brain-dead variable substitution, but good enough for this
	  // simple case
	  if( 0 == strncmp( "{value}", &format[j + 1], 7 ) ) {
	    out << v[i];
	    j += 7;
	  } else if ( 0 == strncmp( "{index}", &format[j + 1], 7 ) ) {
	    out << i;
	    j += 7;
	  } else {
	    out << c;
	  }
	}
	j++;
      }
      out << endl;
    }
  }
}


void SimpleVector::scale( double alpha )
{
  int one = 1;
  dscal_( &n, &alpha, v, &one ); 
}

void SimpleVector::axpy( double alpha, OoqpVector& vec )
{
  assert( n == vec.length() );
  SimpleVector & sv = dynamic_cast<SimpleVector &>(vec);

  int one = 1;
  daxpy_( &n, &alpha, sv.v, &one, v, &one );
}

void SimpleVector::addConstant( double c )
{
  int i;
  for( i = 0; i < n; i++ ) v[i] += c;
}

void SimpleVector::gondzioProjection( double rmin, double rmax )
{
  int i;
  for( i = 0; i < n; i++ ) {
    if( v[i] < rmin ) {
      v[i] = rmin - v[i];
    } else if ( v[i] > rmax ) {
      v[i] = rmax - v[i];
    } else {
      v[i] = 0.0;
    }
    if( v[i] < -rmax ) v[i] = -rmax;
  }
}

void SimpleVector::axzpy( double alpha, OoqpVector& xvec,
			      OoqpVector& zvec )
{
  assert( n == xvec.length() &&
	  n == zvec.length() );

  SimpleVector & sxvec = dynamic_cast<SimpleVector &>(xvec);
  SimpleVector & szvec = dynamic_cast<SimpleVector &>(zvec);

  double * x = sxvec.v;
  double * z = szvec.v;
  double * lx = x + n;
  double * w = v;

  if( alpha == 1.0 ) {
    while( x < lx ) {
      *w += *x * *z;
      w++; x++; z++;
    }
  } else if ( alpha == -1 ) {
    while( x < lx ) {
      *w -= *x * *z;
      w++; x++; z++;
    }
  } else {
    while( x < lx ) {
      *w += alpha * *x * *z;
      w++; x++; z++;
    }
  }
}

void SimpleVector::axdzpy( double alpha, OoqpVector& xvec,
			       OoqpVector& zvec )
{
  SimpleVector & sxvec = dynamic_cast<SimpleVector &>(xvec);
  double * x = sxvec.v;
  SimpleVector & szvec = dynamic_cast<SimpleVector &>(zvec);
  double * z = szvec.v;
  
  assert( n == xvec.length() &&
	  n == zvec.length() );

  int i;
  for( i = 0; i < n; i++ ) {
    //if(x[i] > 0 && z[i] > 0) 
      v[i] += alpha * x[i] / z[i];
  }
}

void SimpleVector::axdzpy( double alpha, OoqpVector& xvec,
			       OoqpVector& zvec, OoqpVector& select )
{
  assert( n == xvec.length() &&
	  n == zvec.length() );

  SimpleVector & sxvec = dynamic_cast<SimpleVector &>(xvec);
  double * x = sxvec.v;
  SimpleVector & szvec = dynamic_cast<SimpleVector &>(zvec);
  double * z = szvec.v;
  SimpleVector & sselect = dynamic_cast<SimpleVector &>(select);
  double * s   = sselect.v;
  int i;
  if( alpha == 1.0 ) {
    for( i = 0; i < n; i++ ) {
      if( 0.0 != s[i] ) v[i] += x[i] / z[i];
    }
  } else if ( alpha == -1.0 ) {
    for( i = 0; i < n; i++ ) {
      if( 0.0 != s[i] ) v[i] -= x[i] / z[i];
    }
  } else {
    for( i = 0; i < n; i++ ) {
      if( 0.0 != s[i] ) v[i] += alpha * x[i] / z[i];
    }
  }
}

double SimpleVector::dotProductWith( OoqpVector& vec )
{
  assert( n == vec.length() );
  SimpleVector & svec = dynamic_cast<SimpleVector &>(vec);
  double * vvec = svec.v;
  
  double dot1 = 0.0;
  double dot2 = 0.0;

  const int size = 8196;
  int kmax       = n / size;
  int i          = 0;

  int k;
  for( k = 0; k < kmax; k++ ) {
    int imax = (k + 1) * 8196;
    for( ; i < imax; i++ ) {
      dot1 += v[i] * vvec[i];
    }
    dot2 += dot1;
    dot1  = 0;
  }
  for( ; i < n; i++ ) {
    dot1 += v[i] * vvec[i];
  }
  
  return dot2 + dot1;
}

double 
SimpleVector::shiftedDotProductWith( double alpha, OoqpVector& mystep,
					 OoqpVector& yvec,
					 double beta,  OoqpVector& ystep )
{
  assert( n == mystep.length() &&
	  n == yvec  .length() && 
	  n == ystep .length() );

  SimpleVector & syvec = dynamic_cast<SimpleVector &>(yvec);
  double * y = syvec.v;

  SimpleVector & smystep = dynamic_cast<SimpleVector &>(mystep);
  double * p = smystep.v;

  SimpleVector & systep = dynamic_cast<SimpleVector &>(ystep);
  double * q = systep.v;

  double dot1 = 0.0;
  double dot2 = 0.0;

  const int size = 8196;
  int kmax       = n / size;
  int i          = 0;

  int k;
  for( k = 0; k < kmax; k++ ) {
    int imax = (k + 1) * 8196;
    for( ; i < imax; i++ ) {
      dot1 += (v[i] + alpha * p[i]) * (y[i] + beta * q[i] );
    }
    dot2 += dot1;
    dot1  = 0;
  }
  for( ; i < n; i++ ) {
    dot1 += (v[i] + alpha * p[i]) * (y[i] + beta * q[i] );
  }
  
  return dot2 + dot1;
}

void SimpleVector::negate()
{
  int i;
  for( i = 0; i < n; i++ ) v[i] = -v[i];
}

void SimpleVector::invert()
{
  int i;
  for( i = 0; i < n; i++ ) v[i] = 1/v[i];
}

int SimpleVector::allPositive()
{
  int i;
  for( i = 0; i < n; i++ ) {
    if( v[i] <= 0 ) return 0;
  }
  return 1;
}

double SimpleVector::stepbound(OoqpVector & pvec, double maxStep )
{
  assert( n == pvec.length() );

  SimpleVector & spvec = dynamic_cast<SimpleVector &>(pvec);
  double * p = spvec.v;
  double * w = v;
  double bound = maxStep;

  int i;
  for( i = 0; i < n; i++ ) {
    double temp = p[i];
    if( w[i] >= 0 && temp < 0 ) {
      temp = -w[i]/temp;
      if( temp < bound ) {
	bound = temp;
      }
    }
  }
  return bound;
}

double SimpleVector::findBlocking(OoqpVector & wstep_vec, 
				      OoqpVector & u_vec, 
				      OoqpVector & ustep_vec, 
				      double maxStep,
				      double *w_elt, 
				      double *wstep_elt,
				      double *u_elt, 
				      double *ustep_elt,
				      int& first_or_second)
{
  double * w     = v;
  SimpleVector & swstep = dynamic_cast<SimpleVector &>(wstep_vec);
  double * wstep = swstep.v;

  SimpleVector & su_vec = dynamic_cast<SimpleVector &>(u_vec);
  double * u     = su_vec.v;

  SimpleVector & sustep_vec = dynamic_cast<SimpleVector &>(ustep_vec);
  double * ustep = sustep_vec.v;

  return ::find_blocking( w, n, 1, wstep, 1, u, 1, ustep, 1, maxStep,
			  w_elt, wstep_elt, u_elt, ustep_elt,
			  first_or_second );
}

int SimpleVector::matchesNonZeroPattern( OoqpVector& select )
{
  SimpleVector & sselect = dynamic_cast<SimpleVector &>(select);
  double * map = sselect.v;

  double * lmap = map + n;
  assert( n == select.length() );

  double *w = v;
  while( map < lmap ) {
    if( *map == 0.0 && *w != 0.0  ) return 0;
    map++;
    w++;
  }

  return 1;
}

void SimpleVector::selectNonZeros( OoqpVector& select )
{
  SimpleVector & sselect = dynamic_cast<SimpleVector &>(select);
  double * map = sselect.v;

  assert( n == select.length() );
  int i;
  for( i = 0; i < n; i++ ) {
    if( 0.0 == map[i] ) v[i] = 0.0;
  }
}

void SimpleVector::addSomeConstants( double c, OoqpVector& select )
{
  SimpleVector & sselect = dynamic_cast<SimpleVector &>(select);
  double * map = sselect.v;

  int i;
  assert( n == select.length() );
  for( i = 0; i < n; i++ ) {
    if( map[i] ) v[i] += c;
  }
}

int SimpleVector::somePositive( OoqpVector& select )
{
  SimpleVector & sselect = dynamic_cast<SimpleVector &>(select);
  double * map = sselect.v;

  assert( n == select.length() );

  int i;
  for( i = 0; i < n; i++ ) {
    if( 0.0 != map[i] && v[i] <= 0 ) {
      cout << "Element " << i << " is nonpositive: " << v[i] << endl;
      return 0;
    }
  }
  return 1;
}

void SimpleVector::divideSome( OoqpVector& div, OoqpVector& select )
{
  if( n == 0 ) return;

  SimpleVector & sselect = dynamic_cast<SimpleVector &>(select);
  double * map = sselect.v;

  SimpleVector & sdiv = dynamic_cast<SimpleVector &>(div);
  double * q   = sdiv.v;
  assert( n == div.length() && n == select.length() );

  double * lmap = map + n;
  double * w = v;
  while( map < lmap ) {
    if( 0 != *map ) {
      *w  /= *q;
    }
    map++;
    w++;
    q++;
  }
}

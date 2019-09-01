/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

/*
 * Edited by: Quan H. Nguyen 
 */
#include "PetscVector.h"
#include "SimpleVector.h"
#include "SimpleVectorHandle.h"
#include "VectorUtilities.h"

#include "mpi.h"

// Some old versions of MPI_Uni don't define MPI_DOUBLE_INT
#ifndef MPI_DOUBLE_INT
struct ooqp_mpi_double_int{ double a; int i; };
#define MPI_DOUBLE_INT sizeof( ooqp_mpi_double_int )
#endif

int PetscVector::instances = 0;
PetscVector::PetscVector( int n_ ) : OoqpVector( n_ )
{
//    if ( n == 0 ) {
//      pv = 0;
//      preserveVec = 1;
//      return;
//    }
  int ierr;
  ierr = VecCreate( PETSC_COMM_WORLD, &pv ); assert(ierr == 0);
  ierr = VecSetSizes( pv, PETSC_DECIDE, n ); assert(ierr == 0);

  ierr = VecSetFromOptions( pv ); assert(ierr == 0);
  ierr = VecGetLocalSize( pv, &m ); assert( ierr  == 0);
  preserveVec = 0;
  PetscVector::instances++;
}

PetscVector::PetscVector( Vec v ) 
{
  pv = v;
  int ierr;
  ierr = VecGetSize( pv, &n ); assert( ierr == 0 );
  ierr = VecGetLocalSize( pv, &m ); assert( ierr  == 0);
  preserveVec = 1;
  PetscVector::instances++;
}

PetscVector::PetscVector( int m_in, int n_ ) : OoqpVector( n_ )
{
  int ierr;

  ierr = VecCreate( PETSC_COMM_WORLD, &pv ); assert(ierr == 0);
  ierr = VecSetSizes( pv, m_in, n ); assert(ierr == 0);
  ierr = VecSetFromOptions( pv ); assert(ierr == 0);
  ierr = VecGetLocalSize( pv, &m ); assert(ierr == 0);
  preserveVec = 0;
  PetscVector::instances++;
}

PetscVector::~PetscVector()
{
  int ierr;
  if( !preserveVec ) {
    ierr = VecDestroy( pv ); assert( ierr  == 0);
  }
  PetscVector::instances--;
}

int PetscVector::numberOfNonzeros()
{
  int ierr;
  double * v;
  int i, count = 0;

  ierr = VecGetArray( pv, &v ); assert( ierr  == 0);
  for( i = 0; i < m; i++ ) {
    if( v[i] != 0 ) count++;
  }
  ierr = VecRestoreArray( pv, &v ); assert( ierr  == 0);

  int allcount;
  ierr = MPI_Allreduce(&count, &allcount, 1,
		       MPI_INT, MPI_SUM,
		       PETSC_COMM_WORLD );
 
  return allcount;
}
int PetscVector::isKindOf( int kind )
{
  return (kPetscVector == kind );
}

void PetscVector::setToZero()
{
  int i, ierr;
  double * a;
  ierr = VecGetArray( pv, &a ); assert( ierr  == 0);

  for( i = 0; i < m; i++ ) a[i] = 0.0;

  ierr = VecRestoreArray( pv, &a ); assert( ierr  == 0);
}

void PetscVector::setToConstant( double c )
{
  int i, ierr;
  double * a;
  ierr = VecGetArray( pv, &a ); assert( ierr  == 0);

  for( i = 0; i < m; i++ ) a[i] = c;

  ierr = VecRestoreArray( pv, &a ); assert( ierr  == 0);
}

void PetscVector::randomize( double alpha, double beta, double * /* ix */ )
{
  int ierr;
  PetscRandom rctx;

  ierr = PetscRandomCreate(PETSC_COMM_WORLD,RANDOM_DEFAULT,&rctx);
  assert(ierr == 0);
  ierr = PetscRandomSetInterval(rctx, alpha, beta ); assert(ierr == 0);
  ierr = VecSetRandom(pv, rctx); assert(ierr == 0);

  ierr = PetscRandomDestroy(rctx); assert(ierr == 0);
}

void PetscVector::copyIntoArray( double v[] ) const
{
  int ierr;

  Vec sv;
  IS is;
  VecScatter ctx;

  ierr = VecCreateSeqWithArray(PETSC_COMM_SELF, n, v, &sv); assert(ierr == 0);  
  ierr = ISCreateStride(PETSC_COMM_WORLD, n, 0, 1, &is); assert( ierr  == 0);
  ierr = VecScatterCreate( pv, is, sv, is, &ctx); assert( ierr  == 0);

  ierr = VecScatterBegin( pv, sv,INSERT_VALUES,SCATTER_FORWARD,
			  ctx);   assert( ierr  == 0);
  ierr = VecScatterEnd( pv, sv,INSERT_VALUES,SCATTER_FORWARD,
			ctx);   assert( ierr  == 0);

  ierr = VecScatterDestroy(ctx); assert( ierr  == 0);
  ierr = ISDestroy( is );  assert(ierr == 0);
  ierr = VecDestroy( sv ); assert(ierr == 0);
}

void PetscVector::copyFromArray( double v[] )
{
  int ierr;

  Vec sv;
  IS is;
  VecScatter ctx;

  ierr = VecCreateSeqWithArray(PETSC_COMM_SELF, n, v, &sv); assert(ierr == 0);  
  ierr = ISCreateStride(PETSC_COMM_WORLD, n, 0, 1, &is); assert( ierr  == 0);
  ierr = VecScatterCreate( sv, is, pv, is, &ctx); assert( ierr  == 0);

  ierr = VecScatterBegin( sv, pv,INSERT_VALUES,SCATTER_FORWARD,
			  ctx);   assert( ierr  == 0);
  ierr = VecScatterEnd( sv, pv,INSERT_VALUES,SCATTER_FORWARD,
			ctx);   assert( ierr  == 0);

  ierr = VecScatterDestroy(ctx); assert( ierr  == 0);
  ierr = ISDestroy( is );  assert(ierr == 0);
  ierr = VecDestroy( sv ); assert(ierr == 0);
}

void PetscVector::copyFromArray( char v[] )
{
  int ierr;

  Vec sv;
  IS is;
  VecScatter ctx;

  ierr = VecCreateSeq(PETSC_COMM_SELF, n, &sv); assert(ierr == 0);
  {
    double * a;
    ierr = VecGetArray( sv, &a ); assert( ierr  == 0);
    int i;
    for( i = 0; i < n; i++ ) {
      a[i] = v[i];
    }
    ierr = VecRestoreArray( sv, &a ); assert( ierr  == 0);
  }

  ierr = ISCreateStride(PETSC_COMM_WORLD, n, 0, 1, &is); assert( ierr  == 0);
  ierr = VecScatterCreate( sv, is, pv, is, &ctx); assert( ierr  == 0);

  ierr = VecScatterBegin( sv, pv,INSERT_VALUES,SCATTER_FORWARD,
			  ctx);   assert( ierr  == 0);
  ierr = VecScatterEnd( sv, pv,INSERT_VALUES,SCATTER_FORWARD,
			ctx);   assert( ierr  == 0);

  ierr = VecScatterDestroy(ctx); assert( ierr  == 0);
  ierr = ISDestroy( is );  assert(ierr == 0);
  ierr = VecDestroy( sv ); assert(ierr == 0);
}

void PetscVector::copyFrom( OoqpVector& v_in )
{
  assert( n == v_in.length() );

  if( v_in.isKindOf( kPetscVector ) ) {
    PetscVector & pet = (PetscVector &) v_in;
    int ierr = VecCopy( pet.pv, pv );
    assert( ierr  == 0);
  } else if ( v_in.isKindOf( kSimpleVector ) ) {
    SimpleVector & sv = (SimpleVector &) v_in;
    this->copyFromArray( sv.elements() );
  } else {
    assert( 0 && "Can't copy from unknown OoqpVector type" );
  }
}

double PetscVector::infnorm()
{
  int ierr;
  double norm;

  ierr = VecNorm(pv, NORM_INFINITY, &norm ); assert( ierr  == 0);

  return norm;
}


void PetscVector::componentMult( OoqpVector& v_in )
{
  int ierr;
  PetscVector & v = dynamic_cast<PetscVector &>(v_in);

  double * x, *z;
  ierr = VecGetArray( this->pv, &x ); assert( ierr  == 0);
  ierr = VecGetArray( v    .pv, &z ); assert( ierr  == 0);
  int k;
  for( k = 0; k < m; k++ ) x[k] *= z[k];

  ierr = VecRestoreArray( v    .pv, &z ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &x ); assert( ierr  == 0);
}

void PetscVector::componentDiv ( OoqpVector& v_in )
{
  int ierr;
  PetscVector & v = dynamic_cast<PetscVector &>(v_in);

  double * x, *z;
  ierr = VecGetArray( this->pv, &x ); assert( ierr  == 0);
  ierr = VecGetArray( v    .pv, &z ); assert( ierr  == 0);
  int k;
  for( k = 0; k < m; k++ ) x[k] /= z[k];

  ierr = VecRestoreArray( v    .pv, &z ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &x ); assert( ierr  == 0);
}

void PetscVector::writeToStream(ostream& out) const
{
  this->writefToStream( out, "%{value}" );
}

void PetscVector::writefToStream( ostream& out,
				      const char format[] ) const
{
  PetscVectorHandle empty( new PetscVector(0) );
  this->writefSomeToStream( out, format, *empty );
}


void PetscVector::scale( double alpha )
{
  int ierr;
  ierr = VecScale( pv, alpha ); assert( ierr  == 0);
}


void PetscVector::axpy  ( double alpha, OoqpVector& x )
{
  int ierr;
  PetscVector & px = dynamic_cast<PetscVector &>(x);
  
  ierr = VecAXPY( pv, alpha, px.pv ); assert( ierr  == 0);
}

void PetscVector::axzpy ( double alpha, OoqpVector& vx_in, OoqpVector& vz_in )
{
  int ierr;
  PetscVector & vx = dynamic_cast<PetscVector &>(vx_in);
  PetscVector & vz = dynamic_cast<PetscVector &>(vz_in);

  assert( vx.getLocalSize() == m && vz.getLocalSize() == m );
  double *x, *y, *z;
  ierr = VecGetArray( vx   .pv, &x ); assert( ierr  == 0);
  ierr = VecGetArray( this->pv, &y ); assert( ierr  == 0);
  ierr = VecGetArray( vz   .pv, &z ); assert( ierr  == 0);
  
  int i;
  for( i = 0; i < m; i++ ) y[i] += alpha * x[i] * z[i];

  ierr = VecRestoreArray( vz   .pv, &z ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &y ); assert( ierr  == 0);
  ierr = VecRestoreArray( vx   .pv, &x ); assert( ierr  == 0);
}

void PetscVector::axdzpy( double alpha, OoqpVector& vx_in,
			      OoqpVector& vz_in )
{
  int ierr;

  PetscVector & vx = dynamic_cast<PetscVector &>(vx_in);
  PetscVector & vz = dynamic_cast<PetscVector &>(vz_in);

  assert( vx.getLocalSize() == m && vz.getLocalSize() == m );
  double *x, *y, *z;
  ierr = VecGetArray( vx   .pv, &x ); assert( ierr  == 0);
  ierr = VecGetArray( this->pv, &y ); assert( ierr  == 0);
  ierr = VecGetArray( vz   .pv, &z ); assert( ierr  == 0);
  
  int i;
  for( i = 0; i < m; i++ ) y[i] += alpha * x[i] / z[i];

  ierr = VecRestoreArray( vz   .pv, &z ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &y ); assert( ierr  == 0);
  ierr = VecRestoreArray( vx   .pv, &x ); assert( ierr  == 0);
}

#undef __FUNC__
#define __FUNC__ PetscVector::ADDCONSTANT
void PetscVector::addConstant( double c )
{
  double * a;
  int ierr, i;

  ierr = VecGetArray( pv, &a ); assert( ierr  == 0);
  for( i =0; i < m; i++ ) a[i] += c;
  ierr = VecRestoreArray( pv, &a ); assert( ierr  == 0);
}

void PetscVector::gondzioProjection( double rmin, double rmax )
{
  double * v;
  int ierr;
  
  ierr = VecGetArray( pv, &v ); assert( ierr  == 0);

  int i;
  for( i = 0; i < m; i++ ) {
    if( v[i] < rmin ) {
      v[i] = rmin - v[i];
    } else if ( v[i] > rmax ) {
      v[i] = rmax - v[i];
    } else {
      v[i] = 0.0;
    }

    if( v[i] < -rmax ) v[i] = -rmax;
  }
  ierr = VecRestoreArray( pv, &v ); assert( ierr  == 0);
}

double PetscVector::dotProductWith( OoqpVector& v )
{
  int ierr;
  double dot;

  PetscVector & w = dynamic_cast<PetscVector &>(v);
  
  ierr = VecDot( pv, w.pv, &dot); assert( ierr  == 0);
  
  return dot;
}

double PetscVector::shiftedDotProductWith( double alpha, OoqpVector& pvec_in,
					       OoqpVector& yvec_in,
					       double beta,  OoqpVector& qvec_in )
{
  PetscVector & pvec = dynamic_cast<PetscVector &>(pvec_in);
  PetscVector & yvec = dynamic_cast<PetscVector &>(yvec_in);
  PetscVector & qvec = dynamic_cast<PetscVector &>(qvec_in);

  double *v, *p, *y, *q;
  int ierr;
  ierr = VecGetArray( this->pv, &v ); assert( ierr  == 0);
  ierr = VecGetArray( pvec .pv, &p ); assert( ierr  == 0);
  ierr = VecGetArray( yvec .pv, &y ); assert( ierr  == 0);
  ierr = VecGetArray( qvec .pv, &q ); assert( ierr  == 0);

  double dot = 0;
  int i;
  for( i = 0; i < m; i++ ) {
    dot += (v[i] + alpha * p[i]) * (y[i] + beta * q[i] );
  }
  
  ierr = VecRestoreArray( qvec .pv, &q ); assert( ierr  == 0);
  ierr = VecRestoreArray( yvec .pv, &y ); assert( ierr  == 0);
  ierr = VecRestoreArray( pvec .pv, &p ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &v ); assert( ierr  == 0);

  double alldot = 0;
  ierr = MPI_Allreduce(&dot, &alldot, 1,
		       MPI_DOUBLE, MPI_SUM,
		       PETSC_COMM_WORLD );
  assert(ierr == 0);

  return alldot;
}

void PetscVector::negate()
{
  int ierr;
  double * a;
  ierr = VecGetArray( pv, &a ); assert( ierr  == 0);
  int k;
  for( k = 0; k < m; k++ ) a[k] = -a[k];
  ierr = VecRestoreArray( pv, &a ); assert( ierr  == 0);
}

void PetscVector::invert()
{
  int ierr = VecReciprocal( pv ); assert( ierr  == 0);
}

int PetscVector::allPositive()
{
  int good = 1;
  double * v;
  int ierr;
  ierr = VecGetArray( pv, &v ); assert(ierr == 0);
  int i;
  for( i = 0; i < m; i++ ) {
    if( v[i] <= 0 ) {
      good = 0;
      break;
    }
  }
  ierr = VecRestoreArray( pv, &v ); assert(ierr == 0);

  int allgood;
  ierr = MPI_Allreduce(&good, &allgood, 1,
		       MPI_INT, MPI_LAND,
		       PETSC_COMM_WORLD );
  return allgood;

}


int PetscVector::matchesNonZeroPattern( OoqpVector& select )
{
  PetscVector & sv = dynamic_cast<PetscVector &>(select);
 
  assert( sv.getLocalSize() == m );

  int ierr;
  double *a, *s;
  ierr = VecGetArray( this->pv, &a ); assert( ierr  == 0);
  ierr = VecGetArray( sv   .pv, &s ); assert( ierr  == 0);
  int i, good = 1;
  for( i = 0; i < m; i++ ) {
    if( a[i] != 0 && s[i] == 0 ) {
      good = 0;
      break;
    }
  }
  ierr = VecRestoreArray( sv   .pv, &s ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &a ); assert( ierr  == 0);

  int allgood;
  ierr = MPI_Allreduce(&good, &allgood, 1,
		       MPI_INT, MPI_LAND,
		       PETSC_COMM_WORLD );
  return allgood;
}

void PetscVector::selectNonZeros( OoqpVector& select )
{
  PetscVector & sv = dynamic_cast<PetscVector &>(select);
 
  assert( sv.getLocalSize() == m );

  int ierr;
  double *a, *s;
  ierr = VecGetArray( this->pv, &a ); assert( ierr  == 0);
  ierr = VecGetArray( sv   .pv, &s ); assert( ierr  == 0);
  int i;
  for( i = 0; i < m; i++ ) {
    if( s[i] == 0 ) a[i] = 0.0;
  }
  ierr = VecRestoreArray( sv   .pv, &s ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &a ); assert( ierr  == 0);
}

void PetscVector::addSomeConstants( double c, OoqpVector& select )
{
  PetscVector & sv = dynamic_cast<PetscVector &>(select);
 
  assert( sv.getLocalSize() == m );

  int ierr;
  double *a, *s;
  ierr = VecGetArray( this->pv, &a ); assert( ierr  == 0);
  ierr = VecGetArray( sv   .pv, &s ); assert( ierr  == 0);
  int i;
  for( i = 0; i < m; i++ ) {
    if( s[i] != 0 ) a[i] += c;
  }
  ierr = VecRestoreArray( sv   .pv, &s ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &a ); assert( ierr  == 0);

}

void PetscVector::writefSomeToStream( ostream& out,
					  const char format[],
					  OoqpVector& select ) const
{
  int ierr, rank;
  ierr = MPI_Comm_rank( PETSC_COMM_WORLD, &rank ); assert( ierr == 0 );
  
  int lm = (rank == 0) ? n : 0; // Only the rank 0 process does output
  
  Vec pvalues;
  ierr = VecCreateMPI( PETSC_COMM_WORLD, lm, n, &pvalues );
  assert( ierr == 0 );

  // Pull all values into the rank 0 process
  IS is;
  ierr = ISCreateStride(PETSC_COMM_WORLD, n, 0, 1, &is); assert( ierr  == 0);
  VecScatter ctx;
  ierr = VecScatterCreate( pv, is, pvalues, is, &ctx); assert( ierr  == 0);

  ierr = VecScatterBegin( pv, pvalues,INSERT_VALUES,SCATTER_FORWARD,
			  ctx);   assert( ierr  == 0);
  ierr = VecScatterEnd( pv, pvalues,INSERT_VALUES,SCATTER_FORWARD,
			ctx);   assert( ierr  == 0);

  double * values;
  ierr = VecGetArray( pvalues, &values ); assert( ierr == 0 );
  
  PetscVector & pvselect = dynamic_cast<PetscVector &>(select);

  if( pvselect.getSize() == 0 ) { // We don't have a select vector
    if( rank == 0 ) { // We are the rank 0 process, let' do some output
      SimpleVectorHandle vv( new SimpleVector(values, n) );
      vv->writefToStream( out, format );
    }
  } else { // We do have a select vector, need to gather it's values
    Vec pselect;
    ierr = VecCreateMPI( PETSC_COMM_WORLD, lm, n, &pselect );
    assert( ierr == 0 );
    ierr = VecScatterBegin( pvselect.pv, pselect, INSERT_VALUES,
			    SCATTER_FORWARD, ctx);   assert( ierr  == 0);

    ierr = VecScatterEnd( pvselect.pv, pselect,INSERT_VALUES,
			  SCATTER_FORWARD, ctx);   assert( ierr  == 0);
    double * dselect;
    ierr = VecGetArray( pselect, &dselect ); assert( ierr == 0 );
    if( rank == 0  ) { // if we are the rank 0 process, do some output
      SimpleVectorHandle vv( new SimpleVector(values, n) );
      SimpleVectorHandle ss( new SimpleVector(dselect, n) );
      vv->writefSomeToStream( out, format, *ss );
    }
    ierr = VecRestoreArray( pselect, &dselect ); assert( ierr == 0 );

    ierr = VecDestroy( pselect ); assert(ierr == 0);
  }

  ierr = VecRestoreArray( pvalues, &values ); assert( ierr == 0 );

  ierr = VecScatterDestroy(ctx); assert( ierr  == 0);
  ierr = ISDestroy( is );  assert(ierr == 0);
  ierr = VecDestroy( pvalues ); assert(ierr == 0);

}

void PetscVector::axdzpy( double alpha, OoqpVector& vx_in,
			      OoqpVector& vz_in, OoqpVector& sel )
{
  int ierr;
  PetscVector & vx = dynamic_cast<PetscVector &>(vx_in);
  PetscVector & vz = dynamic_cast<PetscVector &>(vz_in);
  PetscVector & vmap = dynamic_cast<PetscVector &>(sel);


  assert( vx  .getLocalSize() == m && vz.getLocalSize() == m &&
	  vmap.getLocalSize() == m );
  double *x, *y, *z, *mp;

  ierr = VecGetArray( vx   .pv, &x );  assert( ierr  == 0);
  ierr = VecGetArray( this->pv, &y );  assert( ierr  == 0);
  ierr = VecGetArray( vz   .pv, &z );  assert( ierr  == 0);
  if( &vmap == &vx ) { 
    mp = x; 
  } else if ( &vmap == &vz ) {
    mp = z;
  } else {
    ierr = VecGetArray( vmap.pv, &mp ); assert( ierr  == 0);
  }
  
  int i;
  for( i = 0; i < m; i++ ) {
    if( mp[i] != 0.0 ) y[i] += alpha * x[i] / z[i];
  }
  if( &vmap != &vx && &vmap != &vz ) {
    ierr = VecRestoreArray( vmap.pv, &z ); assert( ierr  == 0);
  }
  ierr = VecRestoreArray( vz   .pv, &z ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &y ); assert( ierr  == 0);
  ierr = VecRestoreArray( vx   .pv, &x ); assert( ierr  == 0);
}


int PetscVector::somePositive( OoqpVector& select )
{
  PetscVector & sv = dynamic_cast<PetscVector &>(select);
 
  assert( sv.getLocalSize() == m );

  int ierr;
  double *a, *s;
  ierr = VecGetArray( this->pv, &a ); assert( ierr  == 0);
  ierr = VecGetArray( sv   .pv, &s ); assert( ierr  == 0);
  int i, good = 1;
  for( i = 0; i < m; i++ ) {
    if( s[i] != 0 && a[i] <= 0 ) {
      good = 0;
      break;
    }
  }
  ierr = VecRestoreArray( sv   .pv, &s ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &a ); assert( ierr  == 0);

  int allgood;
  ierr = MPI_Allreduce(&good, &allgood, 1,
		       MPI_INT, MPI_LAND,
		       PETSC_COMM_WORLD );
  return allgood;
}

void PetscVector::divideSome( OoqpVector& vecb, OoqpVector& select )
{
  PetscVector & sv = dynamic_cast<PetscVector &>(select);
  PetscVector & vb = dynamic_cast<PetscVector &>(vecb);
 
  assert( sv.getLocalSize() == m );

  int ierr;
  double *a, *b, *s;
  ierr = VecGetArray( this->pv, &a ); assert( ierr  == 0);
  ierr = VecGetArray( vb   .pv, &b ); assert( ierr  == 0);
  ierr = VecGetArray( sv   .pv, &s ); assert( ierr  == 0);
  int i;
  for( i = 0; i < m; i++ ) {
    if( s[i] != 0 ) a[i] /= b[i];
  }
  ierr = VecRestoreArray( sv   .pv, &s ); assert( ierr  == 0);
  ierr = VecRestoreArray( vb   .pv, &b ); assert( ierr  == 0);
  ierr = VecRestoreArray( this->pv, &a ); assert( ierr  == 0);
}


double PetscVector::stepbound(OoqpVector & svec_in,
				  double bound )
{
  PetscVector & svec = dynamic_cast<PetscVector &>(svec_in);
  
  double *s, *v;
  int ierr;
  ierr = VecGetArray( pv,       &v ); assert( 0 == ierr );
  ierr = VecGetArray( svec.pv, &s ); assert( 0 == ierr );

  bound = ::stepbound( v, m, 1, s, 1, bound );

  double allbound;
  ierr = MPI_Allreduce( &bound, &allbound, 1, MPI_DOUBLE, MPI_MIN,
			PETSC_COMM_WORLD );
  assert( 0 == ierr );

  ierr = VecRestoreArray( svec.pv, &s ); assert( 0 == ierr );
  ierr = VecRestoreArray( pv,       &v ); assert( 0 == ierr );

  return allbound;
}

//  void showpv( OoqpVector * vec, int first, int lastp1 ) 
//  {
//    int ierr;
//    assert( vec->isKindOf( kPetscVector ) );
//    PetscVector * v =  (PetscVector *) vec;
  
//    int low, high;
//    ierr = VecGetOwnershipRange( v->pv, &low, &high ); assert(ierr == 0);
//    if( low < first )   low  = first;
//    if( high > lastp1 ) high = lastp1;

//    if( low < high ) {
//      double * a;
//      ierr = VecGetArray    ( v->pv, &a ); assert(ierr == 0);
//      int i;
//      for( i = 0; i < high - low; i += 4) {
//        int j;
//        cout << i + low << ": ";
//        for( j = 0; j < 4 && i + j < high - low; j++ ) {
//  	cout << a[i + j] << "    ";
//        }
//        cout << endl;
//      }
//      ierr = VecRestoreArray( v->pv, &a ); assert(ierr == 0);
//    }
//  } 


double PetscVector::onenorm()
{
  double norm;
  int ierr;
  ierr = VecNorm(pv, NORM_1, &norm ); assert( ierr  == 0);

  return norm;

}

double PetscVector::findBlocking(OoqpVector & wstep_vec, 
				     OoqpVector & u_vec, 
				     OoqpVector & ustep_vec, 
				     double maxStep,
				     double *w_elt, 
				     double *wstep_elt,
				     double *u_elt, 
				     double *ustep_elt,
				     int& first_or_second)
{
  int ierr;
  double bound;
  
  {
    PetscVector & wstep_pvec = dynamic_cast<PetscVector &>(wstep_vec);
    PetscVector & u_pvec     = dynamic_cast<PetscVector &>(u_vec);
    PetscVector & ustep_pvec = dynamic_cast<PetscVector &>(ustep_vec);
    {
      double *w, *wstep, *u, *ustep;

      ierr = VecGetArray( pv,            &w );     assert( 0 == ierr );
      ierr = VecGetArray( wstep_pvec.pv, &wstep ); assert( 0 == ierr );
      ierr = VecGetArray( u_pvec.pv,     &u );     assert( 0 == ierr );
      ierr = VecGetArray( ustep_pvec.pv, &ustep ); assert( 0 == ierr );
      
      bound =
	::find_blocking( w, m, 1, wstep, 1, u, 1, ustep, 1, maxStep,
			 w_elt, wstep_elt, u_elt, ustep_elt,
			 first_or_second );
      
      ierr = VecRestoreArray( pv,            &w );     assert( 0 == ierr );
      ierr = VecRestoreArray( wstep_pvec.pv, &wstep ); assert( 0 == ierr );
      ierr = VecRestoreArray( u_pvec.pv,     &u );     assert( 0 == ierr );
      ierr = VecRestoreArray( ustep_pvec.pv, &ustep ); assert( 0 == ierr );
    }
  }
  int selfrank, minrank;
  ierr = MPI_Comm_rank( PETSC_COMM_WORLD, &selfrank); assert( 0 == ierr );
  {
    struct { 
      double val; 
      int   rank; 
    } mini, allmini;
    
    mini.val    = bound; mini.rank    = selfrank;
    allmini.val = 0;     allmini.rank = 0;
    ierr = MPI_Allreduce( &mini, &allmini, 1, MPI_DOUBLE_INT, MPI_MINLOC,
			  PETSC_COMM_WORLD ); 
    assert( 0 == ierr );
    bound = allmini.val;   minrank = allmini.rank;
  }
  {
    double steps[4] = { *w_elt, *wstep_elt, *u_elt, *ustep_elt };
    ierr = MPI_Bcast( steps, 4, MPI_DOUBLE, minrank, PETSC_COMM_WORLD );
    assert( 0 == ierr );
    *w_elt = steps[0];     *wstep_elt = steps[1];
    *u_elt = steps[2];     *ustep_elt = steps[3];
  }
//    cout << "fos1 " << first_or_second << " rank " << selfrank
//         << " arank " << minrank;

  ierr = MPI_Bcast( &first_or_second, 1, MPI_INT,
		    minrank, PETSC_COMM_WORLD );
  assert( 0 == ierr );
  

//    cout << " fos2 " << first_or_second << " rank " << selfrank << endl;
  return bound;
}

void PetscVector::min( double& min, int& index )
{
  int ierr = VecMin( pv, &index, &min );

  assert( ierr == 0 );
}

void PetscVector::scalarMult( double num){
	scale(num);
}



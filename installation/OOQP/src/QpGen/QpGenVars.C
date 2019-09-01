/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "QpGenVars.h"
#include "QpGenData.h"
#include "OoqpVector.h"
#include "Data.h"
#include "QpGenResiduals.h"
#include "QpGenLinsys.h"
#include "SimpleVector.h"
#include "MpsReader.h"

#include "LinearAlgebraPackage.h"

#include <iostream> 
#include <fstream>
using namespace std;

QpGenVars::QpGenVars( OoqpVector * x_in, OoqpVector * s_in,
		      OoqpVector * y_in, OoqpVector * z_in,
		      OoqpVector * v_in, OoqpVector * gamma_in,
		      OoqpVector * w_in, OoqpVector * phi_in,
		      OoqpVector * t_in, OoqpVector * lambda_in,
		      OoqpVector * u_in, OoqpVector * pi_in,
		      OoqpVector * ixlow_in, OoqpVector * ixupp_in,
		      OoqpVector * iclow_in, OoqpVector * icupp_in )
{
  SpReferTo( x, x_in );
  SpReferTo( s, s_in );
  SpReferTo( y, y_in );
  SpReferTo( z, z_in );
  SpReferTo( v, v_in );
  SpReferTo( phi, phi_in );
  SpReferTo( w, w_in );
  SpReferTo( gamma, gamma_in );
  SpReferTo( t, t_in );
  SpReferTo( lambda, lambda_in );
  SpReferTo( u, u_in );
  SpReferTo( pi, pi_in );
  SpReferTo( ixlow, ixlow_in );
  SpReferTo( ixupp, ixupp_in );
  SpReferTo( iclow, iclow_in );
  SpReferTo( icupp, icupp_in );

  nx = x->length();
  my = y->length();
  mz = z->length();

  assert( nx == ixlow->length() || 0 == ixlow->length() );
  assert( nx == ixlow->length() || 0 == ixlow->length() );
  assert( mz == iclow->length()  || 0 == iclow->length() );
  assert( mz == icupp->length() || 0 == icupp->length() );
  
  nxlow = ixlow->numberOfNonzeros();
  nxupp = ixupp->numberOfNonzeros();
  mclow = iclow->numberOfNonzeros();
  mcupp = icupp->numberOfNonzeros();
  nComplementaryVariables = mclow + mcupp + nxlow + nxupp;

  assert( mz == s->length() );
  assert( nx == v     ->length() || ( 0 == v     ->length() && nxlow == 0 ));
  assert( nx == gamma ->length() || ( 0 == gamma ->length() && nxlow == 0 ));

  assert( nx == w     ->length() || ( 0 == w     ->length() && nxupp == 0 ));
  assert( nx == phi   ->length() || ( 0 == phi   ->length() && nxupp == 0 ));

  assert( mz == t     ->length() || ( 0 == t     ->length() && mclow == 0 ));
  assert( mz == lambda->length() || ( 0 == lambda->length() && mclow == 0 ));

  assert( mz == u     ->length() || ( 0 == u     ->length() && mcupp == 0 ));
  assert( mz == pi    ->length() || ( 0 == pi    ->length() && mcupp == 0 ));
}

QpGenVars::QpGenVars( LinearAlgebraPackage * la,
		      int nx_, int my_, int mz_,
		      OoqpVector * ixlow_in, OoqpVector * ixupp_in,
		      OoqpVector * iclow_in, OoqpVector * icupp_in )
{
  SpReferTo( ixlow, ixlow_in );
  SpReferTo( ixupp, ixupp_in );
  SpReferTo( iclow, iclow_in );
  SpReferTo( icupp, icupp_in );

  nx = nx_;
  my = my_;
  mz = mz_;

  assert( nx == ixlow->length() || 0 == ixlow->length() );
  nxlow = ixlow->numberOfNonzeros();
  
  assert( nx == ixlow->length() || 0 == ixlow->length() );
  nxupp = ixupp->numberOfNonzeros();

  assert( mz == iclow->length()  || 0 == iclow->length() );
  mclow = iclow->numberOfNonzeros();
  
  assert( mz == icupp->length() || 0 == icupp->length() );
  mcupp = icupp->numberOfNonzeros();
  
  s = OoqpVectorHandle( la->newVector( mz ) );
  if ( mclow > 0 ) {
    t      = OoqpVectorHandle( la->newVector( mz ) );
    lambda = OoqpVectorHandle( la->newVector( mz ) );
  } else {
    t      = OoqpVectorHandle( la->newVector( 0 ) );
    lambda = OoqpVectorHandle( la->newVector( 0 ) );
  }
  if( mcupp > 0 ) {
    u  = OoqpVectorHandle( la->newVector( mz ) );
    pi = OoqpVectorHandle( la->newVector( mz ) );
  } else {
    u  = OoqpVectorHandle( la->newVector( 0 ) );
    pi = OoqpVectorHandle( la->newVector( 0 ) );
  }
  if( nxlow > 0 ) {
    v     = OoqpVectorHandle( la->newVector( nx ) );
    gamma = OoqpVectorHandle( la->newVector( nx ) );
  } else {
    v     = OoqpVectorHandle( la->newVector( 0 ) );
    gamma = OoqpVectorHandle( la->newVector( 0 ) );
  }
	
  if( nxupp > 0 ) {
    w   = OoqpVectorHandle( la->newVector( nx ) );
    phi = OoqpVectorHandle( la->newVector( nx ) );
  } else {
    w   = OoqpVectorHandle( la->newVector( 0 ) );
    phi = OoqpVectorHandle( la->newVector( 0 ) );
  }

  x = OoqpVectorHandle( la->newVector( nx ) );
  y = OoqpVectorHandle( la->newVector( my ) );
  z = OoqpVectorHandle( la->newVector( mz ) );
  nComplementaryVariables = mclow + mcupp + nxlow + nxupp;

}

double QpGenVars::mu()
{
  if ( nComplementaryVariables == 0 ) {
    return 0.0;
  } else {
    double mu  = 0.0;
    if( mclow > 0 ) mu += t->dotProductWith( *lambda );
    if( mcupp > 0 ) mu += u->dotProductWith( *pi );
    if( nxlow > 0 ) mu += v->dotProductWith( *gamma );
    if( nxupp > 0 ) mu += w->dotProductWith( *phi );

    mu /= nComplementaryVariables;

    return mu;
  }
}

double QpGenVars::mustep(Variables * step_in, double alpha)
{
  QpGenVars * step = (QpGenVars *) step_in;
  if ( nComplementaryVariables == 0 ) {
    return 0.0;
  } else {
    double mu  = 0.0;
    if( mclow > 0 ) {
      mu += t->shiftedDotProductWith( alpha, *step->t,
				      *lambda,
				      alpha, *step->lambda );
    }
    if( mcupp > 0 ) {
      mu += u->shiftedDotProductWith( alpha, *step->u,
				      *pi,
				      alpha, *step->pi );
    }
    if( nxlow > 0 ) {
      mu += v->shiftedDotProductWith( alpha, *step->v,
				      *gamma,
				      alpha, *step->gamma );
    }
    if( nxupp > 0 ) {
      mu += w->shiftedDotProductWith( alpha, *step->w,
				      *phi,
				      alpha, *step->phi );
    }
    mu /= nComplementaryVariables;
    return mu;
  }
}

void QpGenVars::saxpy( Variables *b_in, double alpha )
{
  QpGenVars * b = (QpGenVars *) b_in;

  x->axpy( alpha, *b->x );
  y->axpy( alpha, *b->y );
  z->axpy( alpha, *b->z );
  s->axpy( alpha, *b->s );
  if( mclow > 0 ) {
    assert( b->t     ->matchesNonZeroPattern( *iclow ) &&
	    b->lambda->matchesNonZeroPattern( *iclow ) );

    t     ->axpy( alpha, *b->t );
    lambda->axpy( alpha, *b->lambda );
  }
  if( mcupp > 0 ) {
    assert( b->u     ->matchesNonZeroPattern( *icupp ) &&
	    b->pi    ->matchesNonZeroPattern( *icupp ) );

    u     ->axpy( alpha, *b->u );
    pi    ->axpy( alpha, *b->pi );
  }
  if( nxlow > 0 ) {
    assert( b->v     ->matchesNonZeroPattern( *ixlow ) &&
	    b->gamma ->matchesNonZeroPattern( *ixlow ) );

    v     ->axpy( alpha, *b->v );
    gamma ->axpy( alpha, *b->gamma );
  }
  if( nxupp > 0 ) {
    assert( b->w     ->matchesNonZeroPattern( *ixupp ) &&
	    b->phi   ->matchesNonZeroPattern( *ixupp ) );

    w     ->axpy( alpha, *b->w );
    phi   ->axpy( alpha, *b->phi );
  }
}

void QpGenVars::negate()
{
  s->negate();
  x->negate();
  y->negate();
  z->negate();
  if( mclow > 0 ) {
    t     ->negate();
    lambda->negate();
  }
  if( mcupp > 0 ) {
    u     ->negate();
    pi    ->negate();
  }
  if( nxlow > 0 ) {
    v     ->negate();
    gamma ->negate();
  }
  if( nxupp > 0 ) {
    w     ->negate();
    phi   ->negate();
  }
}

double QpGenVars::stepbound( Variables * b_in )
{
  QpGenVars * b = (QpGenVars *) b_in;
  double maxStep;

  maxStep = 1.0;

  if( mclow > 0 ) {
    assert( t     ->somePositive( *iclow ) );
    assert( lambda->somePositive( *iclow ) );

    maxStep = t     ->stepbound( *b->t, maxStep );
    maxStep = lambda->stepbound( *b->lambda, maxStep );
  }

  if( mcupp > 0 ) {
    assert( u ->somePositive( *icupp ) );
    assert( pi->somePositive( *icupp ) );

    maxStep = u ->stepbound( *b->u,  maxStep );
    maxStep = pi->stepbound( *b->pi, maxStep );
  }
  
  if( nxlow > 0 ) {
    assert( v    ->somePositive( *ixlow ) );
    assert( gamma->somePositive( *ixlow ) );

    maxStep = v    ->stepbound( *b->v,     maxStep );
    maxStep = gamma->stepbound( *b->gamma, maxStep );
  }

  if( nxupp > 0 ) {
    assert( w  ->somePositive( *ixupp ) );
    assert( phi->somePositive( *ixupp ) );

    maxStep = w  ->stepbound( *b->w,   maxStep );
    maxStep = phi->stepbound( *b->phi, maxStep );
  }

  return maxStep;
}
  
int QpGenVars::isInteriorPoint()
{
  int interior = 1;
  if( mclow > 0 ) {
    interior = interior &&
      t     ->somePositive( *iclow ) &&
      lambda->somePositive( *iclow );
  }

  if( mcupp > 0 ) {
    interior = interior &&
      u ->somePositive( *icupp ) &&
      pi->somePositive( *icupp );
  }
  
  if( nxlow > 0 ) {
    interior = interior &&
      v    ->somePositive( *ixlow ) &&
      gamma->somePositive( *ixlow );
  }

  if( nxupp > 0 ) {
    interior = interior &&
      w  ->somePositive( *ixupp ) &&
      phi->somePositive( *ixupp );
  }

  return interior;
}

double QpGenVars::findBlocking( Variables * step, 
				double & primalValue,
				double & primalStep,
				double & dualValue,
				double & dualStep,
				int& firstOrSecond )
{
  double alpha = 1.0;
  firstOrSecond = 0;

  QpGenVars * d = (QpGenVars *) step;

  if( mclow > 0 ) {
    alpha = t->findBlocking( *d->t, *lambda, *d->lambda, alpha,
			     &primalValue, &primalStep,
			     &dualValue, &dualStep,	
			     firstOrSecond );
  }

  if( mcupp > 0 ) {
    alpha = u->findBlocking( *d->u, *pi, *d->pi, alpha,
			     &primalValue, &primalStep,
			     &dualValue, &dualStep,
			     firstOrSecond );
  }
  
  if( nxlow > 0 ) {
    alpha = v->findBlocking( *d->v, *gamma, *d->gamma, alpha,
			     &primalValue, &primalStep,
			     &dualValue, &dualStep,
			     firstOrSecond );
  }

  if( nxupp > 0 ) {
    alpha = w->findBlocking( *d->w, *phi, *d->phi, alpha,
			     &primalValue, &primalStep,
			     &dualValue, &dualStep,
			     firstOrSecond );
  }

  return alpha;
}



//  void QpGenVars::start(Data *prob, Residuals * resid, LinearSystem * sys,
//  		      Variables * step )
//  {
//    s->setToZero();
//    x->setToZero();
//    y->setToZero();
//    z->setToZero();

//    // set the r3 component of the rhs to -(norm of data), and calculate
//    // the residuals that are obtained when all values are zero.

//    double sdatanorm = prob->datanorm();
//    double alpha  = 0.0;
//    double beta   = 0.0;
//    double calpha = 0.0;
//    double cbeta  = 0.0;

//    if( nxlow > 0 ) {
//      v     ->setToConstant ( alpha );
//      v     ->selectNonZeros( ixlow );
//      gamma ->setToConstant ( beta  );
//      gamma ->selectNonZeros( ixlow );
//    }
//    if( nxupp > 0 ) {
//      w     ->setToConstant ( alpha );
//      w     ->selectNonZeros( ixupp );
//      phi   ->setToConstant ( beta  );
//      phi   ->selectNonZeros( ixupp );
//    }

//    if( mclow > 0 ) {
//      t      ->setToConstant ( calpha );
//      t      ->selectNonZeros( iclow );
//      lambda ->setToConstant ( cbeta  );
//      lambda ->selectNonZeros( iclow );
//    }
//    if( mcupp > 0 ) {
//      u      ->setToConstant ( calpha );
//      u      ->selectNonZeros( icupp );
//      pi     ->setToConstant ( cbeta  );
//      pi     ->selectNonZeros( icupp );
//    }
//    resid->set_r3_xz_alpha( this, -sdatanorm );
//    resid->calcresids( prob, this );

//    // next, assign 1 to all the complementary variables, so that there
//    // are identities in the coefficient matrix when we do the solve.

//    alpha  = 1.0;
//    beta   = 1.0;
//    calpha = 1.0;
//    cbeta  = 1.0;

//    if( nxlow > 0 ) {
//      v     ->setToConstant ( alpha );
//      v     ->selectNonZeros( ixlow );
//      gamma ->setToConstant ( beta  );
//      gamma ->selectNonZeros( ixlow );
//    }
//    if( nxupp > 0 ) {
//      w     ->setToConstant ( alpha );
//      w     ->selectNonZeros( ixupp );
//      phi   ->setToConstant ( beta  );
//      phi   ->selectNonZeros( ixupp );
//    }

//    if( mclow > 0 ) {
//      t      ->setToConstant ( calpha );
//      t      ->selectNonZeros( iclow );
//      lambda ->setToConstant ( cbeta  );
//      lambda ->selectNonZeros( iclow );
//    }
//    if( mcupp > 0 ) {
//      u      ->setToConstant ( calpha );
//      u      ->selectNonZeros( icupp );
//      pi     ->setToConstant ( cbeta  );
//      pi     ->selectNonZeros( icupp );
//    }

//    //  resid->calcresids( prob, this);

//    sys->factor(prob, this);
//    sys->solve (prob, this, resid, step);
//    step->negate();

//    // copy the "step" into the current vector

//    this->copy(step);
//    double violation = 0.0, cmin = 0.0;
//    int iblock;

//    if( nxlow > 0 ) {
//      v->min( cmin, iblock );
//      if( cmin < violation ) violation = cmin;
    
//      gamma->min( cmin, iblock );
//      if( cmin < violation ) violation = cmin;
//    }
//    if( nxupp > 0 ) {
//      w->min( cmin, iblock );
//      if( cmin < violation ) violation = cmin;
    
//      phi->min( cmin, iblock );
//      if( cmin < violation ) violation = cmin;
//    }
//    if( mclow > 0 ) {
//      t->min( cmin, iblock );
//      if( cmin < violation ) violation = cmin;
    
//      lambda->min( cmin, iblock );
//      if( cmin < violation ) violation = cmin;
//    }
//    if( mcupp > 0 ) {
//      u->min( cmin, iblock );
//      if( cmin < violation ) violation = cmin;
    
//      pi->min( cmin, iblock );
//      if( cmin < violation ) violation = cmin;
//    }

//    //  cout << "violation is " << violation << endl;
//    double shift = 0.0;
//    if(violation <= 0.0) shift = -1.5 * violation;

//    if( nxlow > 0 ) {
//      v     ->addSomeConstants( shift, ixlow );
//      gamma ->addSomeConstants( shift, ixlow );
//    }
//    if( nxupp > 0 ) {
//      w     ->addSomeConstants( shift, ixupp );
//      phi   ->addSomeConstants( shift, ixupp );
//    }
//    if( mclow > 0 ) {
//      t     ->addSomeConstants( shift, iclow );
//      lambda->addSomeConstants( shift, iclow );
//    }
//    if( mcupp > 0 ) {
//      u     ->addSomeConstants( shift, icupp );
//      pi    ->addSomeConstants( shift, icupp );
//    }

//    // do Mehrotra-type adjustment

//    double mutemp = this->mu();
//    double snorm=0.e0, xnorm=0.e0;
//    if( nxlow > 0 ) {
//      xnorm += v->onenorm();
//      snorm += gamma->onenorm();
//    }
//    if( nxupp > 0 ) {
//      xnorm += w->onenorm();
//      snorm += phi->onenorm();
//    }
//    if( mclow > 0 ) {
//      xnorm += t->onenorm();
//      snorm += lambda->onenorm();
//    }
//    if( mcupp > 0 ) {
//      xnorm += u->onenorm();
//      snorm += pi->onenorm();
//    }

//    cout << "xnorm = " << xnorm << endl;
//    cout << "snorm = " << snorm << endl;

//    double deltax = 0.5 * (nxlow+nxupp+mclow+mcupp) * mutemp / snorm;
//    double deltas = 0.5 * (nxlow+nxupp+mclow+mcupp) * mutemp / xnorm;

//    if( nxlow > 0 ) {
//      v     ->addSomeConstants( deltax, ixlow );
//      gamma ->addSomeConstants( deltas, ixlow );
//    }
//    if( nxupp > 0 ) {
//      w     ->addSomeConstants( deltax, ixupp );
//      phi   ->addSomeConstants( deltas, ixupp );
//    }
//    if( mclow > 0 ) {
//      t     ->addSomeConstants( deltax, iclow );
//      lambda->addSomeConstants( deltas, iclow );
//    }
//    if( mcupp > 0 ) {
//      u     ->addSomeConstants( deltax, icupp );
//      pi    ->addSomeConstants( deltas, icupp );
//    }

//  }

void QpGenVars::interiorPoint( double alpha, double beta )
{
  s->setToZero();
  x->setToZero();
  y->setToZero();
  z->setToZero();

  if( nxlow > 0 ) {
    v     ->setToConstant ( alpha );
    v     ->selectNonZeros( *ixlow );
    gamma ->setToConstant ( beta  );
    gamma ->selectNonZeros( *ixlow );
  }
  if( nxupp > 0 ) {
    w     ->setToConstant ( alpha );
    w     ->selectNonZeros( *ixupp );
    phi   ->setToConstant ( beta  );
    phi   ->selectNonZeros( *ixupp );
  }

  if( mclow > 0 ) {
    t      ->setToConstant ( alpha );
    t      ->selectNonZeros( *iclow );
    lambda ->setToConstant ( beta  );
    lambda ->selectNonZeros( *iclow );
  }
  if( mcupp > 0 ) {
    u      ->setToConstant ( alpha );
    u      ->selectNonZeros( *icupp );
    pi     ->setToConstant ( beta  );
    pi     ->selectNonZeros( *icupp );
  }
}
    
double QpGenVars::violation()
{
  double viol = 0.0, cmin = 0.0;
  int iblock;

  if( nxlow > 0 ) {
    v->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
    
    gamma->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
  }
  if( nxupp > 0 ) {
    w->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
    
    phi->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
  }
  if( mclow > 0 ) {
    t->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
    
    lambda->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
  }
  if( mcupp > 0 ) {
    u->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
    
    pi->min( cmin, iblock );
    if( cmin < viol ) viol = cmin;
  }
  return -viol;
}

void QpGenVars::shiftBoundVariables( double alpha, double beta )
{
  if( nxlow > 0 ) {
    v     ->addSomeConstants( alpha, *ixlow );
    gamma ->addSomeConstants( beta,  *ixlow );
  }
  if( nxupp > 0 ) {
    w     ->addSomeConstants( alpha, *ixupp );
    phi   ->addSomeConstants( beta,  *ixupp );
  }
  if( mclow > 0 ) {
    t     ->addSomeConstants( alpha, *iclow );
    lambda->addSomeConstants( beta,  *iclow );
  }
  if( mcupp > 0 ) {
    u     ->addSomeConstants( alpha, *icupp );
    pi    ->addSomeConstants( beta,  *icupp );
  }
}


void QpGenVars::print()
{
  x->writefToStream( cout, "x[%{index}] = %{value}" );
}
  
void QpGenVars::copy(Variables *b_in)
{
  QpGenVars * b = (QpGenVars *) b_in;
  
  s->copyFrom( *b->s );
  if( nxlow > 0 ) {
    v->copyFrom( *b->v );
    gamma ->copyFrom( *b->gamma );
  }
  if( nxupp > 0 ) {
    w->copyFrom( *b->w );
    phi   ->copyFrom( *b->phi );
  }
  if( mclow > 0 ) {
    t->copyFrom( *b->t );
    lambda->copyFrom( *b->lambda );
  }
  if( mcupp > 0 ) {
    u->copyFrom( *b->u );
    pi    ->copyFrom( *b->pi );
  }
  x->copyFrom( *b->x );
  y->copyFrom( *b->y );
  z->copyFrom( *b->z );
  
}

double QpGenVars::onenorm()
{
  double norm;
  norm  = x->onenorm();
  norm += s->onenorm();
  norm += y->onenorm();
  norm += z->onenorm();

  norm += v->onenorm();
  norm += phi->onenorm();
  norm += w->onenorm();
  norm += gamma->onenorm();
  norm += t->onenorm();
  norm += lambda->onenorm();
  norm += u->onenorm();
  norm += pi->onenorm();

  return norm;
}


double QpGenVars::infnorm()
{
  double norm, temp;
  norm = 0.0;

  temp  = x->infnorm();
  if(temp > norm) norm = temp;
  temp = s->infnorm();
  if(temp > norm) norm = temp;
  temp = y->infnorm();
  if(temp > norm) norm = temp;
  temp = z->infnorm();
  if(temp > norm) norm = temp;

  temp = v->infnorm();
  if(temp > norm) norm = temp;
  temp = phi->infnorm();
  if(temp > norm) norm = temp;

  temp = w->infnorm();
  if(temp > norm) norm = temp;
  temp = gamma->infnorm();
  if(temp > norm) norm = temp;

  temp = t->infnorm();
  if(temp > norm) norm = temp;
  temp = lambda->infnorm();
  if(temp > norm) norm = temp;

  temp = u->infnorm();
  if(temp > norm) norm = temp;
  temp = pi->infnorm();
  if(temp > norm) norm = temp;

  return norm;
}

QpGenVars::~QpGenVars()
{
}

int QpGenVars::validNonZeroPattern()
{
  if( nxlow > 0 && 
      ( !v    ->matchesNonZeroPattern( *ixlow ) ||
	!gamma->matchesNonZeroPattern( *ixlow ) ) ) {
    return 0;
  }

  if( nxupp > 0 &&
      ( !w  ->matchesNonZeroPattern( *ixupp ) ||
	!phi->matchesNonZeroPattern( *ixupp ) ) ) {
    return 0;
  }
  if( mclow > 0 &&
      ( !t     ->matchesNonZeroPattern( *iclow ) ||
	!lambda->matchesNonZeroPattern( *iclow ) ) ) {
    return 0;
  }

  if( mcupp > 0 &&
      ( !u ->matchesNonZeroPattern( *icupp ) ||
	!pi->matchesNonZeroPattern( *icupp ) ) ) {
    return 0;
  }
  
  return 1;
}

void QpGenVars::unscaleSolution(QpGenData * data)
{

// Modifying sx is equivalent to modifying x
SimpleVector & sx = (SimpleVector &) *this->x;

// x = D * x'
sx.componentMult(data->scale());
}

void QpGenVars::unscaleBounds(QpGenData * data)
{

SimpleVector & sxlow = (SimpleVector &) data-> xlowerBound();
SimpleVector & sxupp = (SimpleVector &) data-> xupperBound();

// l = D * l' 
sxlow.componentMult(data->scale());

// u = D * u'
sxupp.componentMult(data->scale());
}

void QpGenVars::printSolution( MpsReader * reader, QpGenData * data, 
			       int scale, int& iErr )
{
  assert( x->isKindOf( kSimpleVector ) ); // Otherwise this routine
  // cannot be used.
  double objective;
  {
    SimpleVectorHandle temp( new SimpleVector(nx) );
    data->getg( *temp );
    data->Qmult( 1.0, *temp, 0.5, *x );
    objective = temp->dotProductWith( *x );
  }

  SimpleVector & sx      = (SimpleVector &) *this->x;
  SimpleVector & sxlow   = (SimpleVector &) data-> xlowerBound();
  SimpleVector & sixlow  = (SimpleVector &) data->ixlowerBound();
  SimpleVector & sxupp   = (SimpleVector &) data-> xupperBound();
  SimpleVector & sixupp  = (SimpleVector &) data->ixupperBound();
  SimpleVector & sgamma  = (SimpleVector &) *this->gamma;
  SimpleVector & sphi    = (SimpleVector &) *this->phi;
  SimpleVector & sy      = (SimpleVector &) *this->y;
  SimpleVector & ss      = (SimpleVector &) *this->s;
  SimpleVector & slambda = (SimpleVector &) *this->lambda;
  SimpleVector & spi     = (SimpleVector &) *this->pi;
  SimpleVector & sz      = (SimpleVector &) *this->z;
  SimpleVector & sclow   = (SimpleVector &) data-> slowerBound();
  SimpleVector & siclow  = (SimpleVector &) data->islowerBound();
  SimpleVector & scupp   = (SimpleVector &) data-> supperBound();
  SimpleVector & sicupp  = (SimpleVector &) data->isupperBound();

  char * cxupp = new char[nx];
  char * cxlow = new char[nx];
  for( int j = 0; j < nx; j++ ) {
    if( nxupp > 0 && sixupp[j] != 0 ) {
      cxupp[j] = 1;
    } else {
      cxupp[j] = 0;
    }
    if( nxlow > 0 && sixlow[j] != 0 ) {
      cxlow[j] = 1;
    } else {
      cxlow[j] = 0;
    }
  }
  char *cclow, *ccupp;
  if( mz <= 0 ) {
    cclow = 0; ccupp = 0;
  } else {
    cclow = new char[mz];
    ccupp = new char[mz];
    for( int i = 0; i < mz; i++ ) {
      if( mclow > 0 && siclow[i] != 0.0 ) {
	cclow[i] = 1;
      } else {
	cclow[i] = 0;
      }
      if( mcupp > 0 && sicupp[i] != 0.0 ) {
	ccupp[i] = 1;
      } else {
	ccupp[i] = 0;
      }
    }
  }

  if( scale ) {
      // Unscale the solution and bounds before printing
      this->unscaleSolution( data);
      this->unscaleBounds( data);
      }

  reader->printSolution( sx.elements(), nx,
			 sxlow.elements(), cxlow, sxupp.elements(), cxupp,
			 sgamma.elements(), sphi.elements(),
			 sy.elements(), my,
			 ss.elements(), mz,
			 sclow.elements(), cclow,
			 scupp.elements(), ccupp,
			 slambda.elements(), spi.elements(),
			 sz.elements(), 
			 objective,
			 iErr );
  delete [] cclow;
  delete [] ccupp;
  delete [] cxlow;
  delete [] cxupp;
}

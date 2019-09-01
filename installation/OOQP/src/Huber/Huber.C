/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "Huber.h"
#include "HuberData.h"
#include "HuberResiduals.h"
#include "HuberVars.h"
#include "HuberLinsys.h"

Data * Huber::makeData(  int nobservations, int npredictors, double cutoff, 
			 double * dXt, double * dY )
{
  return new HuberData( nobservations, npredictors, cutoff, dXt, dY );
}

Data * Huber::makeRandomData( int nobservations, int npredictors,
			      double cutoff )
{
  HuberData * data =
    (HuberData *) this->makeData( nobservations, npredictors, cutoff );
  data->datarandom();
  return data;
}

Data * Huber::makeDataFromText(char  filename[], double cutoff, int& iErr)
{
  HuberData * data = HuberData::textInput(filename, cutoff, iErr);

  return data;
}

Residuals * Huber::makeResiduals( Data * prob_in )
{
  HuberData * prob = (HuberData *) prob_in;

  return new HuberResiduals( prob->nobservations, prob->npredictors );
}

LinearSystem * Huber::makeLinsys( Data * prob_in )
{
  return new HuberLinsys( prob_in );
}

Variables * Huber::makeVariables( Data * prob_in )
{
  HuberData * prob = (HuberData *) prob_in;

  return new HuberVars( prob->nobservations, prob->npredictors );
}

Variables * Huber::makeVariables( Data * prob_in,
				  double * beta, double * t,
				  double * lambda1, double * lambda2,
				  double * gamma1,  double * gamma2 )
{
  HuberData * prob = (HuberData *) prob_in;

  return new HuberVars( prob->nobservations, prob->npredictors, beta, t,
			lambda1, lambda2, gamma1, gamma2 );
}

Huber::~Huber()
{
}



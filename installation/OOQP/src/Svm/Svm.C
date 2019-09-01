/* OOQP                                                               *
 * Authors: E. Michael Gertz, Stephen J. Wright                       *
 * (C) 2001 University of Chicago. See Copyright Notification in OOQP */

#include "Svm.h"
#include "SvmData.h"
#include "SvmResiduals.h"
#include "SvmVars.h"
#include "SvmLinsys.h"
#include "DoubleMatrix.h"

// SVMFactory::SVMFactory( )
// {
// }

Data * Svm::makeData(int hyperplanedim, int nobservations, double penalty )
{
  return new SvmData(hyperplanedim, nobservations, penalty );
}

Data * Svm::makeData(int hyperplanedim, int nobservations,
		     double * X, double * d, double penalty )
{
  return new SvmData(hyperplanedim, nobservations, X, d, penalty );
}

Data * Svm::makeRandomData(int hyperplanedim, int nobservations,
			   double penalty )
{
  SvmData * data = (SvmData *) this->makeData(hyperplanedim, nobservations,
					      penalty);
  data->datarandom();
  return data;
}

Data * Svm::makeDataFromText(char  filename[], double penalty, int& iErr)
{
  SvmData * data = SvmData::textInput(filename, penalty, iErr);
  return data;
} 

Residuals * Svm::makeResiduals( Data * prob_in )
{
  SvmData * prob = (SvmData *) prob_in;

  return new SvmResiduals( prob->hyperplanedim, prob->nobservations );
}

LinearSystem * Svm::makeLinsys( Data * prob_in )
{
  SvmLinsys * svmls = new SvmLinsys();
  svmls->create( prob_in);

  return svmls;
}

Variables * Svm::makeVariables( Data * prob_in )
{
  SvmData * prob = (SvmData *) prob_in;
  return new SvmVars( prob->hyperplanedim, prob->nobservations  );
}

Variables * Svm::makeVariables( Data * prob_in, 
				double w[], double v[], 
				double z[], double u[],
				double s[] )
{
  SvmData * prob = (SvmData *) prob_in;
  return new SvmVars( prob->hyperplanedim, prob->nobservations,
		      w, v, z, u, s  );
}


Svm::~Svm()
{
}



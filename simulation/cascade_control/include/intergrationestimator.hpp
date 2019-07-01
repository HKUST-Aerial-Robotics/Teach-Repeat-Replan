#ifndef INTERGRATIONESTIMATOR_HPP
#define INTERGRATIONESTIMATOR_HPP

#include <Eigen/Eigen>
#include <chrono>

#include "estimator.hpp"

namespace mocka
{

class IntergrationEstimator : public Estimator<double, double>
{
public:
  typedef Estimator<double, double> EstimatorBase;
  using EstimatorBase::In;
  using EstimatorBase::Out;

  typedef std::chrono::time_point<std::chrono::steady_clock> TimePoint;

public:
  IntergrationEstimator();

public:
  virtual Out update(const In& in);

private:
  TimePoint now;
};

} // namespace mocka

#endif // INTERGRATIONESTIMATOR_HPP

#include "intergrationestimator.hpp"

using namespace mocka;

IntergrationEstimator::IntergrationEstimator()
{
}

IntergrationEstimator::EstimatorBase::Out
IntergrationEstimator::update(
  const IntergrationEstimator::EstimatorBase::In& in)
{
  auto tmp = std::chrono::steady_clock::now();

  std::chrono::duration<double> dt = tmp - now;
  now                              = tmp;
  if (isInitialized())
  {
    out += std::chrono::duration_cast<std::chrono::milliseconds>(dt).count() /
           1000.0 * in;
    return out;
  }
  return std::numeric_limits<Out>::quiet_NaN();
}

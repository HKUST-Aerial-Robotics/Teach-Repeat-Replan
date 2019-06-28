#ifndef MIXER_HPP
#define MIXER_HPP

#include <Eigen/Core>
#include <cmath>

namespace mocka
{

template <typename T>
class Mixer
{
public:
  Mixer();

public:
  T mix(const T input, const T feedforward);

public: // getters
  bool getSaturated() const;
  T    getThreshold() const;

public: // setters
  void setThreshold(const T& value);

private:
  T    threshold;
  bool saturated;
};

template <typename T>
Mixer<T>::Mixer()
  : saturated(false)
{
}

class VelocityMixer : public Mixer<Eigen::Vector3d>
{
  // public:
  //  Eigen::Vector3d mix(const Eigen::Vector3d input,
  //                      const Eigen::Vector3d feedforward);
};

//! @note there's no default mix function
// template <typename T>
// T
// Mixer<T>::mix(const T input, const T feedforward)
//{
//  T ans = input;
//  if (std::abs(input) < threshold)
//  {
//    ans       = input + feedforward;
//    saturated = true;
//  }
//  else
//  {
//    saturated = false;
//  }
//  return ans;
//}

template <typename T>
bool
Mixer<T>::getSaturated() const
{
  return saturated;
}

template <typename T>
T
Mixer<T>::getThreshold() const
{
  return threshold;
}

template <typename T>
void
Mixer<T>::setThreshold(const T& value)
{
  threshold = value;
}

} // namespace mocka

#endif // MIXER_HPP

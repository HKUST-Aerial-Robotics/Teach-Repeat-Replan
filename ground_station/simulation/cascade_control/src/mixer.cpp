#include "mixer.hpp"

namespace mocka
{
//! @note translation mixer
template <>
Eigen::Vector3d
Mixer<Eigen::Vector3d>::mix(const Eigen::Vector3d input,
                            const Eigen::Vector3d feedforward)
{
  Eigen::Vector3d ans = input;
  if (input.norm() < threshold.norm())
  {
    saturated = true;
  }
  else
  {
    saturated = false;
  }
  ans = input + feedforward;
  return ans;
}

// Eigen::Vector3d
// VelocityMixer::mix(const Eigen::Vector3d input,
//                   const Eigen::Vector3d feedforward)
//{
//  Eigen::Vector3d ans;

//  ans.x() = input.x() / (input.x() + feedforward.x()) * input.x() +
//            feedforward.x() / (input.x() + feedforward.x()) * feedforward.x();

//  ans.y() = input.y() / (input.y() + feedforward.y()) * input.y() +
//            feedforward.y() / (input.y() + feedforward.y()) * feedforward.y();

//  ans.z() = input.z() / (input.z() + feedforward.z()) * input.z() +
//            feedforward.z() / (input.z() + feedforward.z()) * feedforward.z();

//  return ans;
//}

} // namespace mocka

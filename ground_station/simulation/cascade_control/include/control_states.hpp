#ifndef CONTROL_STATES_HPP
#define CONTROL_STATES_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mocka
{

class SE3;
class SO3;

class SE3
{
public:
  SE3();

public:
  Eigen::Vector3d acceleration;
  Eigen::Vector3d viewPoint;

public:
  SO3 toSO3(const Eigen::Vector3d& viewAxis,
            const Eigen::Vector3d& thrustAxis) const;
};

class SO3
{
public:
  SO3();

public:
  Eigen::Quaternion<double> quaternion;
  double                    thrust;

public:
  SE3 toSE3(const Eigen::Vector3d& viewAxis,
            const Eigen::Vector3d& thrustAxis) const;

public:
  //! @note GCC feature
  static constexpr double g = 9.81;
};

} // namespace mocka

#endif // CONTROL_STATES_HPP

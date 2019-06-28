#include "control_states.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <iostream>

using namespace mocka;

constexpr double SO3::g;

SE3::SE3()
  : acceleration(0, 0, SO3::g)
  , viewPoint(1, 0, 0)
{
}

SO3
SE3::toSO3(const Eigen::Vector3d& viewAxis,
           const Eigen::Vector3d& thrustAxis) const
{
  SO3 ans;

  ans.thrust = this->acceleration.norm();

  Eigen::Quaterniond qBase =
    Eigen::Quaterniond::FromTwoVectors(thrustAxis, this->acceleration);

  Eigen::Vector3d tmpView   = viewAxis.transpose() * qBase.toRotationMatrix();
  Eigen::Vector3d direction = acceleration;
  direction.normalize();
  double ropol = tmpView.dot(direction);
  double ropdl = this->viewPoint.dot(direction);

  Eigen::Vector3d    projOrigin = tmpView - ropol * direction;
  Eigen::Vector3d    projDes    = this->viewPoint - ropdl * direction;
  Eigen::Quaterniond aQ;
  if (projOrigin != projDes)
  {
    double aA =
      acos(projOrigin.dot(projDes) / (projOrigin.norm() * projDes.norm()));

    Eigen::Matrix3d aR;

    Eigen::Vector3d ccwv = projOrigin.cross(projDes);
    ccwv.normalize();

    aR << cos(ccwv.dot(direction) * aA), -sin(ccwv.dot(direction) * aA), 0,
      sin(ccwv.dot(direction) * aA), cos(ccwv.dot(direction) * aA), 0, 0, 0, 1;

    aQ = Eigen::Quaterniond(aR);
    //  debug info
    //    std::cout << "ccwv" << std::endl;
    //    std::cout << ccwv << std::endl
    //              << direction << std::endl
    //              << ccwv.dot(direction);
    //    std::cout << "aR!!!" << std::endl << aR << std::endl;
    //    std::cout << "aQ0" << std::endl << aQ.toRotationMatrix() << std::endl;
  }
  else
  {
    aQ = Eigen::Quaterniond(1, 0, 0, 0);
  }
  aQ.normalize();
  //  debug info
  //  std::cout << projOrigin << std::endl << projDes << std::endl;
  //  std::cout << "qB" << std::endl << qBase.toRotationMatrix() << std::endl;
  //  std::cout << "aQ1" << std::endl << aQ.toRotationMatrix() << std::endl;

  ans.quaternion = qBase * aQ;
  ans.quaternion.normalize();

  return ans;
}

SO3::SO3()
  : quaternion(1, 0, 0, 0)
  , thrust(g)
{
}

SE3
SO3::toSE3(const Eigen::Vector3d& viewAxis,
           const Eigen::Vector3d& thrustAxis) const
{
  SE3 ans;

  Eigen::Vector3d thrustNorm = thrustAxis;
  thrustNorm.normalize();

  Eigen::Vector3d accNorm =
    thrustNorm.transpose() * this->quaternion.toRotationMatrix();

  ans.acceleration = accNorm * this->thrust;

  ans.viewPoint = viewAxis.transpose() * this->quaternion.toRotationMatrix();

  return ans;
}

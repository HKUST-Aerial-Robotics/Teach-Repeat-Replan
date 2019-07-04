#include "so3control.hpp"

using namespace mocka;

SO3Controller::SO3Controller()
{
  view.setZero();
  thrust.setZero();
}

SO3Controller::~SO3Controller()
{
}

SO3Controller::Output
SO3Controller::update()
{
  //! @todo implement
  //! ENU frame not NED
  SO3Controller::Output ans;
  Eigen::Vector3d       FDM; // force divide mass
  //! @note FLU frame
  FDM = getInput().acceleration + SO3::g * Eigen::Vector3d(0, 0, 1);

  //! @todo calculate current FDM add consider its error to SE3 SO3 translation
  Eigen::Vector3d currentFDM;
  currentFDM = Eigen::Vector3d(0, 0, 1).transpose() *
               getFeeds().quaternion.toRotationMatrix();

  Eigen::Quaterniond q;

  if (view.norm() != 0)
  {
    SE3 se3;
    se3.viewPoint    = getInput().viewPoint;
    se3.acceleration = FDM;
    q                = se3.toSO3(view, thrust).quaternion;
  }
  ans.quaternion = q;
  ans.thrust     = FDM.norm();
  return ans;
}

void
SO3Controller::updateInput()
{
}

void
SO3Controller::updateFeeds()
{
}

void
SO3Controller::setView(Eigen::Vector3d value)
{
  view = value;
  view = view / view.norm();
}

Eigen::Vector3d
SO3Controller::getThrust() const
{
  return thrust;
}

void
SO3Controller::setThrust(Eigen::Vector3d value)
{
  thrust = value;
}

Eigen::Vector3d
SO3Controller::getView() const
{
  return view;
}

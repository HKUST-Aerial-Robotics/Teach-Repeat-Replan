#ifndef ATTI_HPP
#define ATTI_HPP

#include <Eigen/Geometry>

#include "controller.hpp"
#include "pid.hpp"

namespace mocka
{

class AttitudeController
  : public Controller<Eigen::Quaterniond, Eigen::Vector3d, Eigen::Quaterniond>
{
  //! @todo implement

public:
  AttitudeController();

protected:
  virtual Output update();
  virtual void   updateInput();
  virtual void   updateFeeds();

  Eigen::Vector3d getViewPoint();
};

} // namespace mocka

#endif // ATTI_HPP

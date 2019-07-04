#ifndef SO3_HPP
#define SO3_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "control_states.hpp"
#include "controller.hpp"

namespace mocka
{

class SO3Controller : public Controller<SE3, SO3, SO3>
{
public:
  SO3Controller();
  virtual ~SO3Controller();

public:
  //! @note getters
  Eigen::Vector3d getView() const;
  Eigen::Vector3d getThrust() const;

public:
  //! @note setters
  void setThrust(Eigen::Vector3d value);
  void setView(Eigen::Vector3d value);

protected:
  virtual Output update();
  virtual void   updateInput();
  virtual void   updateFeeds();

private:
  Eigen::Vector3d view;
  Eigen::Vector3d thrust;
};
} // namespace mocka

#endif // SO3_HPP

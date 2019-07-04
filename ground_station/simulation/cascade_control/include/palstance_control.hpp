#ifndef AEL_HPP
#define AEL_HPP

#include <Eigen/Geometry>

#include "controller.hpp"
#include "pid.hpp"

namespace mocka
{

class PalstanceController
  : public Controller<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d>
{
public:
  typedef enum {
    STRUCTURE_UNKNOWN,
    STRUCTURE_X4, // DJI default structure
    STRUCTURE_C4, // SO3 controller structure

  } MechanicalStructure;

public:
  PalstanceController();

protected:
  virtual Output update();
  virtual void   updateInput();
  virtual void   updateFeeds();

private:
  PID yaw;
  PID roll;
  PID pitch;

  MechanicalStructure structure;
};

} // namespace mocka

#endif // AEL_HPP

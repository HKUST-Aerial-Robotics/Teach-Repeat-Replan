#ifndef CASCADECONTROLLER_HPP
#define CASCADECONTROLLER_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "control_states.hpp"
#include "controller.hpp"
#include "intergrationestimator.hpp"
#include "mixer.hpp"

namespace mocka
{

class PositionController;
class VelocityController;
class AccelerationController;
class SO3Controller;
class AttitudeController;
class PalstanceController;
class ThrustController;
// class VelocityMixer;

typedef struct State
{
  //! @SE3 part
  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector3d acceleration;
  Eigen::Vector3d viewPoint;
  //! @SO3 part
  Eigen::Quaterniond quaternion;
  Eigen::Vector3d    palstance;
  double             thrust;
} State;

class CascadeController : public Controller<State, State, State>
{
public:
  CascadeController();
  virtual ~CascadeController();

public:
  virtual Output update();
  virtual void   updateInput();
  virtual void   updateFeeds();

public:
  //! @note getters

  Eigen::Vector3d getThrustAxies() const;
  Eigen::Vector3d getViewAxies() const;

public:
  //! @note setters

  /*! @note coverage class Controller setters
   *        templeate implemented in cascadecontrol.cpp,
   *        not necessary to be annouced here
   */
  //   void setInput(const Input& value);
  //   void setInput(const Input& value, const uint64_t& time);
  void setFeeds(const Feeds& value);
  void setFeeds(const Feeds& value, const uint64_t& time);

  //! @note SE3 feed setters
  void setFeedPosition(const Eigen::Vector3d& value);
  void setFeedVelocity(const Eigen::Vector3d& value);
  void setFeedAcceleration(const Eigen::Vector3d& value);
  void setFeedViewPoint(const Eigen::Vector3d& value);

  //! @note SO3 feed setters
  void setFeedQuaternion(const Eigen::Quaterniond& value);
  void setFeedPalstance(const Eigen::Vector3d& value);
  void setFeedThrust(const double& value);

  void setThrustAxies(const Eigen::Vector3d& value);
  void setViewAxies(const Eigen::Vector3d& value);

public:
  //! @note setters
  PositionController*     getControllerPos() const;
  VelocityController*     getControllerVel() const;
  AccelerationController* getControllerAcc() const;
  SO3Controller*          getControllerSO3() const;
  AttitudeController*     getControllerAtt() const;
  PalstanceController*    getControllerPal() const;
  ThrustController*       getControllerThr() const;

  Mixer<Eigen::Vector3d>* getMixerVelocity();
  Mixer<Eigen::Vector3d>* getMixerAcceleration();

private:
  //! @note subcontrollers
  PositionController*     controllerPos;
  VelocityController*     controllerVel;
  AccelerationController* controllerAcc;
  SO3Controller*          controllerSO3;
  AttitudeController*     controllerAtt;
  PalstanceController*    controllerPal;
  ThrustController*       controllerThr;

  IntergrationEstimator estimatorPos[3];

  VelocityMixer          mixerVelocity; //! @todo change to a no-linear mixer
  Mixer<Eigen::Vector3d> mixerAcceleration;
  Mixer<SO3>             mixerAttitude;
  Mixer<Eigen::Vector3d> mixerPalstance;
  Mixer<double>          mixerThrust;

private:
  //! @note Quadrotor model
  Eigen::Vector3d thrustAxies;
  Eigen::Vector3d viewAxies;
};

} // namespace mocka
#endif // CASCADECONTROLLER_HPP

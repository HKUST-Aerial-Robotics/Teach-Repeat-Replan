#include "cascadecontrol.hpp"

#include "accelerationcontrol.hpp"
#include "attitude_control.hpp"

#include "palstance_control.hpp"
#include "position.hpp"
#include "so3control.hpp"
#include "thrust.hpp"
#include "velocity.hpp"

#include <iostream>

using namespace mocka;

//! @note REP 103 use ENU and FLU

CascadeController::CascadeController()
{
  controllerPos = new PositionController();
  controllerVel = new VelocityController();
  controllerAcc = new AccelerationController();
  controllerSO3 = new SO3Controller();
  controllerAtt = new AttitudeController();
  controllerPal = new PalstanceController();
  controllerThr = new ThrustController();

  viewAxies   = Eigen::Vector3d(1, 0, 0);
  thrustAxies = Eigen::Vector3d(0, 0, 1);

  controllerSO3->setView(viewAxies);
  controllerSO3->setThrust(thrustAxies);
}

CascadeController::~CascadeController()
{
  delete controllerPos;
  delete controllerVel;
  delete controllerAcc;
  delete controllerSO3;
  delete controllerAtt;
  delete controllerPal;
  delete controllerThr;
}

CascadeController::Output
CascadeController::update()
{
  State currentInput = getInput();
  State currentOutput;

  uint64_t inputTime = getInputTime_ns();

  // position control
  if (std::isnan(currentInput.position.norm()))
  {
    //! @todo use intergration to fix nan input
    for (int i = 0; i < 3; ++i)
    {
      //! @todo add a parameter here, instead of 0.25
      if (!(estimatorPos[i].isInitialized() && currentInput.velocity(i) == 0 &&
            controllerVel->getFeeds()(i) < 0.25))
      {
        estimatorPos[i].reset(controllerPos->getFeeds()(i),
                              currentInput.velocity(i));
      }
      currentInput.position(i) =
        estimatorPos[i].update(currentInput.velocity(i));
    }
    controllerPos->setInput(currentInput.position, inputTime);
    controllerPos->spinOnce();
    currentOutput.position = controllerPos->getOutput();
    for (int i = 0; i < 3; ++i)
    {
      if (std::abs(currentOutput.position(i)) >
          std::abs(0.2 * controllerPos->getController(i)->getKp()))
      {
        estimatorPos[i].setInvaild();
        estimatorPos[i].reset(controllerPos->getFeeds()(i),
                              currentInput.velocity(i));
        currentOutput.position(i) = 0;
      }
    }
  }
  else
  {
    controllerPos->setInput(currentInput.position, inputTime);
    controllerPos->spinOnce();
    currentOutput.position = controllerPos->getOutput();
    estimatorPos[0].setInvaild();
    estimatorPos[1].setInvaild();
    estimatorPos[2].setInvaild();
  }
  if (std::isnan(currentOutput.position.norm()))
  {
    currentOutput.position.setZero();
  }
  // velocity control
  Eigen::Vector3d velocityInput =
    mixerVelocity.mix(currentOutput.position, currentInput.velocity);
  controllerVel->setInput(velocityInput, inputTime);
  controllerVel->spinOnce();
  currentOutput.velocity = controllerVel->getOutput();
  if (std::isnan(currentOutput.velocity.norm()))
    currentOutput.velocity.setZero();
  // acceleration control
  //! @note for reduce stable error

  Eigen::Vector3d accelerationInput =
    mixerAcceleration.mix(currentOutput.velocity, currentInput.acceleration);
  controllerAcc->setInput(accelerationInput, inputTime);
  controllerAcc->spinOnce();
  //! @note FLU frame
  currentOutput.acceleration = controllerAcc->getOutput() + accelerationInput;
  if (std::isnan(currentOutput.acceleration.norm()))
  {
    currentOutput.acceleration.setZero();
  }
  // so3 control
  SO3Controller::Input se3;
  se3.acceleration = currentOutput.acceleration;
  se3.viewPoint    = currentInput.viewPoint;
  controllerSO3->setInput(se3, inputTime);
  controllerSO3->spinOnce();
  SO3Controller::Output so3 = controllerSO3->getOutput();
  currentOutput.viewPoint =
    Eigen::Vector3d(1, 0, 0).transpose() * so3.quaternion.toRotationMatrix();
  currentOutput.quaternion = so3.quaternion;

  // attitude control
  controllerAtt->setInput(so3.quaternion, inputTime);
  controllerAtt->spinOnce();

  // palstance control
  controllerPal->setInput(controllerAtt->getOutput() + currentInput.palstance,
                          inputTime);
  controllerPal->spinOnce();
  currentOutput.palstance = controllerPal->getOutput();

  // thrust control
  //! @todo add trust correction
  controllerThr->setInput(so3.thrust, inputTime);
  controllerThr->spinOnce();
  currentOutput.thrust = controllerThr->getOutput();

  return currentOutput;
}

void
CascadeController::updateInput()
{
  //! @note finish on the output update
}

void
CascadeController::updateFeeds()
{
}

void
CascadeController::setFeeds(const Controller::Feeds& value)
{
  setFeeds(value, getCurrent());
}

void
CascadeController::setFeeds(const Controller::Feeds& value,
                            const uint64_t&          time)
{
  feed         = value;
  feedsTime_ns = time;

  controllerPos->setFeeds(value.position, time);
  controllerVel->setFeeds(value.velocity, time);
  controllerAcc->setFeeds(value.acceleration, time);

  SO3 f;
  f.quaternion = value.quaternion;
  f.thrust     = value.thrust;

  controllerSO3->setFeeds(f, time);
  controllerAtt->setFeeds(value.quaternion, time);
  controllerPal->setFeeds(value.palstance, time);
  controllerThr->setFeeds(value.thrust, time);
}

void
CascadeController::setFeedPosition(const Eigen::Vector3d& value)
{
  controllerPos->setFeeds(value);
}

void
CascadeController::setFeedVelocity(const Eigen::Vector3d& value)
{
  controllerVel->setFeeds(value);
}

void
CascadeController::setFeedAcceleration(const Eigen::Vector3d& value)
{
  controllerAcc->setFeeds(value);
}

void
CascadeController::setFeedViewPoint(const Eigen::Vector3d& value)
{
  SE3 se3;
  se3.acceleration = controllerAcc->getFeeds();
  se3.viewPoint    = value;
  controllerSO3->setFeeds(se3.toSO3(this->viewAxies, this->thrustAxies));
}

void
CascadeController::setFeedQuaternion(const Eigen::Quaterniond& value)
{
  controllerAtt->setFeeds(value);
}

void
CascadeController::setFeedPalstance(const Eigen::Vector3d& value)
{
  controllerPal->setFeeds(value);
}

void
CascadeController::setFeedThrust(const double& value)
{
  controllerThr->setFeeds(value);
}

PositionController*
CascadeController::getControllerPos() const
{
  return controllerPos;
}

VelocityController*
CascadeController::getControllerVel() const
{
  return controllerVel;
}

AccelerationController*
CascadeController::getControllerAcc() const
{
  return controllerAcc;
}

SO3Controller*
CascadeController::getControllerSO3() const
{
  return controllerSO3;
}

AttitudeController*
CascadeController::getControllerAtt() const
{
  return controllerAtt;
}

PalstanceController*
CascadeController::getControllerPal() const
{
  return controllerPal;
}

ThrustController*
CascadeController::getControllerThr() const
{
  return controllerThr;
}

Mixer<Eigen::Vector3d>*
CascadeController::getMixerVelocity()
{
  return &mixerVelocity;
}

Mixer<Eigen::Vector3d>*
CascadeController::getMixerAcceleration()
{
  return &mixerAcceleration;
}

Eigen::Vector3d
CascadeController::getThrustAxies() const
{
  return thrustAxies;
}

void
CascadeController::setThrustAxies(const Eigen::Vector3d& value)
{
  thrustAxies = value;
}

Eigen::Vector3d
CascadeController::getViewAxies() const
{
  return viewAxies;
}

void
CascadeController::setViewAxies(const Eigen::Vector3d& value)
{
  viewAxies = value;
}

namespace mocka
{

template <>
void
Controller<State, State, State>::setInput(const Input& value)
{
  in           = value;
  inputTime_ns = getCurrent();
}

template <>
void
Controller<State, State, State>::setInput(const Input& value,
                                          const uint64_t& time)
{
  in           = value;
  inputTime_ns = time;
}

template <>
void
Controller<State, State, State>::setFeeds(const Feeds& value)
{
  feed         = value;
  feedsTime_ns = getCurrent();
}

template <>
void
Controller<State, State, State>::setFeeds(const Feeds& value,
                                          const uint64_t& time)
{
  feed         = value;
  feedsTime_ns = time;
}

} // namespace mocka

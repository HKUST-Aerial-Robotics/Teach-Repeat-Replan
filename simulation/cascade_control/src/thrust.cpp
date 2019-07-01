#include "thrust.hpp"

using namespace mocka;

ThrustController::ThrustController()
  : stateObserver(new ThrustController::Observer())
{
}

ThrustController::~ThrustController()
{
  delete stateObserver;
}

ThrustController::Observer::Observer()
{
}

ThrustController::Observer::~Observer()
{
}

ThrustController::ObserverBase::Output
ThrustController::Observer::update(ObserverBase::Input input)
{
  //! @todo implement
  return input;
}

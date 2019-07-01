#ifndef THRUST_HPP
#define THRUST_HPP

#include "pid.hpp"
#include "pretreatmentor.hpp"

namespace mocka
{

class ThrustController : public PID
{
  typedef Pretreatmentor<PID::Feeds, double> ObserverBase;
  class Observer : public ObserverBase
  {
  public:
    Observer();
    virtual ~Observer();

  public:
    virtual Pretreatmentor::Output update(Pretreatmentor::Input input);
  };

public:
  ThrustController();
  ~ThrustController();

private:
  ObserverBase* stateObserver;
};

} // namespace mocka
#endif // THRUST_HPP

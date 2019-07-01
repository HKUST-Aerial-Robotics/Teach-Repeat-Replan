#ifndef EVENTSPINNER_HPP
#define EVENTSPINNER_HPP

#include "statemachine.hpp"
#include <memory>
#include <vector>

namespace mocka
{

class EventBase;
class EventSpinner
{
public:
  EventSpinner();

public:
  void push(EventBase* e);
  void spin();

  SafeFIFO<int>* getEvent();

private:
  std::vector<std::shared_ptr<EventBase> > pool;

  SafeFIFO<int> event;
};

class EventBase
{
public:
  EventBase();
  virtual ~EventBase();

public:
  virtual int spin() = 0;
}; // EventSpinnerBase

template <class... Ts>
class Event : public EventBase
{
public:
  /*! @note the reason why we don't use std::function here, is that we want to
   * get the resource reference and modify it.
   * */
  typedef int (*Function)(Event<Ts...>*);

public:
  Event(Function ptr, Ts... ts);

public:
  /*! @brief spinner for process an event, to check if it happened or not
   *  @return the event code of the statemachine
   *  @sa OperationalLogic::RuntimeEvent::id
   * */
  virtual int spin();

public:
  // getters
  aux::Tuple<Ts...>& getResource() const;

  template <int k>
  auto&         get()
  {
    return aux::__impl::get<k>(&resource);
  }

private:
  aux::Tuple<Ts...> resource;
  Function          p;
};

// template EventSpinner function definition below

template <class... Ts>
Event<Ts...>::Event(Function ptr, Ts... ts)
  : resource(ts...)
  , p(ptr)
{
}

template <class... Ts>
int
Event<Ts...>::spin()
{
  return p(this);
}

template <class... Ts>
aux::Tuple<Ts...>&
Event<Ts...>::getResource() const
{
  return resource;
}

} // namespace  mocka
#endif // EVENTSPINNER_HPP

#include "eventspinner.hpp"

using namespace mocka;

EventBase::EventBase()
{
}

EventBase::~EventBase()

{
}

EventSpinner::EventSpinner()
{
  pool.clear();
}

void
EventSpinner::push(EventBase* e)
{
  //  if (pool.max_size() == pool.size())
  //    pool.reserve(pool.max_size() + 1);
  pool.push_back(std::shared_ptr<EventBase>(e));
}

void
EventSpinner::spin()
{
  for (int i = 0; i < pool.size(); ++i)
  {
    int ans = pool[i]->spin();
    if (ans)
      event.push(ans);
  }
  if (event.length() == 0)
    event.push(0);
}

SafeFIFO<int>*
EventSpinner::getEvent()
{
  return &event;
}

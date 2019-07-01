#include "mission.hpp"

#include <sstream>

using namespace mocka;

MissionManager::MissionManager()
{
}

void
MissionManager::push(MissionBase* m)
{
  SafeFIFO<std::shared_ptr<MissionBase> >::push(Item(m));
}

void
MissionManager::poll()
{
  if (externalMission != nullptr)
    externalMission->run();
  if (current == nullptr)
  {
    if (length())
      current = pop();
    else
      return; // is empty
  }
  RESULT s = current->poll();
  switch (s)
  {
    case MISSION_RESULT_FINISHED:
    case MISSION_RESULT_RESOURCE_LOST:
    case MISSION_RESULT_COMMON_ERROR:
      current = pop();
      break;
    case MISSION_RESULT_WIP:
      return; //! @attention dirct return here
    default:
    {
      std::stringstream msg;
      msg << "unknown Mission return code" << s;
      throw std::runtime_error(msg.str().c_str());
    }
    break;
  }
}

void
MissionManager::pend()
{
  if (externalMission != nullptr)
    externalMission->pend();
}

bool
MissionManager::isEmpty()
{
  return length() == 0 && current == nullptr;
}

void
MissionManager::setExternalMission(
  const std::shared_ptr<ExternalMission>& value)
{
  externalMission = value;
}

MissionBase::~MissionBase()
{
}

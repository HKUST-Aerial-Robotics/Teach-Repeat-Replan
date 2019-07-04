#ifndef MISSION_HPP
#define MISSION_HPP

#include <memory>

#include "external_mission.hpp"
#include "mpaux.hpp"
#include "safefifo.hpp"

namespace mocka
{

typedef enum RESULT {
  MISSION_RESULT_FINISHED,
  MISSION_RESULT_WIP,
  MISSION_RESULT_RESOURCE_LOST,
  MISSION_RESULT_COMMON_ERROR
} RESULT;

class MissionBase; //! @note for buildin missions

//! @brief MissionManager
//!
//! for sesequencial process a mission series, seems like a traditional turing
//! machine
class MissionManager : public SafeFIFO<std::shared_ptr<MissionBase> >
{
public:
  typedef std::shared_ptr<MissionBase> Item;

public:
  MissionManager();

public:
  void push(MissionBase* m); // cover the FIFO push, easy enterance
  void poll();
  void pend();
  bool isEmpty();

  void setExternalMission(const std::shared_ptr<ExternalMission>& value);

private:
  std::shared_ptr<MissionBase>     current;
  std::shared_ptr<ExternalMission> externalMission;
};

class MissionBase
{
public:
  virtual ~MissionBase();

  virtual RESULT poll() = 0;
}; // MissionBase

template <class... Ts>
class Mission : public MissionBase
{
public:
  /*! @note the reason why we don't use std::function here, is that we want to
   * get the resource reference and modify it.
   * */
  typedef RESULT (*Function)(Mission<Ts...>*);

public:
  /*! @brief bind with mpaux.hpp mocka::aux::Tuple<class ...Ts> is the
   * recommanded usage
   * */
  Mission(Function ptr, Ts... ts);

public:
  /*! @brief poller of mission
   *  @return poll result
   *  @sa mocka::RESULT
   * */
  virtual RESULT poll();

public:
  // gettsers
  aux::Tuple<Ts...>& getResource();

  template <int k>
  auto&         get()
  {
    return aux::__impl::get<k>(&resource);
  }

private:
  aux::Tuple<Ts...> resource;
  Function          p;
}; // Mission

// template Mission function definition below
template <class... Ts>
Mission<Ts...>::Mission(Mission::Function ptr, Ts... ts)
  : resource(ts...)
  , p(ptr)
{
}

template <class... Ts>
RESULT
Mission<Ts...>::poll()
{
  return p(this);
}

template <class... Ts>
aux::Tuple<Ts...>&
Mission<Ts...>::getResource()
{
  return resource;
}

} // namespace mocka

#endif // MISSION_HPP

#ifndef MEMORY_DEFAULT_HPP
#define MEMORY_DEFAULT_HPP

#include "dji_memory.hpp"
#include "dji_thread_manager.hpp"

namespace DJI
{
namespace OSDK
{

class MutexPrivate;

class MutexDefault : public Mutex
{
public:
  MutexDefault();
  ~MutexDefault();

public:
  void lock();
  void unlock();

private:
  MutexPrivate* instance;
}; // class MutexDefault

} // namespace OSDK
} // namespace DJI

#endif // MEMORY_DEFAULT_HPP

#include "dji_memory_default.hpp"

#include "dji_log.hpp"

#ifdef qt
#elif defined(stm32)
#elif defined(__linux__)
#include <pthread.h>

namespace DJI
{
namespace OSDK
{
class MutexPrivate
{
public:
  MutexPrivate()
  {
    _mutex = PTHREAD_MUTEX_INITIALIZER;
  }
  ~MutexPrivate()
  {
    pthread_mutex_destroy(&_mutex);
  }

public:
  inline void lock()
  {
    pthread_mutex_lock(&_mutex);
  }
  inline void unlock()
  {
    pthread_mutex_unlock(&_mutex);
  }
  pthread_mutex_t _mutex;
}; // class DJI::OSDK::MutexPrivate

} // namespace OSDK
} // namespace DJI
#else // default non-functional driver

namespace DJI
{
namespace OSDK
{

class MutexPrivate
{
public:
  MutexPrivate()
  {
    //! @note cannot use DLOG here
  }
  ~MutexPrivate()
  {
  }

public:
  inline void lock()
  {
  }
  inline void unlock()
  {
  }
}; // class MutexPrivate

} // namespace OSDK
} // namespace DJI

#endif // ugly cross platform support

using namespace DJI::OSDK;

MutexDefault::MutexDefault()
  : instance(new MutexPrivate())
{
  //! @note cannot use DLOG here
}

MutexDefault::~MutexDefault()
{
}

void
MutexDefault::lock()
{
  instance->lock();
}

void
MutexDefault::unlock()
{
  instance->unlock();
}

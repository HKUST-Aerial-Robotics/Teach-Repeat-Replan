/** @file dji_log.hpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Logging mechanism for printing status and error messages to the screen.
 *
 *  @copyright 2016-17 DJI. All right reserved.
 *
 */

#ifndef LOG_H
#define LOG_H

#include "dji_memory.hpp"
#include "dji_singleton.hpp"
#include "dji_thread_manager.hpp"

#ifdef WIN32
#define __func__ __FUNCTION__
#endif // WIN32

#define DLOG(_title_)                                                          \
  DJI::OSDK::Log::instance()                                                   \
    .title((_title_), #_title_, __func__, __LINE__)                            \
    .print

#define STATUS 1
#define ERROR 1
#define DEBUG 0

#define DSTATUS DLOG(STATUS)
#define DERROR DLOG(ERROR)
#define DDEBUG DLOG(DEBUG)

namespace DJI
{
namespace OSDK
{

//! @todo text stream and string class

class Log : public Singleton<Log>
{
public:
  Log(Mutex* m = 0);
  ~Log();

  //! @note if title level is 0, this log would not be print at all
  //! this feature is used for dynamical/statical optional log output
  Log& title(int level, const char* prefix, const char* func, int line);

  Log& print();

  //! @note basic io function
  virtual Log& print(const char* fmt, ...);

  Log& operator<<(bool val);
  Log& operator<<(short val);
  Log& operator<<(uint16_t val);
  Log& operator<<(int val);
  Log& operator<<(uint32_t val);
  Log& operator<<(long val);
  Log& operator<<(unsigned long val);
  Log& operator<<(long long val);
  Log& operator<<(unsigned long long val);
  Log& operator<<(float val);
  Log& operator<<(double val);
  Log& operator<<(long double val);
  Log& operator<<(void* val);
  Log& operator<<(char c);
  Log& operator<<(uint8_t c);
  Log& operator<<(int8_t c);
  Log& operator<<(const char* str);

private:
  Mutex* mutex;
  bool   vaild;

  //! @todo implement
  typedef enum NUMBER_STYLE {
    STYLE_DEC,
    STYLE_HEX,
    STYLE_BIN,
    STYLE_OCT
  } NUMBER_STYLE;

  static const bool release = false;
};

} // namespace OSDK
} // namespace DJI

#endif // LOG_H

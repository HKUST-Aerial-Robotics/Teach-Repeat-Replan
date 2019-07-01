/** @file dji_log.cpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Logging mechanism for printing status and error messages to the screen.
 *
 *  @copyright 2016-17 DJI. All right reserved.
 *
 */

#include "dji_log.hpp"
#include "dji_memory_default.hpp"

#include <stdarg.h>
#include <stdio.h>

using namespace DJI::OSDK;

Log::Log(Mutex* m)
{
  if (m)
  {
    mutex = m;
  }
  else
  {
    mutex = new MutexDefault();
  }
}

Log::~Log()
{
}

Log&
Log::title(int level, const char* prefix, const char* func, int line)
{
  if (level)
  {
    vaild = true;

    const char str[] = "\n%s/%d @ %s, L%d: ";
    print(str, prefix, level, func, line);
  }
  else
  {
    vaild = false;
  }
  return *this;
}

Log&
Log::print()
{
  return *this;
}

Log&
Log::print(const char* fmt, ...)
{
  if ((!release) && vaild)
  {
    va_list args;
    va_start(args, fmt);
    mutex->lock();
    vprintf(fmt, args);
    mutex->unlock();
    va_end(args);
  }
  return *this;
}

Log&
Log::operator<<(bool val)
{
  if (val)
  {
    print("True");
  }
  else
  {
    print("False");
  }
  return *this;
}

Log&
Log::operator<<(short val)
{
  //! @todo NUMBER_STYLE
  print("%d", val);
  return *this;
}

Log&
Log::operator<<(uint16_t val)
{
  //! @todo NUMBER_STYLE
  print("%u", val);
  return *this;
}

Log&
Log::operator<<(int val)
{
  //! @todo NUMBER_STYLE
  print("%d", val);
  return *this;
}

Log&
Log::operator<<(uint32_t val)
{
  //! @todo NUMBER_STYLE
  print("%u", val);
  return *this;
}

Log&
Log::operator<<(long val)
{
  //! @todo NUMBER_STYLE
  print("%ld", val);
  return *this;
}

Log&
Log::operator<<(unsigned long val)
{
  //! @todo NUMBER_STYLE
  print("%lu", val);
  return *this;
}

Log&
Log::operator<<(long long val)
{
  //! @todo NUMBER_STYLE
  print("%lld", val);
  return *this;
}

Log&
Log::operator<<(unsigned long long val)
{
  //! @todo NUMBER_STYLE
  print("%llu", val);
  return *this;
}

Log&
Log::operator<<(float val)
{
  //! @todo NUMBER_STYLE
  print("%f", val);
  return *this;
}

Log&
Log::operator<<(double val)
{
  //! @todo NUMBER_STYLE
  print("%lf", val);
  return *this;
}

Log&
Log::operator<<(long double val)
{
  //! @todo NUMBER_STYLE
  print("%Lf", val);
  return *this;
}

Log&
Log::operator<<(void* val)
{
  //! @todo NUMBER_STYLE
  print("ptr:0x%X", val);
  return *this;
}

Log&
Log::operator<<(const char* str)
{
  print("%s", str);
  return *this;
}

Log&
Log::operator<<(char c)
{
  //! @todo NUMBER_STYLE
  print("%c", c);
  return *this;
}

Log&
Log::operator<<(int8_t c)
{
  //! @todo NUMBER_STYLE
  print("%c", c);
  return *this;
}

Log&
Log::operator<<(uint8_t c)
{
  //! @todo NUMBER_STYLE
  print("0x%.2X", c);
  return *this;
}

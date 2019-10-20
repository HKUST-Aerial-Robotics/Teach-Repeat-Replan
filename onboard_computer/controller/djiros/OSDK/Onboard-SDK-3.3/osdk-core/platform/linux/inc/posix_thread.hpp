/*! @file posix_thread.hpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK Linux/*NIX platforms
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#ifndef LINUXTHREAD_H
#define LINUXTHREAD_H

#include "dji_thread_manager.hpp"
#include "dji_vehicle.hpp"

#include <pthread.h>

namespace DJI
{
namespace OSDK
{

/*! @brief POSIX-compatible threading implementation for *NIX systems
 *
 * @details Threading is handled by the Vehicle, you do not need to
 * manage threads for nominal usage.
 *
 */
class PosixThread : public Thread
{
public:
  PosixThread();
  PosixThread(Vehicle* vehicle, int type);
  ~PosixThread()
  {
  }

  bool createThread();
  int  stopThread();

private:
  pthread_t      threadID;
  pthread_attr_t attr;

  static void* send_call(void* param);
  static void* read_call(void* param);
  static void* callback_call(void* param);
};

} // namespace DJI
} // namespace onboardSDK

#endif // LINUXTHREAD_H

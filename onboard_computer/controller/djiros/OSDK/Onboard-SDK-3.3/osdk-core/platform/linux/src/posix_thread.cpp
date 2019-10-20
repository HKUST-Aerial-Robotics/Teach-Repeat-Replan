/*! @file posix_thread.cpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Pthread-based threading for DJI Onboard SDK on linux platforms
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#include "posix_thread.hpp"
#include <string>

using namespace DJI::OSDK;

PosixThread::PosixThread()
{
  vehicle = 0;
  type    = 0;
}

PosixThread::PosixThread(Vehicle* vehicle, int Type)
{
  this->vehicle = vehicle;
  this->type    = Type;
  vehicle->setStopCond(false);
}

bool
PosixThread::createThread()
{
  int         ret = -1;
  std::string infoStr;

  /* Initialize and set thread detached attribute */
  pthread_attr_init(&attr);
  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

  if (1 == type)
  {
    ret     = pthread_create(&threadID, NULL, send_call, (void*)vehicle);
    infoStr = "sendPoll";
  }
  else if (2 == type)
  {
    ret     = pthread_create(&threadID, NULL, read_call, vehicle);
    infoStr = "readPoll";
  }

  else if (3 == type)
  {
    ret     = pthread_create(&threadID, NULL, callback_call, (void*)vehicle);
    infoStr = "callback";
  }
  else
  {
    infoStr = "error type number";
  }

  if (0 != ret)
  {
    DERROR("fail to create thread for %s!\n", infoStr.c_str());
    return false;
  }

  ret = pthread_setname_np(threadID, infoStr.c_str());
  if (0 != ret)
  {
    DERROR("fail to set thread name for %s!\n", infoStr.c_str());
    return false;
  }
  return true;
}

int
PosixThread::stopThread()
{
  int   ret = -1;
  void* status;
  vehicle->setStopCond(true);

  /* Free attribute and wait for the other threads */
  if (int i = pthread_attr_destroy(&attr))
  {
    DERROR("fail to destroy thread %d\n", i);
  }
  else
  {
    DDEBUG("success to distory thread\n");
  }
  ret = pthread_join(threadID, &status);

  DDEBUG("Main: completed join with thread code: %d\n", ret);
  if (ret)
  {
    // Return error code
    return ret;
  }

  return 0;
}

void*
PosixThread::send_call(void* param)
{
  Vehicle* vehiclePtr = (Vehicle*)param;
  while (true)
  {
    vehiclePtr->protocolLayer->sendPoll();
    usleep(10); //! @note CPU optimization, reduce the CPU usage a lot
  }
}

void*
PosixThread::read_call(void* param)
{

  RecvContainer recvContainer;
  Vehicle*      vehiclePtr = (Vehicle*)param;
  while (!(vehiclePtr->getStopCond()))
  {
    // receive() implemented on the OpenProtocol side
    recvContainer = vehiclePtr->protocolLayer->receive();
    vehiclePtr->processReceivedData(recvContainer);
    usleep(10); //! @note CPU optimization, reduce the CPU usage a lot
  }
  DDEBUG("Quit read function\n");
}

void*
PosixThread::callback_call(void* param)
{
  Vehicle* vehiclePtr = (Vehicle*)param;
  while (!(vehiclePtr->getStopCond()))
  {
    vehiclePtr->callbackPoll();
    usleep(10); //! @note CPU optimization, reduce the CPU usage a lot
  }
  DDEBUG("Quit callback function\n");
}

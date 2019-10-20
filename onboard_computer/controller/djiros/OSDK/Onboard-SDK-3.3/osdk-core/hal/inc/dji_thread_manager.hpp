/*! @file dji_thread_manager.hpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Data protection and thread management abstract classes.
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 */

#ifndef ONBOARDSDK_THREADMANAGER_H
#define ONBOARDSDK_THREADMANAGER_H

namespace DJI
{
namespace OSDK
{

//! Forward Declaration of vehicle
class Vehicle;

//! @todo start use this class
class Mutex
{
public:
  Mutex();
  virtual ~Mutex();

public:
  virtual void lock()   = 0;
  virtual void unlock() = 0;
}; // class Mutex

class ThreadAbstract
{
public:
  ThreadAbstract();
  virtual ~ThreadAbstract();

  //! Mutex operations
public:
  virtual void lockMemory() = 0;
  virtual void freeMemory() = 0;

  virtual void lockMSG() = 0;
  virtual void freeMSG() = 0;

  virtual void lockACK() = 0;
  virtual void freeACK() = 0;

  virtual void lockProtocolHeader();
  virtual void freeProtocolHeader();

  virtual void lockNonBlockCBAck();
  virtual void freeNonBlockCBAck();

  virtual void notifyNonBlockCBAckRecv();
  virtual void nonBlockWait();

  virtual void lockStopCond();
  virtual void freeStopCond();

  virtual void lockFrame();
  virtual void freeFrame();

  //! Thread comm/sync
public:
  virtual void notify()          = 0;
  virtual void wait(int timeout) = 0;

public:
  virtual void init() = 0;
};

class Thread
{
public:
  Thread();
  virtual ~Thread();

  virtual bool createThread() = 0;
  virtual int  stopThread()   = 0;

protected:
  Vehicle* vehicle;
  int      type;
};

} // namespace DJI
} // namespace OSDK

#endif // ONBOARDSDK_THREADMANAGER_H

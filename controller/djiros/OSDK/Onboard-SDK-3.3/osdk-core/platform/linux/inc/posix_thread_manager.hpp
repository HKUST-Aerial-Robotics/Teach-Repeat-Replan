/*! @file posix_thread_manager.hpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Thread safety and data protection for DJI Onboard SDK on linux platforms
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#ifndef POSIXTHREADMANAGER_H
#define POSIXTHREADMANAGER_H

#include "dji_thread_manager.hpp"
#include "pthread.h"

namespace DJI
{
namespace OSDK
{

/*! @brief POSIX-COmpatible Data Protection and Condition Variables for *NIX
 * platforms
 *
 */
class PosixThreadManager : public ThreadAbstract
{
public:
  PosixThreadManager()
  {
  }
  ~PosixThreadManager();

  void init();

  //! Implementing virtual functions from ThreadManager
public:
  void lockMemory();
  void freeMemory();

  void lockMSG();
  void freeMSG();

  void lockACK();
  void freeACK();

  void lockProtocolHeader();
  void freeProtocolHeader();

  void lockNonBlockCBAck();
  void freeNonBlockCBAck();

  void lockStopCond();
  void freeStopCond();

  void lockFrame();
  void freeFrame();

  void notify();
  void notifyNonBlockCBAckRecv();
  void wait(int timeoutInSeconds);
  void nonBlockWait();

private:
  pthread_mutex_t m_memLock;
  pthread_mutex_t m_msgLock;
  pthread_mutex_t m_ackLock;
  pthread_cond_t  m_ackRecvCv;

  pthread_mutex_t m_headerLock;
  pthread_mutex_t m_nbAckLock;
  pthread_cond_t  m_nbAckRecv;

  //! Thread protection for setting stop condition for threads
  pthread_mutex_t m_stopCondLock;

  //! Thread protection for last received frame storage
  pthread_mutex_t m_frameLock;
};

} // namespace OSDK
} // namespace DJI

#endif // POSIXTHREADMANAGER_H

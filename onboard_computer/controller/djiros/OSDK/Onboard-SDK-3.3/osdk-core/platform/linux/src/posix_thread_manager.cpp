/*! @file posix_thread_manager.cpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Thread safety and data protection for DJI Onboard SDK on linux platforms
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#include "posix_thread_manager.hpp"

using namespace DJI::OSDK;

PosixThreadManager::~PosixThreadManager()
{
  pthread_mutex_destroy(&m_memLock);
  pthread_mutex_destroy(&m_msgLock);
  pthread_mutex_destroy(&m_ackLock);

  pthread_mutex_destroy(&m_nbAckLock);
  pthread_mutex_destroy(&m_headerLock);

  pthread_cond_destroy(&m_nbAckRecv);
  pthread_cond_destroy(&m_ackRecvCv);

  pthread_mutex_destroy(&m_frameLock);
  pthread_mutex_destroy(&m_stopCondLock);
}

void
PosixThreadManager::init()
{
  m_memLock = PTHREAD_MUTEX_INITIALIZER;
  m_msgLock = PTHREAD_MUTEX_INITIALIZER;
  m_ackLock = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_init(&m_ackRecvCv, NULL);

  /*! These mutexes are used for the non blocking callback ACK mechanism */
  m_nbAckLock  = PTHREAD_MUTEX_INITIALIZER;
  m_headerLock = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_init(&m_nbAckRecv, NULL);

  /*! Mutex initializations
   * These are newly added mutexes
   */
  m_frameLock    = PTHREAD_MUTEX_INITIALIZER;
  m_stopCondLock = PTHREAD_MUTEX_INITIALIZER;
}

void
PosixThreadManager::lockMemory()
{
  pthread_mutex_lock(&m_memLock);
}

void
PosixThreadManager::freeMemory()
{
  pthread_mutex_unlock(&m_memLock);
}

void
PosixThreadManager::lockMSG()
{
  pthread_mutex_lock(&m_msgLock);
}

void
PosixThreadManager::freeMSG()
{
  pthread_mutex_unlock(&m_msgLock);
}

void
PosixThreadManager::lockACK()
{
  pthread_mutex_lock(&m_ackLock);
}

void
PosixThreadManager::freeACK()
{
  pthread_mutex_unlock(&m_ackLock);
}

void
PosixThreadManager::lockProtocolHeader()
{
  pthread_mutex_lock(&m_headerLock);
}

void
PosixThreadManager::freeProtocolHeader()
{
  pthread_mutex_unlock(&m_headerLock);
}

void
PosixThreadManager::lockNonBlockCBAck()
{
  pthread_mutex_lock(&m_nbAckLock);
}

void
PosixThreadManager::freeNonBlockCBAck()
{
  pthread_mutex_unlock(&m_nbAckLock);
}

void
PosixThreadManager::lockStopCond()
{
  pthread_mutex_lock(&m_stopCondLock);
}

void
PosixThreadManager::freeStopCond()
{
  pthread_mutex_unlock(&m_stopCondLock);
}

void
PosixThreadManager::lockFrame()
{
  pthread_mutex_lock(&m_frameLock);
}

void
PosixThreadManager::freeFrame()
{
  pthread_mutex_unlock(&m_frameLock);
}

void
PosixThreadManager::notify()
{
  pthread_cond_signal(&m_ackRecvCv);
}

void
PosixThreadManager::notifyNonBlockCBAckRecv()
{
  pthread_cond_signal(&m_nbAckRecv);
}

void
PosixThreadManager::nonBlockWait()
{

  pthread_cond_wait(&m_nbAckRecv, &m_nbAckLock);
}

void
PosixThreadManager::wait(int timeoutInSeconds)
{
  struct timespec curTime, absTimeout;
  // Use clock_gettime instead of getttimeofday for compatibility with POSIX
  // APIs
  clock_gettime(CLOCK_REALTIME, &curTime);
  // absTimeout = curTime;
  absTimeout.tv_sec  = curTime.tv_sec + timeoutInSeconds;
  absTimeout.tv_nsec = curTime.tv_nsec;
  pthread_cond_timedwait(&m_ackRecvCv, &m_ackLock, &absTimeout);
}
/*! @file STM32F4DataGuard.h
 *  @version 3.3
 *  @date Apr 12th, 2017
 *
 *  @brief
 *  Data protection for thread access for DJI Onboard SDK Linux/*NIX platforms
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#ifndef STM32F4DATAGUARD_H
#define STM32F4DATAGUARD_H

#include "dji_thread_manager.hpp"

namespace DJI
{
namespace OSDK
{

/*! @brief Data Protection
 *
 */
class STM32F4DataGuard : public DJI::OSDK::ThreadAbstract
{
public:
  STM32F4DataGuard();
  ~STM32F4DataGuard();

  void init();

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
};

} // namespace OSDK
} // namespace DJI

#endif // STM32F4DATAGUARD_H

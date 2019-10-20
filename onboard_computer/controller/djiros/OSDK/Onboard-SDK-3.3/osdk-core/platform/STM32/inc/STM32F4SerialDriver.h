/*! @file STM32F4SerialDriver.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Implementation of HardDriver for the STM32F4Discovery board.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "dji_hard_driver.hpp"

class STM32F4 : public DJI::OSDK::HardDriver
{
public:
  virtual void init()
  {
  }
  virtual size_t send(const uint8_t* buf, size_t len);
  virtual DJI::OSDK::time_ms getTimeStamp();
  virtual bool         getDeviceStatus()
  {
    return true;
  }
  virtual size_t readall(uint8_t* buf, size_t maxlen)
  {
    return 8;
  }
  static void delay_nms(uint16_t time);
};

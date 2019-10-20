/*! @file STM32F4SerialDriver.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Implementation of HardDriver for the STM32F4Discovery board.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#include "stm32f4xx.h"
#include <STM32F4SerialDriver.h>
#include <time.h>

extern uint32_t tick;

void
STM32F4::delay_nms(uint16_t time)
{
  u32 i = 0;
  while (time--)
  {
    i = 30000;
    while (i--)
      ;
  }
}

size_t
STM32F4::send(const uint8_t* buf, size_t len)
{
  char* p = (char*)buf;

  if (NULL == buf)
  {
    return 0;
  }

  while (len--)
  {
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
      ;
    USART_SendData(USART3, *p++);
  }
  return 1;
}

DJI::OSDK::time_ms
STM32F4::getTimeStamp()
{
  return tick;
}


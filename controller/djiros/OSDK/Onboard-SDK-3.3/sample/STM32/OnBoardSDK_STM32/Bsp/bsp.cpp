/*! @file bsp.cpp
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Helper functions for board STM32F4Discovery
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */
 
#include "stm32f4xx.h"
#include "bsp.h"
#include "main.h"

void
BSPinit()
{
  UsartConfig();
  SystickConfig();
  Timer1Config();
  Timer2Config();
}

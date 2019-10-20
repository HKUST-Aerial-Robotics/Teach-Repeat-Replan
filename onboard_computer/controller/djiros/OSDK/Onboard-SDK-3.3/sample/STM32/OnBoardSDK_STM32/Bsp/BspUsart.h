/*! @file BspUsart.h
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  Usart helper functions and ISR for board STM32F4Discovery
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef _BSPUSART_H
#define _BSPUSART_H
#include "dji_vehicle.hpp"
#include "stdio.h"
void                      USART2_Config(void);
void                      USART3_Config(void);
void                      USARTxNVIC_Config(void);
void                      UsartConfig(void);
void                      NVIC_Config(void);
void                      Rx_buff_Handler();
DJI::OSDK::ACK::ErrorCode waitForACK();
#endif //_USART_H

/*! @file Receive.h
 *
 *  @version 3.1.8
 *  @date Aug 05 2016
 *
 *  @brief
 *  This function parses Rx buffer and execute commands sent from computer.
 *
 *  Copyright 2016 DJI. All right reserved.
 *
 * */

#ifndef RECEIVE_H
#define RECEIVE_H
#include "main.h"

#define MAX_RECEIVE 32

class TerminalCommand
{
public:
  uint32_t cmdReadyFlag;       // Rx_Handle_Flag
  uint8_t  cmdIn[MAX_RECEIVE]; // Rx_buff
  int32_t  rxIndex;            // Rx_adr
  int32_t  rxLength;           // Rx_length

  void terminalCommandHandler(Vehicle* vehicle);
  TerminalCommand()
    : cmdReadyFlag(0)
    , rxIndex(0)
    , rxLength(0)
  {
    ;
  }
};

#endif // RECEIVE_H

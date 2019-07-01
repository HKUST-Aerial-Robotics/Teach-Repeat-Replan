/*! @file Receive.cpp
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

#include "stm32f4xx.h"
#include "Receive.h"
#include "main.h"

using namespace DJI::OSDK;

/*
 * @brief Helper function to assemble two bytes into a float number
 */
static float32_t
hex2Float(uint8_t HighByte, uint8_t LowByte)
{
  float32_t high = (float32_t)(HighByte & 0x7f);
  float32_t low  = (float32_t)LowByte;
  if (HighByte & 0x80) // MSB is 1 means a negative number
  {
    return -(high * 256.0f + low) / 100.0f;
  }
  else
  {
    return (high * 256.0f + low) / 100.0f;
  }
}

TerminalCommand myTerminal;

void
TerminalCommand::terminalCommandHandler(Vehicle* vehicle)
{
  if (cmdReadyFlag == 1)
  {
    cmdReadyFlag = 0;
  }
  else
  { // No full command has been received yet.
    return;
  }

  if ((cmdIn[0] != 0xFA) || (cmdIn[1] != 0xFB))
  { // Command header doesn't match
    return;
  }

  switch (cmdIn[2])
  {
    case 0x00:
      vehicle->getDroneVersion();
      break;

    case 0x01:
      userActivate();
      break;

    /*  case 0x02:
        api->setControl(cmdIn[3]);
        break;

      case 0x03:
        flight->setArm(cmdIn[3]);
        break;

      case 0x04:
        if (cmdIn[3] == 0x01)
        {
          flightData.flag = cmdIn[4];
          flightData.x = hex2Float(cmdIn[5], cmdIn[6]);
          flightData.y = hex2Float(cmdIn[7], cmdIn[8]);
          flightData.z = hex2Float(cmdIn[9], cmdIn[10]);
          flightData.yaw = hex2Float(cmdIn[11], cmdIn[12]);
          flight->setFlight(&flightData);
          TIM_Cmd(TIM2, ENABLE);
          printf("roll_or_x =%f\n", hex2Float(cmdIn[5], cmdIn[6]));
          printf("pitch_or_y =%f\n", hex2Float(cmdIn[7], cmdIn[8]));
          printf("thr_z =%f\n", hex2Float(cmdIn[9], cmdIn[10]));
          printf("yaw =%f\n\n", hex2Float(cmdIn[11], cmdIn[12]));
        }
        else if (cmdIn[3] == 0x02)
        {//for display ur hex2int
          printf("roll_or_x =%f\n", hex2Float(cmdIn[5], cmdIn[6]));
          printf("pitch_or_y =%f\n", hex2Float(cmdIn[7], cmdIn[8]));
          printf("thr_z =%f\n", hex2Float(cmdIn[9], cmdIn[10]));
          printf("yaw =%f\n\n", hex2Float(cmdIn[11], cmdIn[12]));
        }
        break;

      case 0x05:
        switch(cmdIn[3])
        {
        case 0x01:
          flight->task(Flight::TASK_GOHOME);
          break;
        case 0x02:
          flight->task(Flight::TASK_TAKEOFF);
          break;
        case 0x03:
          flight->task(Flight::TASK_LANDING);
          break;
        }
        break;

      case 0x06:
        if (cmdIn[3] == 0x00)              //0x06 0x00 to stop VRC
        {
          VRCResetData();
          TIM_Cmd(TIM1, DISABLE);
        }
        else if (cmdIn[3] == 0x02)            //0x06 0x01 to turn VRC to F gear
        {
          VRC_TakeControl();
          TIM_Cmd(TIM1, ENABLE);
        }
        else if (cmdIn[3] == 0x01)          //0x06 0x02 to reset data
        {
          VRCResetData();
          TIM_Cmd(TIM1, ENABLE);
        };
        break;

      case 0x07:
        if(cmdIn[3] == 0x00)
        {
          tryHotpoint(cmdIn[4], cmdIn[5], cmdIn[6]);
        }
        else
        {
          stopHotpoint();
        }
        break;
      case 0x08:
        printf("TimeStamp is %d\r\n", api->getBroadcastData().timeStamp.time);
        printf("Battery capacity remains %d percent\r\n",
            api->getBroadcastData().battery);
        break;

      case 0x09:
        if (cmdIn[3] == 0x01)
        {
          startLocalNavExample();
        }
        else if(cmdIn[3] == 0x00)
        {
          stopLocalNavExample();
        }
        break;
    */
    default:
      break;
  }
}

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus
void
USART2_IRQHandler(void)
{
  if (USART_GetFlagStatus(USART2, USART_FLAG_RXNE) == SET)
  {
    uint8_t oneByte = USART_ReceiveData(USART2);
    if (myTerminal.rxIndex == 0)
    {
      if (oneByte == 0xFA)
      {
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxIndex                   = 1;
      }
      else
      {
        ;
      }
    }
    else
    {
      if (oneByte == 0xFE) // receive a 0xFE would lead to a command-execution
      {
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxLength                  = myTerminal.rxIndex + 1;
        myTerminal.rxIndex                   = 0;
        myTerminal.cmdReadyFlag              = 1;
      }
      else
      {
        myTerminal.cmdIn[myTerminal.rxIndex] = oneByte;
        myTerminal.rxIndex++;
      }
    }
  }
}
#ifdef __cplusplus
}
#endif //__cplusplus

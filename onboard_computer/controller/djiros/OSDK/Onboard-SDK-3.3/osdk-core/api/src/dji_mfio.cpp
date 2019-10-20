/** @file dji_mfio.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  MFIO API for DJI OSDK library
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_mfio.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

MFIO::MFIO(Vehicle* vehicle)
{
  this->vehicle = vehicle;
  channelUsage  = 0;
}

MFIO::~MFIO()
{
}

void
MFIO::config(MFIO::MODE mode, CHANNEL channel, uint32_t defaultValue,
             uint16_t freq, VehicleCallBack callback, UserData userData)
{
  InitData data;
  if ((channelUsage & (1 << channel)) == 0)
  {
    // channelUsage |= (1 << channel);
    data.channel = channel;
    data.mode    = mode;
    data.value   = defaultValue;
    data.freq    = freq;

    int cbIndex = vehicle->callbackIdIndex();
    if (callback)
    {
      vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
      vehicle->nbUserData[cbIndex]          = userData;
    }
    else
    {
      vehicle->nbCallbackFunctions[cbIndex] = (void*)&MFIO::initCallback;
      vehicle->nbUserData[cbIndex]          = NULL;
    }

    vehicle->protocolLayer->send(2, 0, OpenProtocol::CMDSet::MFIO::init, &data,
                                 sizeof(data), 500, 2, true, cbIndex);
  }
  else
  {
    DERROR("channel already used 0x%X,0x%X", channelUsage, 1 << channel);
  }
}

ACK::ErrorCode
MFIO::config(MFIO::MODE mode, CHANNEL channel, uint32_t defaultValue,
             uint16_t freq, int wait_timeout)
{
  ACK::ErrorCode ack;
  InitData       data;

  if ((channelUsage & (1 << channel)) == 0)
  {
    // channelUsage |= (1 << channel);
    data.channel = channel;
    data.mode    = mode;
    data.value   = defaultValue;
    data.freq    = freq;
    DSTATUS("sent");
    vehicle->protocolLayer->send(2, 0, OpenProtocol::CMDSet::MFIO::init, &data,
                                 sizeof(data), 500, 2, false, 0);

    ack = *((ACK::ErrorCode*)vehicle->waitForACK(
      OpenProtocol::CMDSet::MFIO::init, wait_timeout));

    return ack;
  }
  else
  {
    DERROR("Channel already used 0x%X,0x%X", channelUsage, 1 << channel);
    ack.info.cmd_set = OpenProtocol::CMDSet::mfio;
    ack.data         = 0xFF;
    return ack;
  }
}

void
MFIO::initCallback(RecvContainer recvFrame, UserData data)
{
  /* Comment out API_LOG until we have a nicer solution, or we update calback
   * prototype
   * DSTATUS( "MFIO initMFIOCallback");
   */
  uint16_t ack_length =
    recvFrame.recvInfo.len - static_cast<uint16_t>(Protocol::PackageMin);
  uint8_t* ackPtr = recvFrame.recvData.raw_ack_array;

  uint8_t errorFlag;

  memcpy((uint8_t*)&errorFlag, ackPtr, 1);
}

void
MFIO::setValue(MFIO::CHANNEL channel, uint32_t value, VehicleCallBack callback,
               UserData userData)
{

  SetData data;
  data.channel = channel;
  data.value   = value;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)&MFIO::setValueCallback;
    vehicle->nbUserData[cbIndex]          = NULL;
  }

  vehicle->protocolLayer->send(2, 0, OpenProtocol::CMDSet::MFIO::set, &data,
                               sizeof(data), 500, 2, true, cbIndex);
}

ACK::ErrorCode
MFIO::setValue(CHANNEL channel, uint32_t value, int wait_timeout)
{

  ACK::ErrorCode ack;
  SetData        data;
  data.channel = channel;
  data.value   = value;

  vehicle->protocolLayer->send(2, 0, OpenProtocol::CMDSet::MFIO::set, &data,
                               sizeof(data), 500, 2, false, 0);

  ack = *((ACK::ErrorCode*)vehicle->waitForACK(OpenProtocol::CMDSet::MFIO::set,
                                               wait_timeout));

  return ack;
}

void
MFIO::setValueCallback(RecvContainer recvFrame, UserData data)
{

  uint16_t ack_length =
    recvFrame.recvInfo.len - static_cast<uint16_t>(Protocol::PackageMin);
  uint8_t* ackPtr = recvFrame.recvData.raw_ack_array;

  uint8_t errorFlag;

  memcpy((uint8_t*)&errorFlag, ackPtr, 1);
}

void
MFIO::getValue(MFIO::CHANNEL channel, VehicleCallBack callback,
               UserData userData)
{

  GetData data;
  data = channel;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)&MFIO::getValueCallback;
    vehicle->nbUserData[cbIndex]          = NULL;
  }

  vehicle->protocolLayer->send(2, 0, OpenProtocol::CMDSet::MFIO::get, &data,
                               sizeof(data), 500, 3, true, cbIndex);
}

ACK::MFIOGet
MFIO::getValue(CHANNEL channel, int wait_timeout)
{
  ACK::MFIOGet ack;

  GetData data;
  data = channel;

  vehicle->protocolLayer->send(2, 0, OpenProtocol::CMDSet::MFIO::get, &data,
                               sizeof(data), 500, 3, false, 0);

  ack = *((ACK::MFIOGet*)vehicle->waitForACK(OpenProtocol::CMDSet::MFIO::get,
                                             wait_timeout));
  return ack;
}

void
MFIO::getValueCallback(RecvContainer recvFrame, UserData data)
{
  uint16_t ack_length =
    recvFrame.recvInfo.len - static_cast<uint16_t>(Protocol::PackageMin);
  uint8_t* ackPtr = recvFrame.recvData.raw_ack_array;

  uint8_t  result;
  uint32_t value;

  memcpy((uint8_t*)&result, ackPtr, 1);
  memcpy((uint8_t*)&value, ackPtr + 1, 4);

  DSTATUS("\n status: %d\n", result);
  DSTATUS("\n value: %d\n", value);
}

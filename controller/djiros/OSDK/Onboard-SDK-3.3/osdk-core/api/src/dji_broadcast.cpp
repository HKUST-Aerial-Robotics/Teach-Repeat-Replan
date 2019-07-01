/** @file dji_broadcast.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  Broadcast Telemetry API for DJI onboardSDK library
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#include "dji_broadcast.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

void
DataBroadcast::unpackCallback(Vehicle* vehicle, RecvContainer recvFrame,
                              UserData data)
{
  DataBroadcast* broadcastPtr = (DataBroadcast*)data;

  if (broadcastPtr->getVehicle()->getFwVersion() != Version::M100_31)
  {
    broadcastPtr->unpackData(&recvFrame);
  }
  else
  {
    broadcastPtr->unpackM100Data(&recvFrame);
  }

  if (broadcastPtr->userCbHandler.callback)
  {
    broadcastPtr->userCbHandler.callback(vehicle, recvFrame,
                                         broadcastPtr->userCbHandler.userData);
  }
}

DataBroadcast::DataBroadcast(Vehicle* vehiclePtr)
{
  if (vehiclePtr)
  {
    setVehicle(vehiclePtr);
  }
  unpackHandler.callback = unpackCallback;
  unpackHandler.userData = this;

  userCbHandler.callback = 0;
  userCbHandler.userData = 0;
}

DataBroadcast::~DataBroadcast()
{
  this->setUserBroadcastCallback(0, NULL);
  unpackHandler.callback = 0;
  unpackHandler.userData = 0;
}

// clang-format off
Telemetry::TimeStamp
DataBroadcast::getTimeStamp()
{
  Telemetry::TimeStamp  data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  if(vehicle->getFwVersion() != Version::M100_31)
  {
    data = timeStamp;
  }
  else
  {
    // Supported Broadcast data in Matrice 100
    data.time_ms = m100TimeStamp.time;
    data.time_ns = m100TimeStamp.nanoTime;
  }
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::SyncStamp
DataBroadcast::getSyncStamp()
{
  Telemetry::SyncStamp data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  if(vehicle->getFwVersion() != Version::M100_31)
  {
    data = syncStamp;
  }
  else
  {
    // Supported Broadcast data in Matrice 100
    data.flag = m100TimeStamp.syncFlag;
  }
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::Quaternion
DataBroadcast::getQuaternion()
{
  Telemetry::Quaternion data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = q;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::Vector3f
DataBroadcast::getAcceleration()
{
  Telemetry::Vector3f data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = a;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::Vector3f
DataBroadcast::getVelocity()
{
  Telemetry::Vector3f data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  if(vehicle->getFwVersion() != Version::M100_31)
  {
    data = v;
  }
  else
  {
    // Supported Broadcast data in Matrice 100
    data.x = m100Velocity.x;
    data.y = m100Velocity.y;
    data.z = m100Velocity.z;
  }
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::VelocityInfo
DataBroadcast::getVelocityInfo()
{
  Telemetry::VelocityInfo data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  if(vehicle->getFwVersion() != Version::M100_31)
  {
    data = vi;
  }
  else
  {
    // Supported Broadcast data in Matrice 100
    data.health = m100Velocity.health;
    data.reserve = m100Velocity.reserve;
    // TODO add sensorID (only M100)
  }
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::Vector3f
DataBroadcast::getAngularRate()
{
  Telemetry::Vector3f data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = w;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::GlobalPosition
DataBroadcast::getGlobalPosition()
{
  Telemetry::GlobalPosition data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = gp;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

// Not supported on Matrice 100
Telemetry::RelativePosition
DataBroadcast::getRelativePosition()
{
  Telemetry::RelativePosition data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = rp;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

// Not supported on Matrice 100
Telemetry::GPSInfo
DataBroadcast::getGPSInfo()
{
  Telemetry::GPSInfo data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = gps;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

// Not supported on Matrice 100
Telemetry::RTK
DataBroadcast::getRTKInfo()
{
  Telemetry::RTK data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = rtk;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::Mag
DataBroadcast::getMag()
{
  Telemetry::Mag data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = mag;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::RC
DataBroadcast::getRC()
{
  Telemetry::RC data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = rc;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::Gimbal
DataBroadcast::getGimbal()
{
  Telemetry::Gimbal data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = gimbal;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::Status
DataBroadcast::getStatus()
{
  Telemetry::Status data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  if(vehicle->getFwVersion() != Version::M100_31)
  {
    data = status;
  }
  else
  {
    // Supported Broadcast data in Matrice 100
    data.flight = m100FlightStatus;
  }
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::Battery
DataBroadcast::getBatteryInfo()
{
  Telemetry::Battery data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  if(vehicle->getFwVersion() != Version::M100_31)
  {
    data = battery;
  }
  else
  {
    // Supported Broadcast data in Matrice 100
    data.capacity = m100Battery;
  }
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}

Telemetry::SDKInfo
DataBroadcast::getSDKInfo()
{
  Telemetry::SDKInfo data;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  data = info;
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
  return data;
}
// clang-format on

Vehicle*
DataBroadcast::getVehicle() const
{
  return vehicle;
}

void
DataBroadcast::setVehicle(Vehicle* vehiclePtr)
{
  vehicle = vehiclePtr;
}

void
DataBroadcast::setBroadcastFreq(uint8_t* dataLenIs16, VehicleCallBack callback,
                                UserData userData)
{
  for (int i = 0; i < 16; ++i)
  {
    dataLenIs16[i] = (dataLenIs16[i] > 7 ? 5 : dataLenIs16[i]);
  }

  uint32_t cmd_timeout = 100; // unit is ms
  uint32_t retry_time  = 1;

  int cbIndex = vehicle->callbackIdIndex();
  if (callback)
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)callback;
    vehicle->nbUserData[cbIndex]          = userData;
  }
  else
  {
    vehicle->nbCallbackFunctions[cbIndex] = (void*)setFrequencyCallback;
    vehicle->nbUserData[cbIndex]          = NULL;
  }

  vehicle->protocolLayer->send(
    2, 0, OpenProtocol::CMDSet::Activation::frequency, dataLenIs16, 16,
    cmd_timeout, retry_time, true, cbIndex);
}

ACK::ErrorCode
DataBroadcast::setBroadcastFreq(uint8_t* dataLenIs16, int timeout)
{
  ACK::ErrorCode ack;

  for (int i = 0; i < 16; ++i)
  {
    dataLenIs16[i] = (dataLenIs16[i] > 7 ? 5 : dataLenIs16[i]);
  }

  vehicle->protocolLayer->send(2, 0,
                               OpenProtocol::CMDSet::Activation::frequency,
                               dataLenIs16, 16, 100, 1, 0, 0);

  ack = *((ACK::ErrorCode*)getVehicle()->waitForACK(
    OpenProtocol::CMDSet::Activation::frequency, timeout));

  return ack;
}

void
DataBroadcast::unpackData(RecvContainer* pRecvFrame)
{
  uint8_t* pdata = pRecvFrame->recvData.raw_ack_array;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  passFlag = *(uint16_t*)pdata;
  pdata += sizeof(uint16_t);
  // clang-format off
  unpackOne(FLAG_TIME        ,&timeStamp ,pdata,sizeof(timeStamp ));
  unpackOne(FLAG_TIME        ,&syncStamp ,pdata,sizeof(syncStamp ));
  unpackOne(FLAG_QUATERNION  ,&q         ,pdata,sizeof(q         ));
  unpackOne(FLAG_ACCELERATION,&a         ,pdata,sizeof(a         ));
  unpackOne(FLAG_VELOCITY    ,&v         ,pdata,sizeof(v         ));
  unpackOne(FLAG_VELOCITY    ,&vi        ,pdata,sizeof(vi        ));
  unpackOne(FLAG_ANGULAR_RATE,&w         ,pdata,sizeof(w         ));
  unpackOne(FLAG_POSITION    ,&gp        ,pdata,sizeof(gp        ));
  unpackOne(FLAG_POSITION    ,&rp        ,pdata,sizeof(rp        ));
  unpackOne(FLAG_GPSINFO     ,&gps       ,pdata,sizeof(gps       ));
  unpackOne(FLAG_RTKINFO     ,&rtk       ,pdata,sizeof(rtk       ));
  unpackOne(FLAG_MAG         ,&mag       ,pdata,sizeof(mag       ));
  unpackOne(FLAG_RC          ,&rc        ,pdata,sizeof(rc        ));
  unpackOne(FLAG_GIMBAL      ,&gimbal    ,pdata,sizeof(gimbal    ));
  unpackOne(FLAG_STATUS      ,&status    ,pdata,sizeof(status    ));
  unpackOne(FLAG_BATTERY     ,&battery   ,pdata,sizeof(battery   ));
  unpackOne(FLAG_DEVICE      ,&info      ,pdata,sizeof(info      ));
  // clang-format on
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
}

void
DataBroadcast::unpackM100Data(RecvContainer* pRecvFrame)
{
  uint8_t* pdata = pRecvFrame->recvData.raw_ack_array;
  vehicle->protocolLayer->getThreadHandle()->lockMSG();
  passFlag = *(uint16_t*)pdata;
  pdata += sizeof(uint16_t);
  // clang-format off
  unpackOne(FLAG_TIME        ,&m100TimeStamp   ,pdata,sizeof(m100TimeStamp   ));
  unpackOne(FLAG_QUATERNION  ,&q               ,pdata,sizeof(q               ));
  unpackOne(FLAG_ACCELERATION,&a               ,pdata,sizeof(a               ));
  unpackOne(FLAG_VELOCITY    ,&m100Velocity    ,pdata,sizeof(m100Velocity    ));
  unpackOne(FLAG_ANGULAR_RATE,&w               ,pdata,sizeof(w               ));
  unpackOne(FLAG_POSITION    ,&gp              ,pdata,sizeof(gp              ));
  unpackOne(FLAG_M100_MAG    ,&mag             ,pdata,sizeof(mag             ));
  unpackOne(FLAG_M100_RC     ,&rc              ,pdata,sizeof(rc              ));
  unpackOne(FLAG_M100_GIMBAL ,&gimbal          ,pdata,sizeof(gimbal          ));
  unpackOne(FLAG_M100_STATUS ,&m100FlightStatus,pdata,sizeof(m100FlightStatus));
  unpackOne(FLAG_M100_BATTERY,&m100Battery     ,pdata,sizeof(m100Battery     ));
  unpackOne(FLAG_M100_DEVICE ,&info            ,pdata,sizeof(info            ));
  // clang-format on
  vehicle->protocolLayer->getThreadHandle()->freeMSG();
}

void
DataBroadcast::unpackOne(DataBroadcast::FLAG flag, void* data, uint8_t*& buf,
                         size_t size)
{
  if (flag & passFlag)
  {
    memcpy((uint8_t*)data, (uint8_t*)buf, size);
    buf += size;
  }
}

void
DataBroadcast::setVersionDefaults(uint8_t* frequencyBuffer)
{
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    setFreqDefaults(frequencyBuffer);
  }
  else
  {
    setFreqDefaultsM100_31(frequencyBuffer);
  }
}

void
DataBroadcast::setBroadcastFreqDefaults()
{
  uint8_t frequencyBuffer[16];
  setVersionDefaults(frequencyBuffer);
  setBroadcastFreq(frequencyBuffer);
}

ACK::ErrorCode
DataBroadcast::setBroadcastFreqDefaults(int timeout)
{
  uint8_t frequencyBuffer[16];
  setVersionDefaults(frequencyBuffer);
  return setBroadcastFreq(frequencyBuffer, timeout);
}

void
DataBroadcast::setFreqDefaultsM100_31(uint8_t* freq)
{
  /* Channels definition for M100
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - Magnetometer
   * 7 - RC Channels Data
   * 8 - Gimbal Data
   * 9 - Flight Status
   * 10 - Battery Level
   * 11 - Control Information
   */
  freq[0]  = FREQ_1HZ;
  freq[1]  = FREQ_10HZ;
  freq[2]  = FREQ_50HZ;
  freq[3]  = FREQ_100HZ;
  freq[4]  = FREQ_50HZ;
  freq[5]  = FREQ_10HZ;
  freq[6]  = FREQ_1HZ;
  freq[7]  = FREQ_10HZ;
  freq[8]  = FREQ_50HZ;
  freq[9]  = FREQ_100HZ;
  freq[10] = FREQ_50HZ;
  freq[11] = FREQ_10HZ;
}

void
DataBroadcast::setFreqDefaults(uint8_t* freq)
{
  /* Channels definition for A3/N3
   * 0 - Timestamp
   * 1 - Attitude Quaterniouns
   * 2 - Acceleration
   * 3 - Velocity (Ground Frame)
   * 4 - Angular Velocity (Body Frame)
   * 5 - Position
   * 6 - GPS Detailed Information
   * 7 - RTK Detailed Information
   * 8 - Magnetometer
   * 9 - RC Channels Data
   * 10 - Gimbal Data
   * 11 - Flight Statusack
   * 12 - Battery Level
   * 13 - Control Information
   */
  freq[0]  = FREQ_50HZ;
  freq[1]  = FREQ_50HZ;
  freq[2]  = FREQ_50HZ;
  freq[3]  = FREQ_50HZ;
  freq[4]  = FREQ_50HZ;
  freq[5]  = FREQ_50HZ;
  freq[6]  = FREQ_0HZ; // Don't send GPS details
  freq[7]  = FREQ_0HZ; // Don't send RTK
  freq[8]  = FREQ_0HZ; // Don't send Mag
  freq[9]  = FREQ_50HZ;
  freq[10] = FREQ_50HZ;
  freq[11] = FREQ_10HZ;
  freq[12] = FREQ_1HZ;
  freq[13] = FREQ_1HZ;
}

void
DataBroadcast::setBroadcastFreqToZero()
{
  uint8_t freq[16];

  freq[0]  = FREQ_0HZ;
  freq[1]  = FREQ_0HZ;
  freq[2]  = FREQ_0HZ;
  freq[3]  = FREQ_0HZ;
  freq[4]  = FREQ_0HZ;
  freq[5]  = FREQ_0HZ;
  freq[6]  = FREQ_0HZ;
  freq[7]  = FREQ_0HZ;
  freq[8]  = FREQ_0HZ;
  freq[9]  = FREQ_0HZ;
  freq[10] = FREQ_0HZ;
  freq[11] = FREQ_0HZ;
  freq[12] = FREQ_0HZ;
  freq[13] = FREQ_0HZ;
  setBroadcastFreq(freq);
}

void
DataBroadcast::setFrequencyCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                    UserData userData)
{

  ACK::ErrorCode ackErrorCode;
  ackErrorCode.data = OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR;
  ackErrorCode.info = recvFrame.recvInfo;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <= 2)
  {
    // Two-byte ACK
    ackErrorCode.data = recvFrame.recvData.ack;
  }

  if (!ACK::getError(ackErrorCode))
  {
    ACK::getErrorCodeMessage(ackErrorCode, __func__);
  }
}

void
DataBroadcast::setUserBroadcastCallback(VehicleCallBack callback,
                                        UserData        userData)
{
  userCbHandler.callback = callback;
  userCbHandler.userData = userData;
}

uint16_t
DataBroadcast::getPassFlag()
{
  return passFlag;
}

uint16_t
DataBroadcast::getBroadcastLength()
{
  return this->broadcastLength;
}

void
DataBroadcast::setBroadcastLength(uint16_t length)
{
  this->broadcastLength = length;
}

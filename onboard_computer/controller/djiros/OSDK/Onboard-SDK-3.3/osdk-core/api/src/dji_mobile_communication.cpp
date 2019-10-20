/** @file dji_mobile_communication.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Implementation of DJI Mobile-Onboard SDK Communication (MOC)
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#include "dji_mobile_communication.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

MobileCommunication::MobileCommunication(Vehicle* vehicle)
  : vehicle(vehicle)
{
  this->fromMSDKHandler.callback = getDataFromMSDKCallback;
  this->fromMSDKHandler.userData = 0;
}

MobileCommunication::~MobileCommunication()
{
  this->fromMSDKHandler.callback = 0;
  this->fromMSDKHandler.userData = 0;
}

Vehicle*
MobileCommunication::getVehicle() const
{
  return vehicle;
}

void
MobileCommunication::setVehicle(Vehicle* value)
{
  vehicle = value;
}

void
MobileCommunication::sendDataToMSDK(uint8_t* data, uint8_t len)
{
  if (len > 100)
  {
    DERROR("Too much data to send");
    return;
  }
  vehicle->protocolLayer->send(0, 0, OpenProtocol::CMDSet::Activation::toMobile,
                               data, len, 500, 1, NULL, 0);
}

void
MobileCommunication::getDataFromMSDKCallback(Vehicle*      vehiclePtr,
                                             RecvContainer recvFrame,
                                             UserData      userData)
{
  if (recvFrame.recvInfo.len - Protocol::PackageMin <= 100)
  {
    DSTATUS("Received mobile Data of len %d\n", recvFrame.recvInfo.len);
  }
}

void
MobileCommunication::setFromMSDKCallback(VehicleCallBack callback,
                                         UserData        userData)
{
  this->fromMSDKHandler.callback = callback;
  this->fromMSDKHandler.userData = userData;
}

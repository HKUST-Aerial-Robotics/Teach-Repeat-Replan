/** @file dji_hardware_sync.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Hardware Sync API for DJI OSDK
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_hardware_sync.hpp"
#include "dji_vehicle.hpp"

using namespace DJI;
using namespace DJI::OSDK;

HardwareSync::HardwareSync(Vehicle* vehiclePtr)
  : vehicle(vehiclePtr)
{
}

void
HardwareSync::setSyncFreq(uint32_t freqInHz, uint16_t tag)
{
  SyncSettings data;
  data.freq = freqInHz;
  data.tag  = tag;
  startSync(data);
}

void
HardwareSync::startSync(SyncSettings& data)
{
  vehicle->protocolLayer->send(0, encrypt,
                               OpenProtocol::CMDSet::HardwareSync::broadcast,
                               &data, sizeof(data));
}

/** @file dji_hardware_sync.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Hardware Sync API for DJI OSDK
 *  @details Use with Subscription UID_HARD_SYNC.
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef HARDSYNC_H
#define HARDSYNC_H

#include "dji_type.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declarations
class Vehicle;

/*! @brief APIs for controlling Hardware Sync
 *
 *  @details These APIs enable you to output a pulse/pulse train along with
 *  a software packet with synchronized timestamps and sensor data.
 *
 *  @note You must use this in conjunction with TOPIC_HARD_SYNC subscription.
 *  @note You need to set a F-channel to Sync through DJI Assistant 2.
 */
class HardwareSync
{

public:
#pragma pack(1)

  typedef struct SyncSettings
  {
    uint32_t freq;
    uint16_t tag;
  } SyncSettings;

#pragma pack()

public:
  HardwareSync(Vehicle* vehiclePtr = 0);

  /*! @brief Call this API to start sending a hardware pulse and
   *  set up a software packet to accompany it
   *  @details You need to select a pin on DJI Assistant 2 that will output this
   *  hardware pulse.
   *  To receive the software packet that accompanies this pulse,
   *  you will need to subscribe to TOPIC_HARD_SYNC.
   *
   *  @param freqInHz The frequency at which you want this pulse to be output.
   *  @param tag Identification to match pulse with the corresponding software
   *  packet
   */
  void setSyncFreq(uint32_t freqInHz, uint16_t tag = 0);

  /*! @brief Internal setter function that is called by setSyncFreq function
   *  @details Use setSyncFreq instead of this direct interface.
   *
   *  @param data Struct of type SyncCmdData.
   */
  void startSync(SyncSettings& data);

private:
  Vehicle* vehicle;
};
} // OSDK
} // DJI

#endif // HARDSYNC_H

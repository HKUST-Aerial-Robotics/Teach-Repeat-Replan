/** @file dji_command.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK OpenProtocol Command IDs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJI_COMMAND_H
#define DJI_COMMAND_H

#include "dji_error.hpp"
#include <stdint.h>

namespace DJI
{
namespace OSDK
{

class OpenProtocol : public ErrorCode
{
public:
  const static int MAX_CMD_ARRAY_SIZE = 2;

  class CMDSet
  {
  public:
    typedef struct Activation
    {
      const static uint8_t getVersion[MAX_CMD_ARRAY_SIZE];
      const static uint8_t activate[MAX_CMD_ARRAY_SIZE];
      const static uint8_t frequency[MAX_CMD_ARRAY_SIZE];
      const static uint8_t toMobile[MAX_CMD_ARRAY_SIZE];
    } Activation;

    typedef struct Broadcast
    {
      const static uint8_t broadcast[MAX_CMD_ARRAY_SIZE];
      const static uint8_t fromMobile[MAX_CMD_ARRAY_SIZE];
      const static uint8_t lostCTRL[MAX_CMD_ARRAY_SIZE];
      const static uint8_t mission[MAX_CMD_ARRAY_SIZE];
      const static uint8_t subscribe[MAX_CMD_ARRAY_SIZE];
      const static uint8_t test[MAX_CMD_ARRAY_SIZE];
      const static uint8_t waypoint[MAX_CMD_ARRAY_SIZE];
    } Broadcast;

    typedef struct Control
    {
      const static uint8_t setControl[MAX_CMD_ARRAY_SIZE];
      const static uint8_t task[MAX_CMD_ARRAY_SIZE];
      // CMD_ID_STATUS Not used at all
      const static uint8_t status[MAX_CMD_ARRAY_SIZE];
      const static uint8_t control[MAX_CMD_ARRAY_SIZE];
      // CMD_ID_SETARM Supported on Matrice 100, A3, N3
      // with firmware version < 3.3
      const static uint8_t setArm[MAX_CMD_ARRAY_SIZE];
      const static uint8_t gimbalSpeed[MAX_CMD_ARRAY_SIZE];
      const static uint8_t gimbalAngle[MAX_CMD_ARRAY_SIZE];
      const static uint8_t cameraShot[MAX_CMD_ARRAY_SIZE];
      const static uint8_t cameraVideoStart[MAX_CMD_ARRAY_SIZE];
      const static uint8_t cameraVideoStop[MAX_CMD_ARRAY_SIZE];
    } Control;

    typedef struct Mission
    {
      // Waypoint mission commands
      const static uint8_t waypointInit[MAX_CMD_ARRAY_SIZE];
      const static uint8_t waypointAddPoint[MAX_CMD_ARRAY_SIZE];
      const static uint8_t waypointSetStart[MAX_CMD_ARRAY_SIZE];
      const static uint8_t waypointSetPause[MAX_CMD_ARRAY_SIZE];
      const static uint8_t waypointDownload[MAX_CMD_ARRAY_SIZE];
      const static uint8_t waypointIndexDownload[MAX_CMD_ARRAY_SIZE];
      const static uint8_t waypointSetVelocity[MAX_CMD_ARRAY_SIZE];
      const static uint8_t waypointGetVelocity[MAX_CMD_ARRAY_SIZE];
      // Hotpint mission commands
      const static uint8_t hotpointStart[MAX_CMD_ARRAY_SIZE];
      const static uint8_t hotpointStop[MAX_CMD_ARRAY_SIZE];
      const static uint8_t hotpointSetPause[MAX_CMD_ARRAY_SIZE];
      const static uint8_t hotpointYawRate[MAX_CMD_ARRAY_SIZE];
      const static uint8_t hotpointRadius[MAX_CMD_ARRAY_SIZE];
      const static uint8_t hotpointSetYaw[MAX_CMD_ARRAY_SIZE];
      const static uint8_t hotpointDownload[MAX_CMD_ARRAY_SIZE];
      // Follow mission commands
      const static uint8_t followStart[MAX_CMD_ARRAY_SIZE];
      const static uint8_t followStop[MAX_CMD_ARRAY_SIZE];
      const static uint8_t followSetPause[MAX_CMD_ARRAY_SIZE];
      const static uint8_t followTarget[MAX_CMD_ARRAY_SIZE];
    } Mission;

    typedef struct HardwareSync
    {
      const static uint8_t broadcast[MAX_CMD_ARRAY_SIZE];
    } HardwareSync;

    typedef struct VirtualRC
    {
      const static uint8_t settings[MAX_CMD_ARRAY_SIZE];
      const static uint8_t data[MAX_CMD_ARRAY_SIZE];
    } VirtualRC;

    typedef struct MFIO
    {
      const static uint8_t init[MAX_CMD_ARRAY_SIZE];
      const static uint8_t get[MAX_CMD_ARRAY_SIZE];
      const static uint8_t set[MAX_CMD_ARRAY_SIZE];
    } MFIO;

    typedef struct Subscribe
    {
      const static uint8_t versionMatch[MAX_CMD_ARRAY_SIZE];
      const static uint8_t addPackage[MAX_CMD_ARRAY_SIZE];
      const static uint8_t reset[MAX_CMD_ARRAY_SIZE];
      const static uint8_t removePackage[MAX_CMD_ARRAY_SIZE];
      // TODO implement API call
      const static uint8_t updatePackageFreq[MAX_CMD_ARRAY_SIZE];
      const static uint8_t pauseResume[MAX_CMD_ARRAY_SIZE];
      // TODO implement API call
      const static uint8_t getConfig[MAX_CMD_ARRAY_SIZE];
    } Subscribe;

    //! CMD SET definitions
    const static uint8_t activation   = 0x00;
    const static uint8_t control      = 0x01;
    const static uint8_t broadcast    = 0x02;
    const static uint8_t mission      = 0x03;
    const static uint8_t hardwareSync = 0x04;
    const static uint8_t virtualRC    = 0x05;
    const static uint8_t mfio         = 0x09;
    const static uint8_t subscribe    = 0x0B;
  };
};

} // namespace
} // namespace

#endif /* DJI_COMMAND_H */

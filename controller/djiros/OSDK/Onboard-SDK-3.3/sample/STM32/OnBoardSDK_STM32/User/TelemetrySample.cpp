/*! @file telemetry_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Telemetry API usage in a Linux environment.
 *  Shows example usage of the new data subscription API.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#include "TelemetrySample.h"

extern Vehicle  vehicle;
extern Vehicle* v;

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

bool
getBroadcastData()
{
  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 2000;

  // We will listen to five broadcast data sets:
  // 1. Flight Status
  // 2. Global Position
  // 3. RC Channels
  // 4. Velocity
  // 5. Quaternion

  // Please make sure your drone is in simulation mode. You can
  // fly the drone with your RC to get different values.

  Telemetry::Status         status;
  Telemetry::GlobalPosition globalPosition;
  Telemetry::RC             rc;
  Telemetry::Vector3f       velocity;
  Telemetry::Quaternion     quaternion;

  const int TIMEOUT = 20;

  // Re-set Broadcast frequencies to their default values
  ACK::ErrorCode ack = v->broadcast->setBroadcastFreqDefaults(TIMEOUT);

  // Print in a loop for 2 seconds
  while (elapsedTimeInMs < timeToPrintInMs)
  {
    // Matrice 100 broadcasts only flight status
    status         = v->broadcast->getStatus();
    globalPosition = v->broadcast->getGlobalPosition();
    rc             = v->broadcast->getRC();
    velocity       = v->broadcast->getVelocity();
    quaternion     = v->broadcast->getQuaternion();

    printf("Counter = %d:\n", elapsedTimeInMs);
    printf("-------\n");
    printf("Flight Status = %d\n", (unsigned)status.flight);
    printf("Position (LLA) = %.3f, %.3f, %.3f\n", globalPosition.latitude,
           globalPosition.longitude, globalPosition.altitude);
    printf("RC Commands (r/p/y/thr) = %d, %d, %d, %d\n", rc.roll, rc.pitch,
           rc.yaw, rc.throttle);
    printf("Velocity (vx,vy,vz) = %.3f, %.3f, %.3f\n", velocity.x, velocity.y,
           velocity.z);
    printf("Attitude Quaternion (w,x,y,z) = %.3f, %.3f, %.3f, %.3f\n",
           quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    printf("-------\n\n");

    elapsedTimeInMs += 5;
  }

  printf("Done printing!\n");
  return true;
}

bool
subscribeToData()
{

  // Counters
  int elapsedTimeInMs = 0;
  int timeToPrintInMs = 4000;

  // We will subscribe to six kinds of data:
  // 1. Flight Status at 1 Hz
  // 2. Fused Lat/Lon at 10Hz
  // 3. Fused Altitude at 10Hz
  // 4. RC Channels at 50 Hz
  // 5. Velocity at 50 Hz
  // 6. Quaternion at 200 Hz

  // Package 0: Subscribe to flight status at freq 1 Hz
  int       pkgIndex        = 0;
  int       freq            = 1;
  TopicName topicList1Hz[]  = { TOPIC_STATUS_FLIGHT };
  int       numTopic        = sizeof(topicList1Hz) / sizeof(topicList1Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList1Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 1: Subscribe to Lat/Lon, and Alt at freq 10 Hz
  pkgIndex                  = 1;
  freq                      = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED };
  numTopic                  = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  enableTimestamp           = false;

  pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 2: Subscribe to RC Channel and Velocity at freq 50 Hz
  pkgIndex                  = 2;
  freq                      = 50;
  TopicName topicList50Hz[] = { TOPIC_RC, TOPIC_VELOCITY };
  numTopic                  = sizeof(topicList50Hz) / sizeof(topicList50Hz[0]);
  enableTimestamp           = false;

  pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList50Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Package 3: Subscribe to Quaternion at freq 200 Hz.
  pkgIndex                   = 3;
  freq                       = 200;
  TopicName topicList200Hz[] = { TOPIC_QUATERNION };
  numTopic        = sizeof(topicList200Hz) / sizeof(topicList200Hz[0]);
  enableTimestamp = false;

  pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList200Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(500);
  /*ack = waitForACK();
  if(ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, func);

    // Cleanup
    v->subscribe->removePackage(pkgIndex);
    ack = waitForACK();
    if(ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, func);
    }

    return false;
  }*/

  // Wait for the data to start coming in.
  delay_nms(8000);

  // Get all the data once before the loop to initialize vars
  TypeMap<TOPIC_STATUS_FLIGHT>::type flightStatus;
  TypeMap<TOPIC_GPS_FUSED>::type     latLon;
  TypeMap<TOPIC_RC>::type            rc;
  TypeMap<TOPIC_VELOCITY>::type      velocity;
  TypeMap<TOPIC_QUATERNION>::type    quaternion;

  uint32_t PRINT_TIMEOUT = 4000; // milliseconds
  uint32_t RETRY_TICK    = 500;  // milliseconds
  uint32_t nextRetryTick = 0;    // millisesonds
  uint32_t timeoutTick;

  timeoutTick = v->protocolLayer->getDriver()->getTimeStamp() + PRINT_TIMEOUT;
  do
  {
    flightStatus = v->subscribe->getValue<TOPIC_STATUS_FLIGHT>();
    latLon       = v->subscribe->getValue<TOPIC_GPS_FUSED>();
    rc           = v->subscribe->getValue<TOPIC_RC>();
    velocity     = v->subscribe->getValue<TOPIC_VELOCITY>();
    quaternion   = v->subscribe->getValue<TOPIC_QUATERNION>();

    printf("Counter = %d:\n", elapsedTimeInMs);
    printf("-------\n");
    printf("Flight Status = %d\n", (int)flightStatus);
    printf("Position (LLA) = %.3f, %.3f, %.3f\n", latLon.latitude,
           latLon.longitude, latLon.altitude);
    printf("RC Commands (r/p/y/thr) = %d, %d, %d, %d\n", rc.roll, rc.pitch,
           rc.yaw, rc.throttle);
    printf("Velocity (vx,vy,vz) = %.3f, %.3f, %.3f\n", velocity.data.x,
           velocity.data.y, velocity.data.z);
    printf("Attitude Quaternion (w,x,y,z) = %.3f, %.3f, %.3f, %.3f\n",
           quaternion.q0, quaternion.q1, quaternion.q2, quaternion.q3);
    printf("-------\n\n");

    delay_nms(500);
    nextRetryTick = v->protocolLayer->getDriver()->getTimeStamp() + RETRY_TICK;
  } while (nextRetryTick < timeoutTick);

  printf("Done printing!\n");
  v->subscribe->removePackage(0);
  delay_nms(3000);
  v->subscribe->removePackage(1);
  delay_nms(3000);
  v->subscribe->removePackage(2);
  delay_nms(3000);
  v->subscribe->removePackage(3);
  delay_nms(3000);

  return true;
}
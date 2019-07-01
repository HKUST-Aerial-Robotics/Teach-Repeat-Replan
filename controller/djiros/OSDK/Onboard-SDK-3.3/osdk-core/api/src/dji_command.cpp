/** @file dji_command.cpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief All DJI OSDK OpenProtocol Command IDs
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_command.hpp"

const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Activation::getVersion[] = {
  OpenProtocol::CMDSet::activation, 0x00
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Activation::activate[] = {
  OpenProtocol::CMDSet::activation, 0x01
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Activation::frequency[] = {
  OpenProtocol::CMDSet::activation, 0x10
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Activation::toMobile[] = {
  OpenProtocol::CMDSet::activation, 0xFE
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Broadcast::broadcast[] = {
  OpenProtocol::CMDSet::broadcast, 0x00
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Broadcast::lostCTRL[] = {
  OpenProtocol::CMDSet::broadcast, 0x01
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Broadcast::fromMobile[] = {
  OpenProtocol::CMDSet::broadcast, 0x02
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Broadcast::mission[] = {
  OpenProtocol::CMDSet::broadcast, 0x03
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Broadcast::waypoint[] = {
  OpenProtocol::CMDSet::broadcast, 0x04
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Broadcast::subscribe[] = {
  OpenProtocol::CMDSet::broadcast, 0x05
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Broadcast::test[] = {
  OpenProtocol::CMDSet::broadcast, 0xEF
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::setControl[] = {
  OpenProtocol::CMDSet::control, 0x00
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::task[] = {
  OpenProtocol::CMDSet::control, 0x01
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::status[] = {
  OpenProtocol::CMDSet::control, 0x02
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::control[] = {
  OpenProtocol::CMDSet::control, 0x03
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::setArm[] = {
  OpenProtocol::CMDSet::control, 0x05
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::cameraShot[] = {
  OpenProtocol::CMDSet::control, 0x20
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::cameraVideoStart[] = {
  OpenProtocol::CMDSet::control, 0x21
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::cameraVideoStop[] = {
  OpenProtocol::CMDSet::control, 0x22
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::gimbalSpeed[] = {
  OpenProtocol::CMDSet::control, 0x1A
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Control::gimbalAngle[] = {
  OpenProtocol::CMDSet::control, 0x1B
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::waypointInit[] = {
  OpenProtocol::CMDSet::mission, 0x10
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::waypointAddPoint[] = {
  OpenProtocol::CMDSet::mission, 0x11
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::waypointSetStart[] = {
  OpenProtocol::CMDSet::mission, 0x12
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::waypointSetPause[] = {
  OpenProtocol::CMDSet::mission, 0x13
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::waypointDownload[] = {
  OpenProtocol::CMDSet::mission, 0x14
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::waypointIndexDownload[] = {
  OpenProtocol::CMDSet::mission, 0x15
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::waypointSetVelocity[] =
  { OpenProtocol::CMDSet::mission, 0x16 };
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::waypointGetVelocity[] =
  { OpenProtocol::CMDSet::mission, 0x17 };
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::hotpointStart[] = {
  OpenProtocol::CMDSet::mission, 0x20
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::hotpointStop[] = {
  OpenProtocol::CMDSet::mission, 0x21
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::hotpointSetPause[] = {
  OpenProtocol::CMDSet::mission, 0x22
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::hotpointYawRate[] = {
  OpenProtocol::CMDSet::mission, 0x23
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::hotpointRadius[] = {
  OpenProtocol::CMDSet::mission, 0x24
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::hotpointSetYaw[] = {
  OpenProtocol::CMDSet::mission, 0x25
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::hotpointDownload[] = {
  OpenProtocol::CMDSet::mission, 0x26
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::followStart[] = {
  OpenProtocol::CMDSet::mission, 0x30
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::followStop[] = {
  OpenProtocol::CMDSet::mission, 0x31
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::followSetPause[] = {
  OpenProtocol::CMDSet::mission, 0x32
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Mission::followTarget[] = {
  OpenProtocol::CMDSet::mission, 0x33
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::HardwareSync::broadcast[] = {
  OpenProtocol::CMDSet::hardwareSync, 0x00
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::VirtualRC::settings[] = {
  OpenProtocol::CMDSet::virtualRC
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::VirtualRC::data[] = {
  OpenProtocol::CMDSet::virtualRC
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::MFIO::init[] = {
  OpenProtocol::CMDSet::mfio, 0x02
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::MFIO::set[] = {
  OpenProtocol::CMDSet::mfio, 0x03
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::MFIO::get[] = {
  OpenProtocol::CMDSet::mfio, 0x04
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Subscribe::versionMatch[] = {
  OpenProtocol::CMDSet::subscribe, 0x00
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Subscribe::addPackage[] = {
  OpenProtocol::CMDSet::subscribe, 0x01
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Subscribe::reset[] = {
  OpenProtocol::CMDSet::subscribe, 0x02
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Subscribe::removePackage[] = {
  OpenProtocol::CMDSet::subscribe, 0x03
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Subscribe::updatePackageFreq[] =
  { OpenProtocol::CMDSet::subscribe, 0x04 };
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Subscribe::pauseResume[] = {
  OpenProtocol::CMDSet::subscribe, 0x05
};
const uint8_t DJI::OSDK::OpenProtocol::CMDSet::Subscribe::getConfig[] = {
  OpenProtocol::CMDSet::subscribe, 0x06
};

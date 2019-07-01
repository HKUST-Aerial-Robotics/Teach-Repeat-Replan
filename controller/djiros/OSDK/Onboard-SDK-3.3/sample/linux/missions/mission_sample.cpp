/*! @file mission_sample.cpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  GPS Missions API usage in a Linux environment.
 *  Shows example usage of the Waypoint Missions and Hotpoint Missions through
 * the
 *  Mission Manager API.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#include "mission_sample.hpp"

using namespace DJI::OSDK;
using namespace DJI::OSDK::Telemetry;

int
main(int argc, char** argv)
{
  // Initialize variables
  int functionTimeout = 1;

  // Setup OSDK.
  Vehicle* vehicle = setupOSDK(argc, argv);
  if (vehicle == NULL)
  {
    std::cout << "Vehicle not initialized, exiting.\n";
    return -1;
  }

  // Obtain Control Authority
  vehicle->obtainCtrlAuthority(functionTimeout);

  // Setup variables for use
  uint8_t wayptPolygonSides;
  int     hotptInitRadius;
  int     responseTimeout = 1;

  // Display interactive prompt
  std::cout
    << "| Available commands:                                            |"
    << std::endl;
  std::cout
    << "| [a] Waypoint Mission                                           |"
    << std::endl;
  std::cout
    << "| [b] Hotpoint Mission                                           |"
    << std::endl;
  char inputChar;
  std::cin >> inputChar;
  switch (inputChar)
  {
    case 'a':
      // Waypoint call
      wayptPolygonSides = 6;
      runWaypointMission(vehicle, wayptPolygonSides, responseTimeout);
      break;
    case 'b':
      hotptInitRadius = 10;
      runHotpointMission(vehicle, hotptInitRadius, responseTimeout);
      break;
    default:
      break;
  }

  delete (vehicle);
  return 0;
}

bool
setUpSubscription(DJI::OSDK::Vehicle* vehicle, int responseTimeout)
{
  // Telemetry: Verify the subscription
  ACK::ErrorCode subscribeStatus;

  subscribeStatus = vehicle->subscribe->verify(responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    return false;
  }

  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int       freq            = 10;
  TopicName topicList10Hz[] = { TOPIC_GPS_FUSED };
  int       numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool      enableTimestamp = false;

  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    DEFAULT_PACKAGE_INDEX, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  // Start listening to the telemetry data
  subscribeStatus =
    vehicle->subscribe->startPackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
  if (ACK::getError(subscribeStatus) != ACK::SUCCESS)
  {
    ACK::getErrorCodeMessage(subscribeStatus, __func__);
    // Cleanup
    ACK::ErrorCode ack =
      vehicle->subscribe->removePackage(DEFAULT_PACKAGE_INDEX, responseTimeout);
    if (ACK::getError(ack))
    {
      std::cout << "Error unsubscribing; please restart the drone/FC to get "
                   "back to a clean state.\n";
    }
    return false;
  }
  return true;
}

bool
teardownSubscription(DJI::OSDK::Vehicle* vehicle, const int pkgIndex,
                     int responseTimeout)
{
  ACK::ErrorCode ack =
    vehicle->subscribe->removePackage(pkgIndex, responseTimeout);
  if (ACK::getError(ack))
  {
    std::cout << "Error unsubscribing; please restart the drone/FC to get back "
                 "to a clean state.\n";
    return false;
  }
  return true;
}

bool
runWaypointMission(Vehicle* vehicle, uint8_t numWaypoints, int responseTimeout)
{
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    if (!setUpSubscription(vehicle, responseTimeout))
    {
      std::cout << "Failed to set up Subscription!" << std::endl;
      return false;
    }
    sleep(1);
  }

  // Waypoint Mission : Initialization
  WayPointInitSettings fdata;
  setWaypointInitDefaults(&fdata);

  fdata.indexNumber =
    numWaypoints + 1; // We add 1 to get the aircarft back to the start.

  float64_t increment = 0.000001;
  float32_t start_alt = 10;

  ACK::ErrorCode initAck = vehicle->missionManager->init(
    DJI_MISSION_TYPE::WAYPOINT, responseTimeout, &fdata);
  if (ACK::getError(initAck))
  {
    ACK::getErrorCodeMessage(initAck, __func__);
  }

  vehicle->missionManager->printInfo();
  std::cout << "Initializing Waypoint Mission..\n";

  // Waypoint Mission: Create Waypoints
  std::vector<WayPointSettings> generatedWaypts =
    createWaypoints(vehicle, numWaypoints, increment, start_alt);
  std::cout << "Creating Waypoints..\n";

  // Waypoint Mission: Upload the waypoints
  uploadWaypoints(vehicle, generatedWaypts, responseTimeout);
  std::cout << "Uploading Waypoints..\n";

  // Waypoint Mission: Start
  ACK::ErrorCode startAck =
    vehicle->missionManager->wpMission->start(responseTimeout);
  if (ACK::getError(startAck))
  {
    ACK::getErrorCodeMessage(initAck, __func__);
  }
  else
  {
    std::cout << "Starting Waypoint Mission.\n";
  }

  // Cleanup before return. The mission isn't done yet, but it doesn't need any
  // more input from our side.
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    return teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX,
                                responseTimeout);
  }

  return true;
}

void
setWaypointDefaults(WayPointSettings* wp)
{
  wp->damping         = 0;
  wp->yaw             = 0;
  wp->gimbalPitch     = 0;
  wp->turnMode        = 0;
  wp->hasAction       = 0;
  wp->actionTimeLimit = 100;
  wp->actionNumber    = 0;
  wp->actionRepeat    = 0;
  for (int i = 0; i < 16; ++i)
  {
    wp->commandList[i]      = 0;
    wp->commandParameter[i] = 0;
  }
}

void
setWaypointInitDefaults(WayPointInitSettings* fdata)
{
  fdata->maxVelocity    = 10;
  fdata->idleVelocity   = 5;
  fdata->finishAction   = 0;
  fdata->executiveTimes = 1;
  fdata->yawMode        = 0;
  fdata->traceMode      = 0;
  fdata->RCLostAction   = 1;
  fdata->gimbalPitch    = 0;
  fdata->latitude       = 0;
  fdata->longitude      = 0;
  fdata->altitude       = 0;
}

std::vector<DJI::OSDK::WayPointSettings>
createWaypoints(DJI::OSDK::Vehicle* vehicle, int numWaypoints,
                float64_t distanceIncrement, float32_t start_alt)
{
  // Create Start Waypoint
  WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition broadcastGPosition;

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    start_wp.latitude  = subscribeGPosition.latitude;
    start_wp.longitude = subscribeGPosition.longitude;
    start_wp.altitude  = start_alt;
    printf("Waypoint created at (LLA): %f \t%f \t%f\n",
           subscribeGPosition.latitude, subscribeGPosition.longitude,
           start_alt);
  }
  else
  {
    broadcastGPosition = vehicle->broadcast->getGlobalPosition();
    start_wp.latitude  = broadcastGPosition.latitude;
    start_wp.longitude = broadcastGPosition.longitude;
    start_wp.altitude  = start_alt;
    printf("Waypoint created at (LLA): %f \t%f \t%f\n",
           broadcastGPosition.latitude, broadcastGPosition.longitude,
           start_alt);
  }

  std::vector<DJI::OSDK::WayPointSettings> wpVector =
    generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
  return wpVector;
}

std::vector<DJI::OSDK::WayPointSettings>
generateWaypointsPolygon(WayPointSettings* start_data, float64_t increment,
                         int num_wp)
{

  // Let's create a vector to store our waypoints in.
  std::vector<DJI::OSDK::WayPointSettings> wp_list;

  // Some calculation for the polygon
  float64_t extAngle = 2 * M_PI / num_wp;

  // First waypoint
  start_data->index = 0;
  wp_list.push_back(*start_data);

  // Iterative algorithm
  for (int i = 1; i < num_wp; i++)
  {
    WayPointSettings  wp;
    WayPointSettings* prevWp = &wp_list[i - 1];
    setWaypointDefaults(&wp);
    wp.index     = i;
    wp.latitude  = (prevWp->latitude + (increment * cos(i * extAngle)));
    wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
    wp.altitude  = (prevWp->altitude + 1);
    wp_list.push_back(wp);
  }

  // Come back home
  start_data->index = num_wp;
  wp_list.push_back(*start_data);

  return wp_list;
}

void
uploadWaypoints(Vehicle*                                  vehicle,
                std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                int                                       responseTimeout)
{
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
           wp->longitude, wp->altitude);
    ACK::WayPointIndex wpDataACK =
      vehicle->missionManager->wpMission->uploadIndexData(&(*wp),
                                                          responseTimeout);

    ACK::getErrorCodeMessage(wpDataACK.ack, __func__);
  }
}

bool
runHotpointMission(Vehicle* vehicle, int initialRadius, int responseTimeout)
{
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    if (!setUpSubscription(vehicle, responseTimeout))
    {
      std::cout << "Failed to set up Subscription!" << std::endl;
      return false;
    }
    sleep(1);
  }

  // Global position retrieved via subscription
  Telemetry::TypeMap<TOPIC_GPS_FUSED>::type subscribeGPosition;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition broadcastGPosition;

  // Hotpoint Mission Initialize
  vehicle->missionManager->init(DJI_MISSION_TYPE::HOTPOINT, responseTimeout,
                                NULL);
  vehicle->missionManager->printInfo();

  if (vehicle->getFwVersion() != Version::M100_31)
  {
    subscribeGPosition = vehicle->subscribe->getValue<TOPIC_GPS_FUSED>();
    vehicle->missionManager->hpMission->setHotPoint(
      subscribeGPosition.longitude, subscribeGPosition.latitude, initialRadius);
  }
  else
  {
    broadcastGPosition = vehicle->broadcast->getGlobalPosition();
    vehicle->missionManager->hpMission->setHotPoint(
      broadcastGPosition.longitude, broadcastGPosition.latitude, initialRadius);
  }

  // Takeoff
  ACK::ErrorCode takeoffAck = vehicle->control->takeoff(responseTimeout);
  if (ACK::getError(takeoffAck))
  {
    ACK::getErrorCodeMessage(takeoffAck, __func__);

    if (vehicle->getFwVersion() != Version::M100_31)
    {
      teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX, responseTimeout);
    }
    return false;
  }
  else
  {
    sleep(15);
  }

  // Start
  std::cout << "Start with default rotation rate: 15 deg/s" << std::endl;
  ACK::ErrorCode startAck =
    vehicle->missionManager->hpMission->start(responseTimeout);
  if (ACK::getError(startAck))
  {
    ACK::getErrorCodeMessage(startAck, __func__);
    if (vehicle->getFwVersion() != Version::M100_31)
    {
      teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX, responseTimeout);
    }
    return false;
  }
  sleep(20);

  // Pause
  std::cout << "Pause for 5s" << std::endl;
  ACK::ErrorCode pauseAck =
    vehicle->missionManager->hpMission->pause(responseTimeout);
  if (ACK::getError(pauseAck))
  {
    ACK::getErrorCodeMessage(pauseAck, __func__);
  }
  sleep(5);

  // Resume
  std::cout << "Resume" << std::endl;
  ACK::ErrorCode resumeAck =
    vehicle->missionManager->hpMission->resume(responseTimeout);
  if (ACK::getError(resumeAck))
  {
    ACK::getErrorCodeMessage(resumeAck, __func__);
  }
  sleep(10);

  // Update radius, no ACK
  std::cout << "Update radius to 1.5x: new radius = " << 1.5 * initialRadius
            << std::endl;
  vehicle->missionManager->hpMission->updateRadius(1.5 * initialRadius);
  sleep(10);

  // Update velocity (yawRate), no ACK
  std::cout << "Update hotpoint rotation rate: new rate = 5 deg/s" << std::endl;
  HotpointMission::YawRate yawRateStruct;
  yawRateStruct.clockwise = 1;
  yawRateStruct.yawRate   = 5;
  vehicle->missionManager->hpMission->updateYawRate(yawRateStruct);
  sleep(10);

  // Stop
  std::cout << "Stop" << std::endl;
  ACK::ErrorCode stopAck =
    vehicle->missionManager->hpMission->stop(responseTimeout);

  std::cout << "land" << std::endl;
  ACK::ErrorCode landAck = vehicle->control->land(responseTimeout);
  if (ACK::getError(landAck))
  {
    ACK::getErrorCodeMessage(landAck, __func__);
  }
  else
  {
    // No error. Wait for a few seconds to land
    sleep(10);
  }

  // Clean up
  ACK::getErrorCodeMessage(startAck, __func__);
  if (vehicle->getFwVersion() != Version::M100_31)
  {
    teardownSubscription(vehicle, DEFAULT_PACKAGE_INDEX, responseTimeout);
  }

  return true;
}

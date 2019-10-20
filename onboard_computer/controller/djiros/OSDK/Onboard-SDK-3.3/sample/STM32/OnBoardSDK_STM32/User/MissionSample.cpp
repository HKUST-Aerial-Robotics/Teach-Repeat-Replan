#include "MissionSample.h"

using namespace DJI::OSDK;

extern Vehicle  vehicle;
extern Vehicle* v;

bool
setUpSubscription(DJI::OSDK::Vehicle* vehicle)
{
  // Telemetry: Subscribe to flight status and mode at freq 10 Hz
  int                  pkgIndex        = DEFAULT_PACKAGE_INDEX;
  int                  freq            = 10;
  Telemetry::TopicName topicList10Hz[] = { Telemetry::TOPIC_GPS_FUSED,
                                           Telemetry::TOPIC_ALTITUDE_FUSIONED };
  int  numTopic        = sizeof(topicList10Hz) / sizeof(topicList10Hz[0]);
  bool enableTimestamp = false;

  bool pkgStatus = v->subscribe->initPackageFromTopicList(
    pkgIndex, numTopic, topicList10Hz, enableTimestamp, freq);
  if (!(pkgStatus))
  {
    return pkgStatus;
  }

  v->subscribe->startPackage(pkgIndex);
  delay_nms(8000);

  return true;
}

bool
runWaypointMission(uint8_t numWaypoints)
{
  if (v->getFwVersion() != Version::M100_31)
  {
    if (!setUpSubscription(v))
    {
      printf("Failed to set up Subscription!\n");
      return false;
    }
  }

  // Waypoint Mission : Initialization
  WayPointInitSettings fdata;
  setWaypointInitDefaults(&fdata);
  fdata.indexNumber =
    numWaypoints + 1; // We add 1 to get the aircarft back to the start.
  float64_t increment = 0.000001;
  float32_t start_alt = 10;

  v->missionManager->init(WAYPOINT, 1, &fdata);
  delay_nms(500);

  v->missionManager->printInfo();
  printf("Initializing Waypoint Mission..\n");

  // Waypoint Mission: Create Waypoints
  std::vector<WayPointSettings> generatedWaypts =
    createWaypoints(numWaypoints, increment, start_alt);

  printf("Creating Waypoints..\n");

  // Waypoint Mission: Upload the waypoints
  uploadWaypoints(generatedWaypts);

  printf("Uploading Waypoints..\n");

  // Waypoint Mission: Start
  v->missionManager->wpMission->start();
  delay_nms(500);

  printf("Starting Waypoint Mission.\n");

  // Cleanup before return. The mission isn't done yet, but it doesn't need any
  // more input from our side.
  v->subscribe->removePackage(DEFAULT_PACKAGE_INDEX);
  delay_nms(3000);

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
createWaypoints(int numWaypoints, float64_t distanceIncrement,
                float32_t start_alt)
{
  delay_nms(8000);

  // Create Start Waypoint
  WayPointSettings start_wp;
  setWaypointDefaults(&start_wp);

  // Global position retrieved via subscription
  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type subscribeGPosition;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition broadcastGPosition;

  if (v->getFwVersion() != Version::M100_31)
  {
    subscribeGPosition = v->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
    start_wp.latitude  = subscribeGPosition.latitude;
    start_wp.longitude = subscribeGPosition.longitude;
    start_wp.altitude  = start_alt;
    printf("Waypoint created at (LLA): %f \t%f \t%f\n",
           subscribeGPosition.latitude, subscribeGPosition.longitude,
           start_alt);
  }
  else
  {
    broadcastGPosition = v->broadcast->getGlobalPosition();
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
uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list)
{
  for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
       wp != wp_list.end(); ++wp)
  {
    printf("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
           wp->longitude, wp->altitude);

    v->missionManager->wpMission->uploadIndexData(&(*wp));
  }
}

bool
runHotpointMission(int initialRadius)
{
  ACK::ErrorCode ack;

  if (v->getFwVersion() != Version::M100_31)
  {
    if (!setUpSubscription(v))
    {
      printf("Failed to set up Subscription!\n");
      return false;
    }
  }

  // Global position retrieved via subscription
  Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type subscribeGPosition;
  // Global position retrieved via broadcast
  Telemetry::GlobalPosition broadcastGPosition;

  // Hotpoint Mission Initialize
  v->missionManager->init(HOTPOINT, 1, NULL);
  v->missionManager->printInfo();

  if (v->getFwVersion() != Version::M100_31)
  {
    subscribeGPosition = v->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
    v->missionManager->hpMission->setHotPoint(
      subscribeGPosition.longitude, subscribeGPosition.latitude, initialRadius);
  }
  else
  {
    broadcastGPosition = v->broadcast->getGlobalPosition();
    v->missionManager->hpMission->setHotPoint(
      broadcastGPosition.longitude, broadcastGPosition.latitude, initialRadius);
  }

  // Takeoff
  monitoredTakeOff();

  // Start
  printf("Start with default rotation rate: 15 deg/s");

  v->missionManager->hpMission->start();
  delay_nms(500);
  /*ack = waitForAck();
  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
    ACK::ErrorCode ack =
    v->subscribe->removePackage(pkgIndex);
    if (ACK::getError(ack))
    {
      printf("Error unsubscribing; please restart the drone/FC to get "
                     "back to a clean state.\n");
    }
    return false;
  }*/

  delay_nms(4000);

  // Pause
  printf("Pause for 5s\n");
  v->missionManager->hpMission->pause();
  delay_nms(5000);
  /*ack = waitForACK();
  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }*/

  // Resume
  printf("Resume\n");
  v->missionManager->hpMission->resume();
  delay_nms(4000);
  /*ack = waitForACK();
  if (ACK::getError(ack))
  {
    ACK::getErrorCodeMessage(ack, __func__);
  }*/

  // Update radius, no ACK
  printf("Update radius to 1.5x: new radius = %0.3f\n", 1.5 * initialRadius);
  v->missionManager->hpMission->updateRadius(1.5 * initialRadius);
  delay_nms(10000);

  // Update velocity (yawRate), no ACK
  printf("Update hotpoint rotation rate: new rate = 5 deg/s\n");
  HotpointMission::YawRate yawRateStruct;
  yawRateStruct.clockwise = 1;
  yawRateStruct.yawRate   = 5;

  v->missionManager->hpMission->updateYawRate(yawRateStruct);
  delay_nms(10000);

  // Give it a bit more time to finish a circle
  delay_nms(20000);

  // Stop
  printf("Stop\n");
  v->missionManager->hpMission->stop();
  delay_nms(1000);

  printf("Land\n");

  // Free existing packages
  if (v->getFwVersion() != Version::M100_31)
  {
    v->subscribe->removePackage(DEFAULT_PACKAGE_INDEX);
    delay_nms(3000);
    /*ack = waitForACK();
    if (ACK::getError(ack))
    {
      printf("Error unsubscribing; please restart the drone/FC to get back "
                   "to a clean state.\n");
    }*/
  }

  return monitoredLanding() ? true : false;
}

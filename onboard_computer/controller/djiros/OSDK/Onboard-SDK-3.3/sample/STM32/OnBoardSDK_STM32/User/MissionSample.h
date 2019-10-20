#ifndef MISSIONSAMPLE_H
#define MISSIONSAMPLE_H

#include "BspUsart.h"
#include "FlightControlSample.h"
#include "dji_vehicle.hpp"
#include "timer.h"
#include <math.h>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

bool setUpSubscription(DJI::OSDK::Vehicle* vehicle);

bool runWaypointMission(uint8_t numWaypoints = 6);

void setWaypointDefaults(DJI::OSDK::WayPointSettings* wp);
void setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* fdata);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(
  int numWaypoints, DJI::OSDK::float64_t distanceIncrement,
  DJI::OSDK::float32_t start_alt);

std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(
  DJI::OSDK::WayPointSettings* start_data, DJI::OSDK::float64_t increment,
  int num_wp);

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings>& wp_list);

bool runHotpointMission(int initialRadius = 10);

const int DEFAULT_PACKAGE_INDEX = 1;

#endif // MISSIONSAMPLE_H

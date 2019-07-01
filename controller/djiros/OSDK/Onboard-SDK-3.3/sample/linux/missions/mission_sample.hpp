/*! @file mission_sample.hpp
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

#ifndef DJIOSDK_MISSIONSAMPLE_HPP
#define DJIOSDK_MISSIONSAMPLE_HPP

// System Includes
#include <cmath>
#include <vector>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

// Subscription not supported in Matrice 100
bool setUpSubscription(DJI::OSDK::Vehicle* vehicle, int responseTimeout);
bool teardownSubscription(DJI::OSDK::Vehicle* vehicle, const int pkgIndex,
                          int responseTimeout);

bool runWaypointMission(DJI::OSDK::Vehicle* vehicle, uint8_t numWaypoints,
                        int responseTimeout);

void setWaypointDefaults(DJI::OSDK::WayPointSettings* wp);
void setWaypointInitDefaults(DJI::OSDK::WayPointInitSettings* fdata);

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(
  DJI::OSDK::Vehicle* vehicle, int numWaypoints,
  DJI::OSDK::float64_t distanceIncrement, DJI::OSDK::float32_t start_alt);

std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(
  DJI::OSDK::WayPointSettings* start_data, DJI::OSDK::float64_t increment,
  int num_wp);

void uploadWaypoints(DJI::OSDK::Vehicle*                       vehicle,
                     std::vector<DJI::OSDK::WayPointSettings>& wp_list,
                     int                                       responseTimeout);

bool runHotpointMission(DJI::OSDK::Vehicle* vehicle, int initialRadius,
                        int responseTimeout);

const int DEFAULT_PACKAGE_INDEX = 0;

#endif // DJIOSDK_MISSIONSAMPLE_HPP

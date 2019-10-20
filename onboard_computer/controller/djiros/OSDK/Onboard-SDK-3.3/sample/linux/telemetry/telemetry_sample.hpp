/*! @file telemetry_sample.hpp
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

#ifndef DJIOSDK_TELEMETRYSAMPLE_HPP
#define DJIOSDK_TELEMETRYSAMPLE_HPP

// System Includes
#include <iostream>

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

bool subscribeToData(DJI::OSDK::Vehicle* vehiclePtr, int responseTimeout = 1);

// Broadcast data implementation for Matrice 100
bool getBroadcastData(DJI::OSDK::Vehicle* vehicle, int responseTimeout = 1);

#endif // DJIOSDK_TELEMETRYSAMPLE_HPP

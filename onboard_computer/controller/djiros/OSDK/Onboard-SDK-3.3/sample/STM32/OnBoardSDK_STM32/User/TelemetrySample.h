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

#ifndef TELEMETRYSAMPLE_H
#define TELEMETRYSAMPLE_H

#include "dji_vehicle.hpp"
#include "stdio.h"
#include "timer.h"

bool getBroadcastData();
bool subscribeToData();

#endif // TELEMETRYSAMPLE_H

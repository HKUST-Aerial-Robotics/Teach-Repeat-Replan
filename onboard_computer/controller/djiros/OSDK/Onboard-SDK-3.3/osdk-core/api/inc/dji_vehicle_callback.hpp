/** @file dji_vehicle_callback.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Type definition for new Vehicle-style callbacks
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJI_VEHICLECALLBACK_H
#define DJI_VEHICLECALLBACK_H

#include "dji_open_protocol.hpp"

namespace DJI
{
namespace OSDK
{
class Vehicle;

//! @todo move definition below to class Vehicle
//! so that we could remove this file

/*! @brief Function prototype for all callback functions used in the OSDK
 *
 * @details If you want to register a function as a callback funtion, make sure
 * it matches this prototype.
 *
 */
typedef void (*VehicleCallBack)(Vehicle* vehicle, RecvContainer recvFrame,
                                UserData userData);

/*! @brief The CallBackHandler struct allows users to encapsulate callbacks and
 * data in one struct
 *
 */
typedef struct VehicleCallBackHandler
{
  VehicleCallBack callback;
  UserData        userData;
} VehicleCallBackHandler;

} // namespace OSDK
} // namespace DJI
#endif /* DJI_VEHICLECALLBACK_H */

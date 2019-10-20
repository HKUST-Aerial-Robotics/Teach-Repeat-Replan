/** @file dji_mobile_communication.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief Implementation of DJI Mobile-Onboard SDK Communication (MOC)
 *
 *  @copyright 2016-17 DJI. All rights reserved.
 *
 */

#ifndef MOC_H
#define MOC_H

#include "dji_vehicle_callback.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declarations
class Vehicle;

/*! @brief APIs for Mobile-Onboard SDK Communication
 *
 * @details This class implements the Onboard SDK side of
 * Data Transparent Transmission functionality. You must implement APIs
 * available
 * in the Mobile SDK to have full functionality on both directions of the
 * pipeline.
 */
class MobileCommunication
{
public:
  MobileCommunication(Vehicle* vehicle = 0);
  ~MobileCommunication();

  Vehicle* getVehicle() const;
  void setVehicle(Vehicle* value);

public:
  /*!
   * @brief sending data from OSDK to MSDK
   *
   * @param data sent data
   * @param len length of data
   */
  void sendDataToMSDK(uint8_t* data, uint8_t len);
  static void getDataFromMSDKCallback(Vehicle*      vehiclePtr,
                                      RecvContainer recvFrame,
                                      UserData      userData);

public:
  VehicleCallBackHandler fromMSDKHandler;
  void setFromMSDKCallback(VehicleCallBack callback, UserData userData = 0);

private:
  Vehicle* vehicle;
};

} // OSDK
} // DJI

#endif // MOC_H

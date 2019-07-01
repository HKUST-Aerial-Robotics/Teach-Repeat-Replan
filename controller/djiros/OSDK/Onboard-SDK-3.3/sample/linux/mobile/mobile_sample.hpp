/*! @file mobile_sample.hpp
 *  @version 3.3
 *  @date Jun 05 2017
 *
 *  @brief
 *  Mobile SDK Communication API usage in a Linux environment.
 *  Shows example usage of the mobile<-->onboard SDK communication API.
 *
 *  @copyright
 *  2017 DJI. All rights reserved.
 * */

#ifndef DJIOSDK_MOBILESAMPLE_HPP
#define DJIOSDK_MOBILESAMPLE_HPP

// DJI OSDK includes
#include <dji_vehicle.hpp>

// Helpers
#include <dji_linux_helpers.hpp>

void controlAuthorityMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                    DJI::OSDK::RecvContainer recvFrame,
                                    DJI::OSDK::UserData      userData);
void actionMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                          DJI::OSDK::RecvContainer recvFrame,
                          DJI::OSDK::UserData      userData);

void parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                             DJI::OSDK::RecvContainer recvFrame,
                             DJI::OSDK::UserData      userData);

bool setupMSDKParsing(DJI::OSDK::Vehicle* vehicle);

typedef struct AckReturnToMobile
{
  uint16_t cmdID;
  uint16_t ack;
} AckReturnToMobile;

#endif // DJIOSDK_MOBILESAMPLE_HPP

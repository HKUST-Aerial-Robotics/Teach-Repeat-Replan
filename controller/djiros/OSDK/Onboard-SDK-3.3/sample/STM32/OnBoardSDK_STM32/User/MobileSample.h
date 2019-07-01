#ifndef MOBILESAMPLE_H
#define MOBILESAMPLE_H

#include <cstdint>
#include <dji_vehicle.hpp>

void controlAuthorityMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                    DJI::OSDK::RecvContainer recvFrame,
                                    DJI::OSDK::UserData      userData);
void actionMobileCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                          DJI::OSDK::RecvContainer recvFrame,
                          DJI::OSDK::UserData      userData);

void parseFromMobileCallback(DJI::OSDK::Vehicle*      vehicle,
                             DJI::OSDK::RecvContainer recvFrame,
                             DJI::OSDK::UserData      userData);

bool setupMSDKParsing();

typedef struct AckReturnToMobile
{
  uint16_t cmdID;
  uint16_t ack;
} AckReturnToMobile;

#endif // MOBILESAMPLE_H

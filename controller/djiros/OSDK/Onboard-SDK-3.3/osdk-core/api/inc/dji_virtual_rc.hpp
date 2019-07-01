/**@file dji_virtual_rc.hpp
 *  @version 3.1.7
 *  @date July 1st, 2016
 *
 *  @brief
 *  Virtual Radio Control API for DJI onboardSDK library
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */

#ifndef DJI_VIRTUAL_RC_HPP
#define DJI_VIRTUAL_RC_HPP

#include "dji_ack.hpp"
#include "dji_open_protocol.hpp"
#include "dji_type.hpp"
//#include "dji_vehicle_callback.hpp"
#include "dji_telemetry.hpp"

namespace DJI
{
namespace OSDK
{

/*! @brief VirtualRC class has all the methods to mimic the RC functionality via OSDK
 *  @details Only supported on M100
 */
class VirtualRC
{
public:
  enum CutOff
  {
    CutOff_ToLogic  = 0,
    CutOff_ToRealRC = 1
  };

public:
  VirtualRC(Vehicle* vehicle = 0);
  ~VirtualRC();

  /*! @attention Safety notes
   * You must use these methods below really carefully with following rules:
   * - Use API setControl(bool enable, CutOff cutoffType); only once in your
   * main loop;
   * - Check your control device status by useing the following method:
   *   if(getControlDevice() != CONTROL_BY_VIRTUALRC)
   *   {
   *    reset code;
   *   }
   * - @attention Your reset code must not be "setControl(true);" without any
   * status check
   *   It may cause your drone became a nut drone.
   * - @note You could quit your VRC(Virtual Remot Control) by switching your
   * remote
   *   controller out of modeF
   * - @attention Most dangerous, setControl(true); must not be called over
   * 0.5Hz or your
   *   drone will be locked in a logic-checking loop. And you cannot exit
   * VirtualRC mode by
   *   switching your remote controller's mode. Actually, in this situation your
   * drone will
   *   be a full automatically controlled by it self. It will keep flying until
   * its bettery
   *   is empty or your code make it stop.
   * - @attention If you do not know what reset code you need to write, please
   * just output
   *   your datalog and keep it empty.
   * - @attention It would be realy dangous if you keep calling sendData(); in a
   * loop with out
   *   control-losing protection. like :
   *
   *   Global:
   *   VirtualRCData myData;
   *
   *   Thread 1:
   *   while(1)
   *   {
   *    myData = myAPIToSetupDataFromGroundStation();
   *   }
   *
   *   Thread 2:
   *   while(1)
   *   {
   *    sendData(myData);
   *    msleep(200);
   *   }
   *
   *   When your drone lose signal, it will keep the recent command sent by your
   * API:
   *   myAPIToSetupDataFromGroundStation();
   *   This may result in a catastrophic crash.
   *
   *   @note API "sendData();" need to be called above 2Hz, and not greater than
   * 25hz.
   *   @note API "sendSafeModeData();" will lead your drone hover;
   *
   * */
  void setControl(bool enable, CutOff cutoffType);

  void sendData(VirtualRCData Data);

  void resetVRCData();

  void neutralVRCSticks();

  Telemetry::RC getRCData() const;

  VirtualRCData getVRCData() const;

  void setVRCData(const VirtualRCData& value);

  bool isVirtualRC() const;

  static Telemetry::RC toRCData(VirtualRCData& vData);

  static VirtualRCData toVirtualRCData(Telemetry::RC& rcData);

  Vehicle* getVehicle() const;

  void setVehicle(Vehicle* v);

private:
  Vehicle*      vehicle;
  VirtualRCData vrcData;
};

} //! namespace OSDK
} //! namespace DJI

#endif //! DJI_VIRTUAL_RC_HPP

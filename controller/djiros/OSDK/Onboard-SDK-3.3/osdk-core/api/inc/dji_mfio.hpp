/** @file dji_mfio.hpp
 *  @version 3.3
 *  @date April 2017
 *
 *  @brief
 *  MFIO API for DJI OSDK library
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#ifndef DJI_MFIO_H
#define DJI_MFIO_H

#include "dji_vehicle_callback.hpp"

namespace DJI
{
namespace OSDK
{

// Forward Declarations
class Vehicle;

/*! @brief APIs for Multi-Function Input-Output functionality
 *
 * @details This class offers control over the F-channel pins on DJI products.
 * Five modes are available through the F-channels:
 * 1. PWM Input (not supported yet)
 * 2. PWM Output
 * 3. Digital Input (GPI)
 * 4. Digital Output (GPO)
 * 5. Analog-Digital Conversion (ADC) Input
 *
 * @note You must map F-channels to SDK channels through DJI Assistant 2 to use
 * MFIO functionality.
 */
class MFIO
{
public:
  typedef enum MODE {
    MODE_PWM_OUT  = 0,
    MODE_PWM_IN   = 1,
    MODE_GPIO_OUT = 2,
    MODE_GPIO_IN  = 3,
    MODE_ADC      = 4
  } MODE;

  typedef enum CHANNEL {
    CHANNEL_0 = 0,
    CHANNEL_1 = 1,
    CHANNEL_2 = 2,
    CHANNEL_3 = 3,
    CHANNEL_4 = 4,
    CHANNEL_5 = 5,
    CHANNEL_6 = 6,
    CHANNEL_7 = 7,
  } CHANNEL;

public:
  MFIO(Vehicle* vehicle);
  ~MFIO();

  // Non-blocking API

  /*! @brief Non-blocking call for initializing an MFIO channel
   *
   * @param mode The mode (PWM, GPIO, ADC) to initialize to
   * @param channel The channel (0-7) to initialize
   * @param defaultValue The starting value [for output]
   * @param freq The frequency
   * @param fn Callback function you want called upon ACK
   * @param userData Additional data you want the callback function to have
   * access to
   */
  void config(MODE mode, CHANNEL channel, uint32_t defaultValue, uint16_t freq,
              VehicleCallBack fn = 0, UserData userData = 0);

  /*! @brief Non-blocking call for setting an MFIO value to a channel
   *
   * @param channel The channel (0-7) to set the value to
   * @param value The value you want to set
   * @param fn Callback function you want called upon ACK
   * @param data Additional data you want the callback function to have access
   * to
   */
  void setValue(CHANNEL channel, uint32_t value, VehicleCallBack fn = 0,
                UserData data = 0);

  /*! @brief Non-blocking call for getting data from an MFIO channel
   *
   * @param channel The channel (0-7) to get the value from
   * @param fn Callback function you want called upon ACK
   * @param data Additional data you want the callback function to have access
   * to
   */
  void getValue(CHANNEL channel, VehicleCallBack fn = 0, UserData data = 0);

  // Blocking API

  /*! @brief Blocking call for initializing an MFIO channel
   *
   * @param mode The mode (PWM, GPIO, ADC) to initialize to
   * @param channel The channel (0-7) to initialize
   * @param defaultValue The starting value [for output]
   * @param freq The frequency
   * @param wait_timeout Time(in s) you want the function to wait for an ACK
   * @return ACK::ErrorCode struct containing the ACK and metadata
   */
  ACK::ErrorCode config(MODE mode, CHANNEL channel, uint32_t defaultValue,
                        uint16_t freq, int wait_timeout);

  /*! Blocking call for setting an MFIO value to a channel
   *
   * @param channel The channel (0-7) to set the value to
   * @param value The value you want to set
   * @param wait_timeout Time(in s) you want the function to wait for an ACK
   * @return ACK::ErrorCode struct containing the ACK and metadata
   */
  ACK::ErrorCode setValue(CHANNEL channel, uint32_t value, int wait_timeout);

  /*! Blocking call for getting data from an MFIO channel
   *
   * @param channel The channel (0-7) to get the value from
   * @param wait_timeout Time(in s) you want the function to wait for an ACK
   * @return ACK::ErrorCode struct containing the ACK and metadata
   */
  ACK::MFIOGet getValue(CHANNEL channel, int wait_timeout);

private:
  static void initCallback(RecvContainer recvFrame, UserData data);
  static void setValueCallback(RecvContainer recvFrame, UserData data);
  static void getValueCallback(RecvContainer recvFrame, UserData data);

private:
  Vehicle* vehicle;

  uint8_t channelUsage;

private:
#pragma pack(1)
  typedef struct InitData
  {
    uint8_t  channel;
    uint8_t  mode;
    uint32_t value;
    uint16_t freq;
  } InitData; // pack(1)

  typedef struct SetData
  {
    uint8_t  channel;
    uint32_t value;
  } SetData; // pack(1)

  typedef uint32_t GetData;

  typedef struct GetResult
  {
    uint8_t  reserved;
    uint32_t value;
  } GetResult; // pack(1)

#pragma pack()
}; // class MFIO

} // OSDK
} // DJI

#endif // DJI_MFIO_H

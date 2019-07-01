/** @file dji_circular_buffer.hpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Circular buffer class for the DJI OSDK
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_open_protocol.hpp"
#include "dji_vehicle_callback.hpp"
#include <cstdlib>

#if STM32
#include <stdlib.h>
#endif

namespace DJI
{
namespace OSDK
{

/*! @brief Circular buffer for callback function storage
 *
 * @details This buffer is not currently generic, so do not use it for any other
 * purpose.
 */
class CircularBuffer
{
public:
  int cbPush(CircularBuffer* CBuffer, VehicleCallBackHandler data,
             RecvContainer data2);
  int cbPop(CircularBuffer* CBuffer, VehicleCallBackHandler* data,
            RecvContainer* data2);
  CircularBuffer();
  ~CircularBuffer();
  int head;
  int tail;

private:
  VehicleCallBackHandler* buffer;
  RecvContainer*          buffer2;
  const int               maxLen;
}; // class CircularBuffer

} // namespace OSDK
} // namespace DJI

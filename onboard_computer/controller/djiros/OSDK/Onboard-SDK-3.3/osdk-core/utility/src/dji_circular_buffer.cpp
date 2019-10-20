/** @file dji_circular_buffer.cpp
 *  @version 3.3
 *  @date Jun 2017
 *
 *  @brief Circular buffer class for the DJI OSDK
 *
 *  @copyright 2017 DJI. All rights reserved.
 *
 */

#include "dji_circular_buffer.hpp"

using namespace DJI;
using namespace DJI::OSDK;

CircularBuffer::CircularBuffer()
  : maxLen(5000)
{
  buffer =
    (VehicleCallBackHandler*)malloc(5000 * sizeof(VehicleCallBackHandler));
  buffer2 = (RecvContainer*)malloc(5000 * sizeof(RecvContainer));
  head    = 0;
  tail    = 0;
}

int
CircularBuffer::cbPush(CircularBuffer*                   CBuffer,
                       DJI::OSDK::VehicleCallBackHandler cbData,
                       RecvContainer                     recvData)
{
  int next = head + 1;
  if (next >= maxLen)
  {
    next = 0;
  }
  //! Circular buffer is full, pop the old value and discard.
  if (next == tail)
  {
    CBuffer->cbPop(CBuffer, &cbData, &recvData);
    DSTATUS("Warning: Circular Buffer Full. Discarded Callback from Tail \n");
  }
  buffer2[head] = recvData;
  buffer[head]  = cbData;
  head          = next;
  return 0;
}

int
CircularBuffer::cbPop(CircularBuffer*                    CBuffer,
                      DJI::OSDK::VehicleCallBackHandler* cbData,
                      RecvContainer*                     recvData)
{
  if (head == tail)
  {
    DSTATUS("Circular Buffer empty \n");
    return -1;
  }
  *cbData   = buffer[tail];
  *recvData = buffer2[tail];

  //! Clear data
  memset(&buffer[tail], 0, sizeof(VehicleCallBackHandler));
  memset(&buffer2[tail], 0, sizeof(RecvContainer));

  int next = tail + 1;
  if (next >= maxLen)
    next = 0;
  tail   = next;
  return 0;
}

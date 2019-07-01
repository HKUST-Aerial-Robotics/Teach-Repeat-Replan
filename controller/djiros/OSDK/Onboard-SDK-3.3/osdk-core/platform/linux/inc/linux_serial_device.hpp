/*! @file linux_serial_device.hpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Serial device hardware implementation for *NIX machines.
 *
 *  @details
 *  This is a generic POSIX-compatible serial device implementation.
 *
 *  The Vehicle class handles Serial drivers, so you do not need to
 *  instantiate drivers in your code.
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#ifndef LINUXSERIALDEVICE_H
#define LINUXSERIALDEVICE_H

#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "dji_hard_driver.hpp"

namespace DJI
{

namespace OSDK
{

/*! @brief POSIX-Compatible Serial Driver for *NIX platforms
 *
 */
class LinuxSerialDevice : public HardDriver
{
public:
  static const int BUFFER_SIZE = 2048;

public:
  LinuxSerialDevice(const char* device, uint32_t baudrate);
  ~LinuxSerialDevice();

  void init();
  bool getDeviceStatus();

  void setBaudrate(uint32_t baudrate);
  void setDevice(const char* device);

  //! Public interfaces to private functions. Use these functions to validate
  //! your serial connection
  int checkBaudRate(uint8_t (&buf)[BUFFER_SIZE])
  {
    _checkBaudRate(buf);
  }
  int setSerialPureTimedRead();
  int unsetSerialPureTimedRead();
  int serialRead(uint8_t* buf, int len);

  //! Start of DJI_HardDriver virtual function implementations
  size_t send(const uint8_t* buf, size_t len);
  size_t readall(uint8_t* buf, size_t maxlen);

  //! Implemented here because ..
  DJI::OSDK::time_ms getTimeStamp();

  void delay_nms(uint16_t time)
  {
    ;
  }

private:
  const char* m_device;
  uint32_t    m_baudrate;

  int    m_serial_fd;
  fd_set m_serial_fd_set;
  bool   deviceStatus;

  bool _serialOpen(const char* dev);
  bool _serialClose();
  bool _serialFlush();
  bool _serialConfig(int baudrate, char data_bits, char parity_bits,
                     char stop_bits, bool testForData = false);

  int _serialStart(const char* dev_name, int baud_rate);
  int _serialWrite(const uint8_t* buf, int len);
  int _serialRead(uint8_t* buf, int len);

  int _checkBaudRate(uint8_t (&buf)[BUFFER_SIZE]);
};
}
}

#endif // LINUXSERIALDEVICE_H

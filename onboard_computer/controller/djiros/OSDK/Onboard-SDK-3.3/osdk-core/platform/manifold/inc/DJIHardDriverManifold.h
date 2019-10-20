/*! @file DJIHardDriverManifold.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Old serial device hardware abstraction for DJI Onboard SDK command line example.
 *
 *  @warning Deprecated. Please use linux_serial_device.hpp/.cpp for the most current implementation.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef __DJIHARDDRIVERMANIFOLD_H__
#define __DJIHARDDRIVERMANIFOLD_H__

#include <DJI_HardDriver.h>
#include <dji_type.hpp>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

namespace DJI
{

namespace onboardSDK
{

class HardDriverManifold : public HardDriver
{

public:
  HardDriverManifold(std::string device, uint32_t baudrate);
  ~HardDriverManifold();

  void init();

  /**
   * @brief Implement a USB hand-shaking protocol for SDK
   */
  void usbHandshake(std::string device);

  void setBaudrate(uint32_t baudrate);
  void setDevice(std::string device);

  DJI::OSDK::time_ms getTimeStamp();

  size_t send(const uint8_t* buf, size_t len);
  size_t readall(uint8_t* buf, size_t maxlen);

  void lockMemory();
  void freeMemory();
  void lockMSG();
  void freeMSG();

private:
  std::string     m_device;
  uint32_t        m_baudrate;
  pthread_mutex_t m_memLock;
  pthread_mutex_t m_msgLock;

  int    m_serial_fd;
  fd_set m_serial_fd_set;

  bool _serialOpen(const char* dev);
  bool _serialClose();
  bool _serialFlush();
  bool _serialConfig(int baudrate, char data_bits, char parity_bits,
                     char stop_bits);

  int _serialStart(const char* dev_name, int baud_rate);
  int _serialWrite(const uint8_t* buf, int len);
  int _serialRead(uint8_t* buf, int len);
};
}
}

#endif

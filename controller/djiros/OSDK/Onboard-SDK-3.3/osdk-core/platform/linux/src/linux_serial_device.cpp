/*! @file linux_serial_device.cpp
 *  @version 3.3
 *  @date Jun 15 2017
 *
 *  @brief
 *  Serial device hardware implementation for Linux machines.
 *  This is a generic Linux serial device implementation.
 *
 *  Use this in your own Linux-based DJI Onboard SDK implementations.
 *
 *  @copyright
 *  2016-17 DJI. All rights reserved.
 * */

#include "linux_serial_device.hpp"
#include <algorithm>
#include <iterator>

using namespace DJI::OSDK;

/*! Implementing inherited functions from abstract class DJI_HardDriver */

LinuxSerialDevice::LinuxSerialDevice(const char* device, uint32_t baudrate)
{
  m_device   = device;
  m_baudrate = baudrate;
}

LinuxSerialDevice::~LinuxSerialDevice()
{
  _serialClose();
}

void
LinuxSerialDevice::init()
{
  DSTATUS("Attempting to open device %s with baudrate %u...\n", m_device,
          m_baudrate);
  if (_serialStart(m_device, m_baudrate) < 0)
  {
    _serialClose();
    DERROR("...Failed to start serial\n");
    deviceStatus = false;
  }
  else
  {
    DSTATUS("...Serial started successfully.\n");
    deviceStatus = true;
  }
}

bool
LinuxSerialDevice::getDeviceStatus()
{
  return deviceStatus;
}

DJI::OSDK::time_ms
LinuxSerialDevice::getTimeStamp()
{
  return (uint32_t)time(NULL);
}

size_t
LinuxSerialDevice::send(const uint8_t* buf, size_t len)
{
  return _serialWrite(buf, len);
}

size_t
LinuxSerialDevice::readall(uint8_t* buf, size_t maxlen)
{
  return _serialRead(buf, maxlen);
}

/*! Implement functions specific to this hardware driver */

/****
  The next few functions set serial port I/O parameters and implement serial R/W
functions.
  Implement termios-based serial i/o.
****/
void
LinuxSerialDevice::setBaudrate(uint32_t baudrate)
{
  m_baudrate = baudrate;
}

void
LinuxSerialDevice::setDevice(const char* device)
{
  m_device = device;
}

int
LinuxSerialDevice::setSerialPureTimedRead()
{
  return _serialConfig(m_baudrate, 8, 'N', 1, true);
}

int
LinuxSerialDevice::unsetSerialPureTimedRead()
{
  return _serialConfig(m_baudrate, 8, 'N', 1, false);
}

int
LinuxSerialDevice::serialRead(uint8_t* buf, int len)
{
  return _serialRead(buf, len);
}

int LinuxSerialDevice::_checkBaudRate(uint8_t (&buf)[BUFFER_SIZE])
{
  int lengthForCheck   = 200;
  int timeoutInSeconds = 2;

  struct timespec curTime, absTimeout;
  // Use clock_gettime instead of getttimeofday for compatibility with POSIX
  // APIs
  clock_gettime(CLOCK_REALTIME, &curTime);
  absTimeout.tv_sec  = curTime.tv_sec + timeoutInSeconds;
  absTimeout.tv_nsec = curTime.tv_nsec;

  int receivedBytes = _serialRead(buf, lengthForCheck);

  while (curTime.tv_sec < absTimeout.tv_sec)
  {
    if (receivedBytes < lengthForCheck)
      receivedBytes +=
        _serialRead(buf + receivedBytes, lengthForCheck - receivedBytes);
    else
      break;

    clock_gettime(CLOCK_REALTIME, &curTime);
  }
  if (curTime.tv_sec >= absTimeout.tv_sec)
    return -1;
  if (std::end(buf) == std::find(std::begin(buf), std::end(buf), 0xAA))
    return -2;

  return 1;
}

bool
LinuxSerialDevice::_serialOpen(const char* dev)
{
#ifdef __arm__
  m_serial_fd = open(dev, O_RDWR | O_NONBLOCK);
#elif __x86_64__
  m_serial_fd = open(dev, O_RDWR | O_NOCTTY);
#else
  m_serial_fd = open(dev, O_RDWR | O_NOCTTY);
#endif
  if (m_serial_fd < 0)
  {
    DERROR("cannot open device %s\n", dev);
    return false;
  }
  return true;
}

bool
LinuxSerialDevice::_serialClose()
{
  close(m_serial_fd);
  m_serial_fd = -1;
  return true;
}

bool
LinuxSerialDevice::_serialFlush()
{
  if (m_serial_fd < 0)
  {
    DERROR("flushing fail because no device is opened\n");
    return false;
  }
  else
  {
    tcflush(m_serial_fd, TCIFLUSH);
    return true;
  }
}

bool
LinuxSerialDevice::_serialConfig(int baudrate, char data_bits, char parity_bits,
                                 char stop_bits, bool testForData)
{
  int st_baud[] = { B4800,  B9600,   B19200,  B38400,
                    B57600, B115200, B230400, B921600, B1000000};
  int std_rate[] = { 4800,   9600,   19200,   38400,   57600,  115200,
                     230400, 921600, 1000000, 1152000, 3000000 };

  int            i, j;
  struct termios newtio, oldtio;
  /* save current port parameter */
  if (tcgetattr(m_serial_fd, &oldtio) != 0)
  {
    DERROR("fail to save current port\n");
    return false;
  }
  memset(&newtio, 0, sizeof(newtio));

  /* config the size of char */
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  /* config data bit */
  switch (data_bits)
  {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
  }
  /* config the parity bit */
  switch (parity_bits)
  {
    /* odd */
    case 'O':
    case 'o':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      break;
    /* even */
    case 'E':
    case 'e':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      break;
    /* none */
    case 'N':
    case 'n':
      newtio.c_cflag &= ~PARENB;
      break;
  }
  /* config baudrate */
  j = sizeof(std_rate) / 4;
  for (i = 0; i < j; ++i)
  {
    if (std_rate[i] == baudrate)
    {
      /* set standard baudrate */
      cfsetispeed(&newtio, st_baud[i]);
      cfsetospeed(&newtio, st_baud[i]);
      break;
    }
  }
  /* config stop bit */
  if (stop_bits == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (stop_bits == 2)
    newtio.c_cflag |= CSTOPB;

/* config waiting time & min number of char */
//! If you just want to see if there is data on the line, put the serial config
//! in an unconditional timeout state
#if __x86_64__
  if (testForData)
  {
    newtio.c_cc[VTIME] = 8;
    newtio.c_cc[VMIN]  = 0;
  }
  else
  {
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN]  = 18;
  }
#endif
  /* using the raw data mode */
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  newtio.c_oflag &= ~OPOST;

  /* flush the hardware fifo */
  tcflush(m_serial_fd, TCIFLUSH);

  /* activite the configuration */
  if ((tcsetattr(m_serial_fd, TCSANOW, &newtio)) != 0)
  {
    DERROR("failed to activate serial configuration\n");
    return false;
  }
  return true;
}

int
LinuxSerialDevice::_serialStart(const char* dev_name, int baud_rate)
{
  const char* ptemp;
  if (dev_name == NULL)
  {
    ptemp = "/dev/ttyUSB0";
  }
  else
  {
    ptemp = dev_name;
  }
  if (true == _serialOpen(ptemp) && true == _serialConfig(baud_rate, 8, 'N', 1))
  {

    FD_ZERO(&m_serial_fd_set);
    FD_SET(m_serial_fd, &m_serial_fd_set);
    return m_serial_fd;
  }
  return -1;
}

int
LinuxSerialDevice::_serialWrite(const uint8_t* buf, int len)
{
  return write(m_serial_fd, buf, len);
}

//! Current _serialRead behavior: Wait for 500 ms between characters till 18
//! char, read 18 characters if data available & return
//! 500 ms: long timeout to make sure that if we query the input buffer in the
//! middle of a packet we still get the full packet
//! 18 char: len of most ACK packets.
int
LinuxSerialDevice::_serialRead(uint8_t* buf, int len)
{
  int ret = -1;

  if (NULL == buf)
  {
    return -1;
  }
  else
  {
    ret = read(m_serial_fd, buf, len);
    return ret;
  }
}

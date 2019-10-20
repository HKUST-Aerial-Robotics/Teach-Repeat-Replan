/*! @file DJIHardDriverManifold.cpp
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Serial device hardware abstraction for DJI Onboard SDK command line example.
 *  Note that this is a generic Linux serial device abstraction even though it
 * is called HardDriverManifold.
 *  This may be changed in a future release.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#include "DJIHardDriverManifold.h"

using namespace DJI::onboardSDK;

HardDriverManifold::HardDriverManifold(std::string device, uint32_t baudrate)
{
  m_device   = device;
  m_baudrate = baudrate;
  m_memLock  = PTHREAD_MUTEX_INITIALIZER;
  m_msgLock  = PTHREAD_MUTEX_INITIALIZER;
}

HardDriverManifold::~HardDriverManifold()
{
  _serialClose();
}

void
HardDriverManifold::init()
{
  DSTATUS( "going to open device %s with baudrate %u...\n",
          m_device.c_str(), m_baudrate);
  if (_serialStart(m_device.c_str(), m_baudrate) < 0)
  {
    _serialClose();
    DERROR( "...fail to start serial\n");
  }
  else
  {
    DSTATUS( "...succeed to start serial\n");
  }
}

/**
 * @brief Implement a USB hand-shaking protocol for SDK
 */
void
HardDriverManifold::usbHandshake(std::string device)
{
  _serialStart(device.c_str(), 38400);
  _serialStart(device.c_str(), 19200);
  _serialStart(device.c_str(), 38400);
  _serialStart(device.c_str(), 19200);
}

void
HardDriverManifold::setBaudrate(uint32_t baudrate)
{
  m_baudrate = baudrate;
}

void
HardDriverManifold::setDevice(std::string device)
{
  m_device = device;
}

DJI::OSDK::time_ms
HardDriverManifold::getTimeStamp()
{
  return (uint32_t)time(NULL);
}

size_t
HardDriverManifold::send(const uint8_t* buf, size_t len)
{
  return _serialWrite(buf, len);
}

size_t
HardDriverManifold::readall(uint8_t* buf, size_t maxlen)
{
  return _serialRead(buf, maxlen);
}

void
HardDriverManifold::lockMemory()
{
  pthread_mutex_lock(&m_memLock);
}

void
HardDriverManifold::freeMemory()
{
  pthread_mutex_unlock(&m_memLock);
}

void
HardDriverManifold::lockMSG()
{
  pthread_mutex_lock(&m_msgLock);
}

void
HardDriverManifold::freeMSG()
{
  pthread_mutex_unlock(&m_msgLock);
}

bool
HardDriverManifold::_serialOpen(const char* dev)
{
  m_serial_fd = open(dev, O_RDWR | O_NOCTTY);
  if (m_serial_fd < 0)
  {
    DERROR( "cannot open device %s\n", dev);
    return false;
  }
  return true;
}

bool
HardDriverManifold::_serialClose()
{
  close(m_serial_fd);
  m_serial_fd = -1;
  return true;
}

bool
HardDriverManifold::_serialFlush()
{
  if (m_serial_fd < 0)
  {
    DERROR( "flushing fail because no device is opened\n");
    return false;
  }
  else
  {
    tcflush(m_serial_fd, TCIFLUSH);
    return true;
  }
}

bool
HardDriverManifold::_serialConfig(int baudrate, char data_bits,
                                  char parity_bits, char stop_bits)
{
  int st_baud[]  = { B4800, B9600, B19200, B38400, B57600, B115200, B230400 };
  int std_rate[] = {
    4800, 9600, 19200, 38400, 57600, 115200, 230400, 1000000, 1152000, 3000000,
  };

  int            i, j;
  struct termios newtio, oldtio;
  /* save current port parameter */
  if (tcgetattr(m_serial_fd, &oldtio) != 0)
  {
    DERROR( "fail to save current port\n");
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
  newtio.c_cc[VTIME] = 5;
  newtio.c_cc[VMIN]  = 18;

  /* using the raw data mode */
  newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  newtio.c_oflag &= ~OPOST;

  /* flush the hardware fifo */
  tcflush(m_serial_fd, TCIFLUSH);

  /* activite the configuration */
  if ((tcsetattr(m_serial_fd, TCSANOW, &newtio)) != 0)
  {
    DERROR( "fail to active configuration\n");
    return false;
  }
  return true;
}

int
HardDriverManifold::_serialStart(const char* dev_name, int baud_rate)
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
HardDriverManifold::_serialWrite(const uint8_t* buf, int len)
{
  return write(m_serial_fd, buf, len);
}

int
HardDriverManifold::_serialRead(uint8_t* buf, int len)
{
  int saved = 0;
  int ret   = -1;

  if (NULL == buf)
  {
    return -1;
  }
  else
  {
    ret = read(m_serial_fd, buf + saved, len - saved);
    if (ret > 0)
      saved += ret;

    return saved;
  }
}

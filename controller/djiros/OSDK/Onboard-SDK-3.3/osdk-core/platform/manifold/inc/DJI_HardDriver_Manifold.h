/*! @file DJI_HardDriver_Manifold.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Old serial device hardware abstraction for DJI Onboard SDK ROS.
 *
 *  @warning Deprecated. Please use linux_serial_device.hpp/.cpp for the most current implementation.
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */


#ifndef __DJI_HARDDRIVER_MANIFOLD_H__
#define __DJI_HARDDRIVER_MANIFOLD_H__

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

#define BUFFER_SIZE 1024

namespace DJI
{

namespace onboardSDK
{

class HardDriver_Manifold : public HardDriver
{

public:
  HardDriver_Manifold(std::string device, uint32_t baudrate)
  {
    m_device   = device;
    m_baudrate = baudrate;
    m_memLock  = PTHREAD_MUTEX_INITIALIZER;
    m_msgLock  = PTHREAD_MUTEX_INITIALIZER;
  }

  ~HardDriver_Manifold()
  {
    _serialClose();
  }

  void init()
  {
    DSTATUS( "Open serial device %s with baudrate %u...\n",
            m_device.c_str(), m_baudrate);
    if (_serialStart(m_device.c_str(), m_baudrate) < 0)
    {
      _serialClose();
      DERROR( "Failed to start serial device\n");
      deviceStatus = false;
    }

    uint8_t buf[BUFFER_SIZE];
    usleep(5000);

    //! Check if serial connection valid
    if (_serialRead(buf, BUFFER_SIZE) > 0)
    {
      DSTATUS( "Succeeded to read from serial device\n");
      deviceStatus = true;
      return;
    }

    DERROR( "Failed to read from serial device\n");
    deviceStatus = false;
  }

  /**
   * @brief Implement a USB hand-shaking protocol for SDK
   */
  void usbHandshake(std::string device)
  {
    _serialStart(device.c_str(), 38400);
    _serialStart(device.c_str(), 19200);
    _serialStart(device.c_str(), 38400);
    _serialStart(device.c_str(), 19200);
  }

  void setBaudrate(uint32_t baudrate)
  {
    m_baudrate = baudrate;
  }

  void setDevice(std::string device)
  {
    m_device = device;
  }

  bool getDevieStatus()
  {
    return deviceStatus;
  }

  time_ms getTimeStamp()
  {
#ifdef __MACH__
    struct timeval now;
    gettimeofday(&now, NULL);
    return (uint64_t)now.tv_sec * 1000 + (uint64_t)(now.tv_usec / 1.0e3);
#else
    struct timespec time;
    clock_gettime(CLOCK_REALTIME, &time);
    return (uint64_t)time.tv_sec * 1000 + (uint64_t)(time.tv_nsec / 1.0e6);
#endif
  }

  size_t send(const uint8_t* buf, size_t len)
  {
    return _serialWrite(buf, len);
  }

  size_t readall(uint8_t* buf, size_t maxlen)
  {
    return _serialRead(buf, maxlen);
  }

  void lockMemory()
  {
    pthread_mutex_lock(&m_memLock);
  }

  void freeMemory()
  {
    pthread_mutex_unlock(&m_memLock);
  }

  void lockMSG()
  {
    pthread_mutex_lock(&m_msgLock);
  }

  void freeMSG()
  {
    pthread_mutex_unlock(&m_msgLock);
  }

private:
  std::string     m_device;
  uint32_t        m_baudrate;
  pthread_mutex_t m_memLock;
  pthread_mutex_t m_msgLock;

  int    m_serial_fd;
  fd_set m_serial_fd_set;

  bool deviceStatus;

  bool _serialOpen(const char* dev)
  {
    // notice: use O_NONBLOCK to raise the frequency that read data from buffer
    m_serial_fd = open(dev, O_RDWR | O_NONBLOCK);
    if (m_serial_fd < 0)
    {
      DERROR( "Failed to open serial device %s\n", dev);
      return false;
    }
    return true;
  }

  bool _serialClose()
  {
    close(m_serial_fd);
    m_serial_fd = -1;
    return true;
  }

  bool _serialFlush()
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

  bool _serialConfig(int baudrate, char data_bits, char parity_bits,
                     char stop_bits)
  {
    int st_baud[] = { B4800,  B9600,   B19200,  B38400,
                      B57600, B115200, B230400, B921600 };
    int std_rate[] = {
      4800,   9600,   19200,   38400,   57600,   115200,
      230400, 921600, 1000000, 1152000, 3000000,
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
    newtio.c_cc[VTIME] = 1;
    newtio.c_cc[VMIN]  = 1;

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

  int _serialStart(const char* dev_name, int baud_rate)
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
    if (true == _serialOpen(ptemp) &&
        true == _serialConfig(baud_rate, 8, 'N', 1))
    {

      FD_ZERO(&m_serial_fd_set);
      FD_SET(m_serial_fd, &m_serial_fd_set);
      return m_serial_fd;
    }
    return -1;
  }

  int _serialWrite(const uint8_t* buf, int len)
  {
    return write(m_serial_fd, buf, len);
  }

  int _serialRead(uint8_t* buf, int len)
  {
    int saved = 0;
    int ret   = -1;

    if (NULL == buf)
    {
      return -1;
    }
    else
    {
      for (; saved < len;)
      {
        ret = read(m_serial_fd, buf + saved, len - saved);
        if (ret > 0)
          saved += ret;
        else
          break;
      }
      return saved;
    }
  }
};
}
}

#endif

#include <QDateTime>
#include <QDebug>
#include <QScrollBar>
#include <QThread>
#include <qt_serial_device.hpp>

using namespace DJI::OSDK;

QHardDriver::QHardDriver()
{
  port     = 0;
  baudrate = 230400;
}
QTextBrowser*
QHardDriver::getDisplay() const
{
  return display;
}

void
QHardDriver::setDisplay(QTextBrowser* value)
{
  display = value;
}

QHardDriver::QHardDriver(QObject* parent, const char* portName, int baudrate)
  : QObject(parent)
{
  this->portName = QString(portName);
  this->baudrate = baudrate;
  display        = 0;
}

QHardDriver::QHardDriver(QObject* parent, QSerialPort* extSerialPort)
{
  this->port     = extSerialPort;
  this->baudrate = extSerialPort->baudRate();
  display        = 0;
}

void
QHardDriver::init()
{
  port = new QSerialPort(QString(portName));
  if (port != 0)
  {
    if (port->isOpen())
      port->close();
    port->setBaudRate(baudrate);
    port->setParity(QSerialPort::NoParity);
    port->setDataBits(QSerialPort::Data8);
    port->setStopBits(QSerialPort::OneStop);
    port->setFlowControl(QSerialPort::NoFlowControl);
    if (port->open(QIODevice::ReadWrite))
    {
      DSTATUS("port %s open success", port->portName().toLocal8Bit().data());

      DSTATUS("Read buf size: %d", port->readBufferSize());
    }
    else
    {
      DERROR("fail to open port %s", port->portName().toLocal8Bit().data());
    }
    DSTATUS("BaudRate: %d", port->baudRate());
  }
}

time_ms
QHardDriver::getTimeStamp()
{
  return QDateTime::currentMSecsSinceEpoch();
}

size_t
QHardDriver::send(const uint8_t* buf, size_t len)
{
  sendLock.lock();
  size_t sent = 0;
  if (port != 0)
  {
    //    if (port->isOpen())
    while (sent != len)
    {
      sent += port->write(reinterpret_cast<const char*>(buf + sent), len);
      port->waitForBytesWritten(2);
    }
    sendLock.unlock();
    return sent;
  }
  else
  {
    sendLock.unlock();
    return 0;
  }
  sendLock.unlock();
  return sent;
}

size_t
QHardDriver::readall(uint8_t* buf, size_t maxlen)
{
  size_t ans = 0;
  if (port != 0)
  {
    if (port->isOpen())
    {
      if (port->bytesAvailable() > 0)
      {
        QThread::usleep(10);
        ans = port->read(reinterpret_cast<char*>(buf), maxlen);
        // bufferLock.unlock();
      }
    }
  }
  return ans;
}

// void
// QHardDriver::displayLog(const char* buf)
//{
//  if (buf)
//    qDebug("%s", buf);
//  else
//  {
//    if (display)
//    {
//      bufferLock.lock();
//      QString data = QString(DJI::OSDK::buffer);
//      size_t  len  = data.length();
//      if (len < DJI::OSDK::bufsize)
//        display->append(data);
//      bufferLock.unlock();
//      display->verticalScrollBar()->setValue(
//        display->verticalScrollBar()->maximum());
//    }
//    else
//    {
//      bufferLock.lock();
//      qDebug("%s", DJI::OSDK::buffer);
//      bufferLock.unlock();
//    }
//  }
//}

void
QHardDriver::setBaudrate(int value)
{
  baudrate = value;
}

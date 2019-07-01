/*! @file DJI_HardDriver_Qt.h
 *  @version 3.1.7
 *  @date Jul 01 2016
 *
 *  @brief
 *  Serial device hardware abstraction for DJI Onboard SDK Qt example.
 *
 *  @note New Qt sample coming soon!
 *
 *  @copyright
 *  2016 DJI. All rights reserved.
 * */

#ifndef QONBOARDSDK_H
#define QONBOARDSDK_H

#include "dji_hard_driver.hpp"

#include <QComboBox>
#include <QItemDelegate>
#include <QMutex>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTextBrowser>

class QHardDriver : public QObject, public DJI::OSDK::HardDriver
{
  Q_OBJECT

public:
  explicit QHardDriver(QObject* parent, const char* portName = 0,
                       int baudrate = 230400);
  QHardDriver(QObject* parent = 0, QSerialPort* extSerialPort = 0);
  ~QHardDriver()
  {
  }

  DJI::OSDK::time_ms getTimeStamp();
  size_t send(const uint8_t* buf, size_t len);
  size_t readall(uint8_t* buf, size_t maxlen);

  void setBaudrate(int value);

  QTextBrowser* getDisplay() const;
  void setDisplay(QTextBrowser* value);

  QSerialPort* port;

private:
  QHardDriver();

public slots:
  void init();

private:
  int           baudrate;
  QMutex        memory;
  QMutex        msg;
  QMutex        sendLock;
  QMutex        bufferLock;
  QTextBrowser* display;
  QString       portName;
};

#endif // QONBOARDSDK_H

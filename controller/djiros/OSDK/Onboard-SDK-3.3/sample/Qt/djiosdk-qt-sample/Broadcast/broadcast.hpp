#ifndef BROADCAST_H
#define BROADCAST_H

#include <QTimer>
#include <QWidget>
#include <dji_vehicle.hpp>

namespace Ui
{
class Broadcast;
}

class Broadcast : public QWidget
{
  Q_OBJECT

public:
  explicit Broadcast(QWidget* parent = 0, DJI::OSDK::Vehicle* vehicle = 0);
  ~Broadcast();

private:
  void updateFlightAcc();
  void updateFlightPal();
  void updateFlightMagnet();
  void updateFlightVelocity();
  void updateFlightPosition();
  void updateFlightQuaternion();
  void updateVirturalRCData();
  void updateCameraPitch();
  void updateCameraRoll();
  void updateCameraYaw();
  void updateGPS();
  void updateRTK();
  void updateTime();
  void updateCapacity();
  void updateFlightStatus();
  void updateControlDevice();
  void manageDisplayEnabling(uint8_t* curFreqSettings);

private slots:
  void updateDisplay();

  void on_pushButtonBroadcastFreqSet_clicked();

private:
  Ui::Broadcast*      ui;
  DJI::OSDK::Vehicle* vehicle;
  QTimer*             broadcastTimer;
};

#endif // BROADCAST_H

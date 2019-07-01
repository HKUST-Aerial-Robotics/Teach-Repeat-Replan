#ifndef CONTROLPANNEL_H
#define CONTROLPANNEL_H

#include "dji_vehicle.hpp"
#include <QAbstractButton>
#include <QWidget>

namespace Ui
{
class FlightControlPanel;
}

class FlightControlPanel : public QWidget
{
  Q_OBJECT

public:
  explicit FlightControlPanel(QWidget*            parent  = 0,
                              DJI::OSDK::Vehicle* vehicle = 0);
  ~FlightControlPanel();

  static void actionCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                             DJI::OSDK::RecvContainer recvFrame,
                             DJI::OSDK::UserData      userData);

private:
  void handleXYRanges(double minXY, double maxXY);
  void handleZRanges(double minZ, double maxZ);
  void handleYawRanges(double minYaw, double maxYaw);

private slots:
  void on_btn_flight_runCommand_clicked();

  void on_buttonGroupHorizontal_buttonClicked(QAbstractButton* button);
  void on_buttonGroupVertical_buttonClicked(QAbstractButton* button);
  void on_buttonGroupYaw_buttonClicked(QAbstractButton* button);
  void on_buttonGroupFrame_buttonClicked(QAbstractButton* button);
  void on_buttonGroupStable_buttonClicked(QAbstractButton* button);

  void on_xControlSpinBox_valueChanged(double arg1);
  void on_xControlSlider_valueChanged(int value);

  void on_yControlSpinBox_valueChanged(double arg1);
  void on_yControlSlider_valueChanged(int value);

  void on_zControlSpinBox_valueChanged(double arg1);
  void on_zControlSlider_valueChanged(int value);

  void on_yawControlSpinBox_valueChanged(double arg1);
  void on_yawControlSlider_valueChanged(int value);

  void on_btn_flight_send_clicked();

  void commandStatusChanged(QString);

  void on_cb_command_activated(const QString& arg1);

signals:
  void changeCommandStatus(QString textToDisplay);

private:
  Ui::FlightControlPanel*      ui;
  DJI::OSDK::Vehicle*          vehicle;
  DJI::OSDK::Control::CtrlData command;
};

#endif // CONTROLPANNEL_H

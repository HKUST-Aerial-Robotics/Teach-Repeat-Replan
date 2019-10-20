#ifndef QGIMBAL_H
#define QGIMBAL_H

#include <QAbstractButton>
#include <QWidget>
#include <dji_vehicle.hpp>

namespace Ui
{
class CameraGimbalControl;
}

class CameraGimbalControl : public QWidget
{
  Q_OBJECT

public:
  explicit CameraGimbalControl(QWidget*            parent  = 0,
                               DJI::OSDK::Vehicle* vehicle = 0);
  ~CameraGimbalControl();

private:
  void handleRPYRanges(QString modeString);

private slots:
  void on_rollCheckBox_stateChanged(int arg1);
  void on_pitchCheckBox_stateChanged(int arg1);
  void on_yawCheckBox_stateChanged(int arg1);
  void on_controlModeButtonGroup_buttonClicked(QAbstractButton*);
  void on_angleControlButtonGroup_buttonClicked(QAbstractButton*);

  void on_pitchControlSpinBox_valueChanged(double arg1);
  void on_pitchControlSlider_valueChanged(int value);
  void on_rollControlSpinBox_valueChanged(double arg1);
  void on_rollControlSlider_valueChanged(int value);
  void on_yawControlSpinBox_valueChanged(double arg1);
  void on_yawControlSlider_valueChanged(int value);

  void on_sendGimbalCommand_clicked();

  void on_photoButton_clicked();

  void on_videoButton_clicked();

private:
  Ui::CameraGimbalControl*     ui;
  DJI::OSDK::Vehicle*          vehicle;
  DJI::OSDK::Gimbal::AngleData angleData;
  DJI::OSDK::Gimbal::SpeedData speedData;
};

#endif // QGIMBAL_H

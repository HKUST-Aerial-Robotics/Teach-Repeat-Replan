#include "camera_gimbal_control_panel.hpp"
#include "ui_camera_gimbal_control_panel.h"

using namespace DJI::OSDK;

CameraGimbalControl::CameraGimbalControl(QWidget* parent, Vehicle* vehicle)
  : QWidget(parent)
  , ui(new Ui::CameraGimbalControl)
{
  ui->setupUi(this);
  //    ui->videoButton->setStyleSheet("background-color: green; color: white");

  if (vehicle->gimbal == 0)
  {
    ui->groupBox->setEnabled(false);
    ui->groupBox->setTitle("Gimbal Not Mounted");
  }

  this->angleData.mode = this->angleData.duration = 0;
  this->angleData.pitch = this->angleData.roll = this->angleData.yaw = 0;
  this->speedData.pitch = this->speedData.roll = this->speedData.yaw = 0;

  this->vehicle = vehicle;
}

CameraGimbalControl::~CameraGimbalControl()
{
  delete ui;
}

void
CameraGimbalControl::on_controlModeButtonGroup_buttonClicked(
  QAbstractButton* button)
{
  QString name = button->text();
  if (name == "Angle")
  {
    if (!ui->angleControlMode->isEnabled())
    {
      ui->angleControlMode->setEnabled(true);
      ui->rollCheckBox->setEnabled(true);
      ui->pitchCheckBox->setEnabled(true);
      ui->yawCheckBox->setEnabled(true);
    }
    ui->durationLabel->setText("Reach desired position in: ");
    ui->rollControlModeText->setText("Roll (deg):");
    ui->pitchControlModeText->setText("Pitch (deg):");
    ui->yawControlModeText->setText("Yaw (deg):");
  }
  else if (name == "Speed")
  {
    ui->angleControlMode->setEnabled(false);
    ui->rollCheckBox->setEnabled(false);
    ui->pitchCheckBox->setEnabled(false);
    ui->yawCheckBox->setEnabled(false);
    ui->durationLabel->setText("Send speed commands for: ");
    ui->rollControlModeText->setText("Roll rate (deg/s):");
    ui->pitchControlModeText->setText("Pitch rate (deg/s):");
    ui->yawControlModeText->setText("Yaw rate (deg/s):");
  }
  this->handleRPYRanges(name);
}

void
CameraGimbalControl::on_angleControlButtonGroup_buttonClicked(
  QAbstractButton* button)
{
  QString name = button->text();
  angleData.mode &= 0xFE;
  if (name == "Absolute")
  {
    angleData.mode |= 0x01;
  }
}

void
CameraGimbalControl::handleRPYRanges(QString modeString)
{
  int minRoll;
  int minPitch;
  int maxRoll;
  int maxPitch;
  int minYaw;
  int maxYaw;

  if (modeString == "Angle")
  {
    minRoll  = -35;
    maxRoll  = 35;
    minPitch = -90;
    maxPitch = 30;
    minYaw   = -320;
    maxYaw   = 320;
  }
  else if (modeString == "Speed")
  {
    minPitch = minRoll = minYaw = -180;
    maxPitch = maxRoll = maxYaw = 180;
  }
  //! X min
  ui->rollControlSlider->setMinimum(minRoll);
  ui->rollControlSpinBox->setMinimum(minRoll);
  ui->rollControlMin->setText(QString::number(minRoll));

  //! X Max
  ui->rollControlSlider->setMaximum(maxRoll);
  ui->rollControlSpinBox->setMaximum(maxRoll);
  ui->rollControlMax->setText(QString::number(maxRoll));

  //! X default value
  ui->rollControlSlider->setValue(0);

  //! Y Min
  ui->pitchControlSlider->setMinimum(minPitch);
  ui->pitchControlSpinBox->setMinimum(minPitch);
  ui->pitchControlMin->setText(QString::number(minPitch));

  //! Y Max
  ui->pitchControlSlider->setMaximum(maxPitch);
  ui->pitchControlSpinBox->setMaximum(maxPitch);
  ui->pitchControlMax->setText(QString::number(maxPitch));

  //! Y default value
  ui->pitchControlSlider->setValue(0);

  //! Yaw Min
  ui->yawControlSlider->setMinimum(minYaw);
  ui->yawControlSpinBox->setMinimum(minYaw);
  ui->yawControlMin->setText(QString::number(minYaw));

  //! Yaw Max
  ui->yawControlSlider->setMaximum(maxYaw);
  ui->yawControlSpinBox->setMaximum(maxYaw);
  ui->yawControlMax->setText(QString::number(maxYaw));

  //! Yaw default value
  ui->yawControlSlider->setValue(0);
}

void
CameraGimbalControl::on_rollCheckBox_stateChanged(int arg1)
{
  angleData.mode &= 0xFB;
  if (arg1 == Qt::Unchecked)
  {
    angleData.mode |= 0x04;
    ui->rollControlSpinBox->setEnabled(false);
    ui->rollControlSlider->setEnabled(false);
  }
  else if (arg1 == Qt::Checked)
  {
    if (!ui->rollControlSpinBox->isEnabled())
    {
      ui->rollControlSpinBox->setEnabled(true);
      ui->rollControlSlider->setEnabled(true);
    }
  }
}

void
CameraGimbalControl::on_pitchCheckBox_stateChanged(int arg1)
{
  angleData.mode &= 0xF7;
  if (arg1 == Qt::Unchecked)
  {
    angleData.mode |= 0x08;
    ui->pitchControlSpinBox->setEnabled(false);
    ui->pitchControlSlider->setEnabled(false);
  }
  else if (arg1 == Qt::Checked)
  {
    if (!ui->pitchControlSpinBox->isEnabled())
    {
      ui->pitchControlSpinBox->setEnabled(true);
      ui->pitchControlSlider->setEnabled(true);
    }
  }
}

void
CameraGimbalControl::on_yawCheckBox_stateChanged(int arg1)
{
  angleData.mode &= 0xFD;
  if (arg1 == Qt::Unchecked)
  {
    angleData.mode |= 0x02;
    ui->yawControlSpinBox->setEnabled(false);
    ui->yawControlSlider->setEnabled(false);
  }
  else if (arg1 == Qt::Checked)
  {
    if (!ui->yawControlSpinBox->isEnabled())
    {
      ui->yawControlSpinBox->setEnabled(true);
      ui->yawControlSlider->setEnabled(true);
    }
  }
}

void
CameraGimbalControl::on_yawControlSpinBox_valueChanged(double arg1)
{
  if (ui->yawControlSlider->value() != (int)arg1)
  {
    ui->yawControlSlider->setValue((int)arg1);
  }
  if (ui->controlModeButtonGroup->checkedButton()->text() == "Angle")
  {
    this->angleData.yaw = (int16_t)(arg1 * 10);
  }
  else if (ui->controlModeButtonGroup->checkedButton()->text() == "Speed")
  {
    this->speedData.yaw = (int16_t)(arg1 * 10);
  }
}

void
CameraGimbalControl::on_yawControlSlider_valueChanged(int value)
{
  ui->yawControlSpinBox->setValue(value);
}

void
CameraGimbalControl::on_rollControlSpinBox_valueChanged(double arg1)
{
  if (ui->rollControlSlider->value() != (int)arg1)
  {
    ui->rollControlSlider->setValue((int)arg1);
  }

  if (ui->controlModeButtonGroup->checkedButton()->text() == "Angle")
  {
    this->angleData.roll = (int16_t)(arg1 * 10);
  }
  else if (ui->controlModeButtonGroup->checkedButton()->text() == "Speed")
  {
    this->speedData.roll = (int16_t)(arg1 * 10);
  }
}

void
CameraGimbalControl::on_rollControlSlider_valueChanged(int value)
{
  ui->rollControlSpinBox->setValue(value);
}

void
CameraGimbalControl::on_pitchControlSpinBox_valueChanged(double arg1)
{
  if (ui->pitchControlSlider->value() != (int)arg1)
  {
    ui->pitchControlSlider->setValue((int)arg1);
  }

  if (ui->controlModeButtonGroup->checkedButton()->text() == "Angle")
  {
    this->angleData.pitch = (int16_t)(arg1 * 10);
  }
  else if (ui->controlModeButtonGroup->checkedButton()->text() == "Speed")
  {
    this->speedData.pitch = (int16_t)(arg1 * 10);
  }
}

void
CameraGimbalControl::on_pitchControlSlider_valueChanged(int value)
{
  ui->pitchControlSpinBox->setValue(value);
}

void
CameraGimbalControl::on_sendGimbalCommand_clicked()
{
  if (ui->controlModeButtonGroup->checkedButton()->text() == "Angle")
  {
    this->angleData.duration = (int16_t)(10 * ui->durationSpinBox->value());
    vehicle->gimbal->setAngle(&angleData);
  }
  else if (ui->controlModeButtonGroup->checkedButton()->text() == "Speed")
  {
    double duration = 0, elapsed = 0;
    duration = ui->durationSpinBox->value();
    while (elapsed < duration)
    {
      vehicle->gimbal->setSpeed(&speedData);
      QThread::msleep(100);
      elapsed += 0.1;
    }
  }
}

void
CameraGimbalControl::on_photoButton_clicked()
{
  vehicle->camera->shootPhoto();
}

void
CameraGimbalControl::on_videoButton_clicked()
{
  if (ui->videoButton->text() == "Start Video")
  {
    vehicle->camera->videoStart();
    ui->videoButton->setText(QString("Stop Video"));
    //    ui->videoButton->setStyleSheet("background-color: red; color: white");
  }
  if (ui->videoButton->text() == "Stop Video")
  {
    vehicle->camera->videoStop();
    ui->videoButton->setText(QString("Start Video"));
    //    ui->videoButton->setStyleSheet("background-color: green; color:
    //    white");
  }
}

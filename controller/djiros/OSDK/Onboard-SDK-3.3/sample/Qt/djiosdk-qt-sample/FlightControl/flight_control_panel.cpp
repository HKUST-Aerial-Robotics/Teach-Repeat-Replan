#include "flight_control_panel.hpp"
#include "ui_flight_control_panel.h"
#include <QDebug>

using namespace DJI;
using namespace DJI::OSDK;

FlightControlPanel::FlightControlPanel(QWidget* parent, Vehicle* vehicle)
  : QWidget(parent)
  , ui(new Ui::FlightControlPanel)
  , command(0, 0, 0, 0, 0)
{
  ui->setupUi(this);
  this->vehicle = vehicle;
  // connect(this, SIGNAL(changeCommandStatus(QString)), this,
  // SLOT(commandStatusChanged(Qstring)));
}

FlightControlPanel::~FlightControlPanel()
{
  delete ui;
}

void
FlightControlPanel::on_btn_flight_runCommand_clicked()
{
  int flightTask;
  switch (ui->cb_command->currentIndex())
  {
    case 0:
      flightTask = Control::FlightCommand::takeOff;
      break;
    case 1:
      flightTask = Control::FlightCommand::landing;
      break;
    case 2:
      flightTask = Control::FlightCommand::goHome;
      break;
    case 3:
      flightTask = Control::FlightCommand::exitTakeOff;
      break;
    case 4:
      flightTask = Control::FlightCommand::exitLanding;
      break;
    case 5:
      flightTask = Control::FlightCommand::exitGoHome;
      break;
    case 6:
      flightTask = Control::FlightCommand::calibrateCompass;
      break;
    case 7:
      flightTask = Control::FlightCommand::exitCalibrateCompass;
      break;
    case 8:
      flightTask = Control::FlightCommand::landingGearUp;
      break;
    case 9:
      flightTask = Control::FlightCommand::landingGearDown;
      break;
    default:
      DERROR("Unknown command %d", ui->cb_command->currentIndex());
      break;
  }
  qDebug() << ui->cb_command->currentIndex() << flightTask;
  vehicle->control->action(flightTask, FlightControlPanel::actionCallback,
                           this);
}

void
FlightControlPanel::on_buttonGroupHorizontal_buttonClicked(
  QAbstractButton* button)
{
  QString name = button->text();
  command.flag &= 0x3F;
  if (name == "Angle")
  {
    command.flag |= Control::HorizontalLogic::HORIZONTAL_ANGLE;
    ui->xControlModeText->setText("Roll (deg):");
    ui->yControlModeText->setText("Pitch (deg):");
    this->handleXYRanges(-30, 30);
  }
  else if (name == "Velocity")
  {
    command.flag |= Control::HorizontalLogic::HORIZONTAL_VELOCITY;
    ui->xControlModeText->setText("X Vel (m/s):");
    ui->yControlModeText->setText("Y Vel (m/s):");
    this->handleXYRanges(-10, 10);
  }
  else if (name == "Position")
  {
    command.flag |= Control::HorizontalLogic::HORIZONTAL_POSITION;
    ui->xControlModeText->setText("X Offset (m):");
    ui->yControlModeText->setText("Y Offset (m):");
    this->handleXYRanges(-10, 10);
  }
  else if (name == "Angular Rate")
  {
    command.flag |= Control::HorizontalLogic::HORIZONTAL_ANGULAR_RATE;
    ui->xControlModeText->setText("Roll rate (deg/s):");
    ui->yControlModeText->setText("Pitch rate (deg/s):");
    this->handleXYRanges(-50, 50);
  }
}

void
FlightControlPanel::on_buttonGroupVertical_buttonClicked(
  QAbstractButton* button)
{
  QString name = button->text();
  command.flag &= 0xCF;
  if (name == "Thrust")
  {
    command.flag |= Control::VerticalLogic::VERTICAL_THRUST;
    ui->zControlModeText->setText("Z Thr (%):");
    this->handleZRanges(0, 100);
    ui->zControlSlider->setValue(25);
  }
  else if (name == "Velocity")
  {
    command.flag |= Control::VerticalLogic::VERTICAL_VELOCITY;
    ui->zControlModeText->setText("Z Vel (m/s):");
    this->handleZRanges(-5, 5);
  }
  else if (name == "Position")
  {
    command.flag |= Control::VerticalLogic::VERTICAL_POSITION;
    ui->zControlModeText->setText("Z Abs Pos (m):");
    this->handleZRanges(0, 120);
    ui->zControlSlider->setValue(1.2);
  }
}

void
FlightControlPanel::on_buttonGroupYaw_buttonClicked(QAbstractButton* button)
{
  QString name = button->text();
  command.flag &= 0xF7;
  if (name == "Yaw Angle")
  {
    command.flag |= Control::YawLogic::YAW_ANGLE;
    ui->yawControlModeText->setText("Yaw (deg):");
    this->handleYawRanges(-180, 180);
  }
  else
  {
    command.flag |= Control::YawLogic::YAW_RATE;
    ui->yawControlModeText->setText("Yaw rate (deg/s):");
    this->handleYawRanges(-100, 100);
  }
}

void
FlightControlPanel::on_buttonGroupFrame_buttonClicked(QAbstractButton* button)
{
  QString name = button->text();
  command.flag &= 0xF9;
  if (name == "Ground")
  {
    command.flag |= Control::HorizontalCoordinate::HORIZONTAL_GROUND;
  }
  else
  {
    command.flag |= Control::HorizontalCoordinate::HORIZONTAL_BODY;
  }
}

void
FlightControlPanel::on_buttonGroupStable_buttonClicked(QAbstractButton* button)
{
  if (vehicle->getFwVersion() > Version::FW(3, 1, 0, 0))
  {
    QString name = button->text();
    command.flag &= 0xFE;
    if (name == "Disable")
    {
      command.flag |= Control::StableMode::STABLE_DISABLE;
    }
    else
    {
      command.flag |= Control::StableMode::STABLE_ENABLE;
    }
  }
  else
  {
    ui->groupBox9->setDisabled(true);
  }
}

void
FlightControlPanel::on_xControlSpinBox_valueChanged(double arg1)
{
  if (ui->xControlSlider->value() != (int)arg1)
  {
    ui->xControlSlider->setValue((int)arg1);
  }
  this->command.x = arg1;
}

void
FlightControlPanel::on_xControlSlider_valueChanged(int value)
{
  ui->xControlSpinBox->setValue(value);
}

void
FlightControlPanel::on_yControlSpinBox_valueChanged(double arg1)
{
  if (ui->yControlSlider->value() != (int)arg1)
  {
    ui->yControlSlider->setValue((int)arg1);
  }
  this->command.y = arg1;
}

void
FlightControlPanel::on_yControlSlider_valueChanged(int value)
{
  ui->yControlSpinBox->setValue(value);
}

void
FlightControlPanel::on_zControlSpinBox_valueChanged(double arg1)
{
  if (ui->zControlSlider->value() != (int)arg1)
  {
    ui->zControlSlider->setValue((int)arg1);
  }
  this->command.z = arg1;
}

void
FlightControlPanel::on_zControlSlider_valueChanged(int value)
{
  ui->zControlSpinBox->setValue(value);
}

void
FlightControlPanel::on_yawControlSpinBox_valueChanged(double arg1)
{
  if (ui->yawControlSlider->value() != (int)arg1)
  {
    ui->yawControlSlider->setValue((int)arg1);
  }
  this->command.yaw = arg1;
}

void
FlightControlPanel::on_yawControlSlider_valueChanged(int value)
{
  ui->yawControlSpinBox->setValue(value);
}

void
FlightControlPanel::handleXYRanges(double minXY, double maxXY)
{
  //! X min
  ui->xControlSlider->setMinimum(minXY);
  ui->xControlSpinBox->setMinimum(minXY);
  ui->xControlMin->setText(QString::number(minXY));

  //! X Max
  ui->xControlSlider->setMaximum(maxXY);
  ui->xControlSpinBox->setMaximum(maxXY);
  ui->xControlMax->setText(QString::number(maxXY));

  //! X default value
  ui->xControlSlider->setValue(0);

  //! Y Min
  ui->yControlSlider->setMinimum(minXY);
  ui->yControlSpinBox->setMinimum(minXY);
  ui->yControlMin->setText(QString::number(minXY));

  //! Y Max
  ui->yControlSlider->setMaximum(maxXY);
  ui->yControlSpinBox->setMaximum(maxXY);
  ui->yControlMax->setText(QString::number(maxXY));

  //! Y default value
  ui->yControlSlider->setValue(0);
}

void
FlightControlPanel::handleZRanges(double minZ, double maxZ)
{
  //! Z min
  ui->zControlSlider->setMinimum(minZ);
  ui->zControlSpinBox->setMinimum(minZ);
  ui->zControlMin->setText(QString::number(minZ));

  //! Z max
  ui->zControlSlider->setMaximum(maxZ);
  ui->zControlSpinBox->setMaximum(maxZ);
  ui->zControlMax->setText(QString::number(maxZ));

  //! Z default value
  ui->zControlSlider->setValue(0);
}

void
FlightControlPanel::handleYawRanges(double minYaw, double maxYaw)
{
  //! Yaw max
  ui->yawControlSlider->setMinimum(minYaw);
  ui->yawControlSpinBox->setMinimum(minYaw);
  ui->yawControlMin->setText(QString::number(minYaw));

  //! Yaw min
  ui->yawControlSlider->setMaximum(maxYaw);
  ui->yawControlSpinBox->setMaximum(maxYaw);
  ui->yawControlMin->setText(QString::number(maxYaw));

  //! Yaw default value
  ui->yawControlSlider->setValue(0);
}

void
FlightControlPanel::on_btn_flight_send_clicked()
{
  float32_t duration = ui->durationSpinBox->value();
  float32_t elapsed  = 0;

  //! 50Hz control
  while (elapsed < duration)
  {
    vehicle->control->flightCtrl(command);
    QThread::usleep(20000);
    elapsed += 0.02;
  }
}

void
FlightControlPanel::actionCallback(Vehicle* vehiclePtr, RecvContainer recvFrame,
                                   UserData userData)
{
  FlightControlPanel* fcPanel = (FlightControlPanel*)userData;
  ACK::ErrorCode      ack;
  Control*            controlPtr = vehiclePtr->control;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <= sizeof(uint16_t))
  {

    ack.info = recvFrame.recvInfo;
    ack.data = recvFrame.recvData.commandACK;

    if (ACK::getError(ack))
    {
      ACK::getErrorCodeMessage(ack, __func__);
      emit fcPanel->changeCommandStatus("Error! See console.");
    }
    else
    {
      emit fcPanel->changeCommandStatus("Success");
    }
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
  }
}

void
FlightControlPanel::commandStatusChanged(QString textToDisplay)
{
  this->ui->btn_flight_runCommand->setText(textToDisplay);
}

void
FlightControlPanel::on_cb_command_activated(const QString& arg1)
{
  this->ui->btn_flight_runCommand->setText(QString("Run Command"));
}

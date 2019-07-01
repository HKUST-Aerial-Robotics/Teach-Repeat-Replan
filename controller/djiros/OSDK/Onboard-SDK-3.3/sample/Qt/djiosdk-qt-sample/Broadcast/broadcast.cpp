#include "broadcast.hpp"
#include "ui_broadcast.h"
#include <QDebug>

using namespace DJI::OSDK;

Broadcast::Broadcast(QWidget* parent, Vehicle* vehicle)
  : QWidget(parent)
  , ui(new Ui::Broadcast)
{
  ui->setupUi(this);
  this->vehicle  = vehicle;
  broadcastTimer = new QTimer(this);
  connect(broadcastTimer, SIGNAL(timeout()), this, SLOT(updateDisplay()));
  broadcastTimer->start(10);
}

Broadcast::~Broadcast()
{
  delete ui;
}

void
Broadcast::updateDisplay()
{
  //! @note this function cost too much time to run.
  //! it is better run outside the broadcastCallback.
  if (ui->tsDisplay->isEnabled() == true)
    updateTime();
  if (ui->le_coreCapacity->isEnabled() == true)
    updateCapacity();
  if (ui->le_coreFlightStatus->isEnabled() == true)
    updateFlightStatus();
  if (ui->le_coreControlDevice->isEnabled() == true)
    updateControlDevice();
  if (ui->gimbalDisplay->isEnabled() == true)
  {
    updateCameraYaw();
    updateCameraRoll();
    updateCameraPitch();
  }
  if (ui->rcDisplay->isEnabled() == true)
    updateVirturalRCData();
  if (ui->accDisplay->isEnabled() == true)
    updateFlightAcc();
  if (ui->wDisplay->isEnabled() == true)
    updateFlightPal();
  if (ui->magDisplay->isEnabled() == true)
    updateFlightMagnet();
  if (ui->qDisplay->isEnabled() == true)
    updateFlightQuaternion();
  if (ui->velDisplay->isEnabled() == true)
    updateFlightVelocity();
  if (ui->posDisplay->isEnabled() == true)
    updateFlightPosition();
  if (ui->gpsDisplay->isEnabled() == true)
    updateGPS();
  if (ui->rtkDisplay->isEnabled() == true)
    updateRTK();
}

void
Broadcast::updateTime()
{
  ui->le_coreTimeStamp->setText(
    QString::number(vehicle->broadcast->getTimeStamp().time_ms));
  ui->le_coreNanoStamp->setText(
    QString::number(vehicle->broadcast->getTimeStamp().time_ns));
}

void
Broadcast::updateCapacity()
{
  ui->le_coreCapacity->setText(
    QString::number(vehicle->broadcast->getBatteryInfo().capacity));
}

void
Broadcast::updateFlightStatus()
{
  ui->le_coreFlightStatus->setText(
    QString::number(vehicle->broadcast->getStatus().flight));
}

void
Broadcast::updateControlDevice()
{
  ui->le_coreControlDevice->setText(
    QString::number(vehicle->broadcast->getSDKInfo().controlMode));
}

void
Broadcast::updateCameraYaw()
{
  ui->le_cameraYaw->setText(
    QString::number(vehicle->broadcast->getGimbal().yaw));
  if (vehicle->broadcast->getGimbal().yawLimit)
    ui->cb_cameraYawLimit->setChecked(true);
  else
    ui->cb_cameraYawLimit->setChecked(false);
}

void
Broadcast::updateCameraRoll()
{
  ui->le_cameraRoll->setText(
    QString::number(vehicle->broadcast->getGimbal().roll));
  if (vehicle->broadcast->getGimbal().rollLimit)
    ui->cb_cameraRollLimit->setChecked(true);
  else
    ui->cb_cameraRollLimit->setChecked(false);
}

void
Broadcast::updateCameraPitch()
{
  ui->le_cameraPitch->setText(
    QString::number(vehicle->broadcast->getGimbal().pitch));
  if (vehicle->broadcast->getGimbal().pitchLimit)
    ui->cb_cameraPitchLimit->setChecked(true);
  else
    ui->cb_cameraPitchLimit->setChecked(false);
}

void
Broadcast::updateVirturalRCData()
{
  ui->le_vrcYaw->setText(QString::number(vehicle->broadcast->getRC().yaw));
  ui->le_vrcRoll->setText(QString::number(vehicle->broadcast->getRC().roll));
  ui->le_vrcPitch->setText(QString::number(vehicle->broadcast->getRC().pitch));
  ui->le_vrcThrottle->setText(
    QString::number(vehicle->broadcast->getRC().throttle));
  ui->le_vrcMode->setText(QString::number(vehicle->broadcast->getRC().mode));
  ui->le_vrcGear->setText(QString::number(vehicle->broadcast->getRC().gear));
}

void
Broadcast::updateFlightAcc()
{
  ui->le_Flight_accx->setText(
    QString::number(vehicle->broadcast->getAcceleration().x));
  ui->le_Flight_accy->setText(
    QString::number(vehicle->broadcast->getAcceleration().y));
  ui->le_Flight_accz->setText(
    QString::number(vehicle->broadcast->getAcceleration().z));
}

void
Broadcast::updateFlightPal()
{
  ui->le_Flight_palx->setText(
    QString::number(vehicle->broadcast->getAngularRate().x));
  ui->le_Flight_paly->setText(
    QString::number(vehicle->broadcast->getAngularRate().y));
  ui->le_Flight_palz->setText(
    QString::number(vehicle->broadcast->getAngularRate().z));
}

void
Broadcast::updateFlightMagnet()
{
  ui->le_Flight_magx->setText(QString::number(vehicle->broadcast->getMag().x));
  ui->le_Flight_magy->setText(QString::number(vehicle->broadcast->getMag().y));
  ui->le_Flight_magz->setText(QString::number(vehicle->broadcast->getMag().z));
}

void
Broadcast::updateFlightQuaternion()
{
  ui->le_Flight_Q0->setText(
    QString::number(vehicle->broadcast->getQuaternion().q0));
  ui->le_Flight_Q1->setText(
    QString::number(vehicle->broadcast->getQuaternion().q1));
  ui->le_Flight_Q2->setText(
    QString::number(vehicle->broadcast->getQuaternion().q2));
  ui->le_Flight_Q3->setText(
    QString::number(vehicle->broadcast->getQuaternion().q3));
}

void
Broadcast::updateFlightVelocity()
{
  ui->le_Flight_Vx->setText(
    QString::number(vehicle->broadcast->getVelocity().x));
  ui->le_Flight_Vy->setText(
    QString::number(vehicle->broadcast->getVelocity().y));
  ui->le_Flight_Vz->setText(
    QString::number(vehicle->broadcast->getVelocity().z));
  ui->le_Flight_VH->setText(
    QString::number(vehicle->broadcast->getVelocityInfo().health));
}

void
Broadcast::updateFlightPosition()
{
  ui->le_Flight_PosH->setText(
    QString::number(vehicle->broadcast->getGlobalPosition().height));
  ui->le_Flight_PosLa->setText(
    QString::number(vehicle->broadcast->getGlobalPosition().latitude));
  ui->le_Flight_PosLo->setText(
    QString::number(vehicle->broadcast->getGlobalPosition().longitude));
  ui->le_Flight_PosAl->setText(
    QString::number(vehicle->broadcast->getGlobalPosition().altitude));
  ui->le_Flight_PosHealth->setText(
    QString::number(vehicle->broadcast->getGlobalPosition().health));
}

void
Broadcast::updateGPS()
{

  ui->le_gps_date->setText(
    QString::number(vehicle->broadcast->getGPSInfo().time.date));
  ui->le_gps_time->setText(
    QString::number(vehicle->broadcast->getGPSInfo().time.time));
  ui->le_gps_longitude->setText(
    QString::number(vehicle->broadcast->getGPSInfo().longitude));
  ui->le_gps_latitude->setText(
    QString::number(vehicle->broadcast->getGPSInfo().latitude));
  ui->le_gps_hmsl->setText(
    QString::number(vehicle->broadcast->getGPSInfo().HFSL));
  ui->le_gps_VN->setText(
    QString::number(vehicle->broadcast->getGPSInfo().velocityNED.x));
  ui->le_gps_VE->setText(
    QString::number(vehicle->broadcast->getGPSInfo().velocityNED.y));
  ui->le_gps_VD->setText(
    QString::number(vehicle->broadcast->getGPSInfo().velocityNED.z));
  ui->le_gps_hdop->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.hdop));
  ui->le_gps_pdop->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.pdop));

  ui->le_gps_GNSSFlag->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.gnssStatus));
  ui->le_gps_gnssFix->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.fix));
  ui->le_gps_hacc->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.hacc));
  ui->le_gps_sacc->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.sacc));
  ui->le_gps_guse->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.usedGPS));
  ui->le_gps_gnuse->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.usedGLN));
  ui->le_gps_svn->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.NSV));
  ui->le_gps_gps->setText(
    QString::number(vehicle->broadcast->getGPSInfo().detail.GPScounter));
}

void
Broadcast::updateRTK()
{
  //! @todo replace
  ui->le_rtk_date->setText(
    QString::number(vehicle->broadcast->getRTKInfo().pos.time.date));
  ui->le_rtk_time->setText(
    QString::number(vehicle->broadcast->getRTKInfo().pos.time.time));
  ui->le_rtk_longitude->setText(
    QString::number(vehicle->broadcast->getRTKInfo().pos.data.longitude));
  ui->le_rtk_latitude->setText(
    QString::number(vehicle->broadcast->getRTKInfo().pos.data.latitude));
  ui->le_rtk_hmsl->setText(
    QString::number(vehicle->broadcast->getRTKInfo().pos.data.HFSL));
  ui->le_rtk_VN->setText(
    QString::number(vehicle->broadcast->getRTKInfo().velocityNED.x));
  ui->le_rtk_VE->setText(
    QString::number(vehicle->broadcast->getRTKInfo().velocityNED.y));
  ui->le_rtk_VD->setText(
    QString::number(vehicle->broadcast->getRTKInfo().velocityNED.z));
  ui->le_rtk_yaw->setText(
    QString::number(vehicle->broadcast->getRTKInfo().yaw));

  ui->le_rtk_flag_1->setText(
    QString::number(vehicle->broadcast->getRTKInfo().posHealthFlag));
  ui->le_rtk_flag_2->setText(
    QString::number(vehicle->broadcast->getRTKInfo().yawHealthFlag));
}

void
Broadcast::on_pushButtonBroadcastFreqSet_clicked()
{
  uint8_t data[16];
  data[0] = ui->comboBoxTs->currentIndex();
  data[1] = ui->comboBoxQ->currentIndex();
  data[2] = ui->comboBoxAcc->currentIndex();
  data[3] = ui->comboBoxVel->currentIndex();
  data[4] = ui->comboBoxW->currentIndex();
  data[5] = ui->comboBoxPos->currentIndex();
  if (strcmp(vehicle->getHwVersion(), "M100") == 0)
  {
    data[6]  = ui->comboBoxMag->currentIndex();
    data[7]  = ui->comboBoxRC->currentIndex();
    data[8]  = ui->comboBoxGimbal->currentIndex();
    data[9]  = ui->comboBoxStatus->currentIndex();
    data[10] = ui->comboBoxBat->currentIndex();
    data[11] = ui->comboBoxCtrlInfo->currentIndex();
    data[12] = 0;
    data[13] = 0;
    data[14] = 0;
    data[15] = 0;
  }
  else
  {
    data[6]  = ui->comboBoxGPS->currentIndex(); // rtk
    data[7]  = ui->comboBoxRTK->currentIndex(); // gps
    data[8]  = ui->comboBoxMag->currentIndex();
    data[9]  = ui->comboBoxRC->currentIndex();
    data[10] = ui->comboBoxGimbal->currentIndex();
    data[11] = ui->comboBoxStatus->currentIndex();
    data[12] = ui->comboBoxBat->currentIndex();
    data[13] = ui->comboBoxCtrlInfo->currentIndex();
    data[14] = 0;
    data[15] = 0;
  }
  manageDisplayEnabling(data);
  vehicle->broadcast->setBroadcastFreq(data);
}

void
Broadcast::manageDisplayEnabling(uint8_t* data)
{
  // Timestamp
  if (data[0] == 0)
  {
    ui->tsDisplay->setEnabled(false);
  }
  else
  {
    if (!ui->tsDisplay->isEnabled())
    {
      ui->tsDisplay->setEnabled(true);
    }
  }

  // Quaternion
  if (data[1] == 0)
  {
    ui->qDisplay->setEnabled(false);
  }
  else
  {
    if (!ui->qDisplay->isEnabled())
    {
      ui->qDisplay->setEnabled(true);
    }
  }

  // Acc
  if (data[2] == 0)
  {
    ui->accDisplay->setEnabled(false);
  }
  else
  {
    if (!ui->accDisplay->isEnabled())
    {
      ui->accDisplay->setEnabled(true);
    }
  }

  // Vel
  if (data[3] == 0)
  {
    ui->velDisplay->setEnabled(false);
  }
  else
  {
    if (!ui->velDisplay->isEnabled())
    {
      ui->velDisplay->setEnabled(true);
    }
  }

  // Ang Vel
  if (data[4] == 0)
  {
    ui->wDisplay->setEnabled(false);
  }
  else
  {
    if (!ui->wDisplay->isEnabled())
    {
      ui->wDisplay->setEnabled(true);
    }
  }

  // Pos
  if (data[5] == 0)
  {
    ui->posDisplay->setEnabled(false);
  }
  else
  {
    if (!ui->posDisplay->isEnabled())
    {
      ui->posDisplay->setEnabled(true);
    }
  }

  if (strcmp(vehicle->getHwVersion(), "M100") == 0)
  {
    // Mag
    if (data[6] == 0)
    {
      ui->magDisplay->setEnabled(false);
    }
    else
    {
      if (!ui->magDisplay->isEnabled())
      {
        ui->magDisplay->setEnabled(true);
      }
    }

    // RC
    if (data[7] == 0)
    {
      ui->rcDisplay->setEnabled(false);
    }
    else
    {
      if (!ui->rcDisplay->isEnabled())
      {
        ui->rcDisplay->setEnabled(true);
      }
    }

    // Gimbal
    if (data[8] == 0)
    {
      ui->gimbalDisplay->setEnabled(false);
    }
    else
    {
      if (!ui->gimbalDisplay->isEnabled())
      {
        ui->gimbalDisplay->setEnabled(true);
      }
    }

    // Status
    if (data[9] == 0)
    {
      ui->le_coreFlightStatus->setEnabled(false);
    }
    else
    {
      if (!ui->le_coreFlightStatus->isEnabled())
      {
        ui->le_coreFlightStatus->setEnabled(true);
      }
    }

    // Battery
    if (data[10] == 0)
    {
      ui->le_coreCapacity->setEnabled(false);
    }
    else
    {
      if (!ui->le_coreCapacity->isEnabled())
      {
        ui->le_coreCapacity->setEnabled(true);
      }
    }

    // Control Info
    if (data[11] == 0)
    {
      ui->le_coreControlDevice->setEnabled(false);
    }
    else
    {
      if (!ui->le_coreControlDevice->isEnabled())
      {
        ui->le_coreControlDevice->setEnabled(true);
      }
    }
  }
  else // M600/A3/N3
  {
    // GPS
    if (data[6] == 0)
    {
      ui->gpsDisplay->setEnabled(false);
    }
    else
    {
      if (!ui->gpsDisplay->isEnabled())
      {
        ui->gpsDisplay->setEnabled(true);
      }
    }

    // RTK
    if (data[7] == 0)
    {
      ui->rtkDisplay->setEnabled(false);
    }
    else
    {
      if (!ui->rtkDisplay->isEnabled())
      {
        ui->rtkDisplay->setEnabled(true);
      }
    }

    // Mag
    if (data[8] == 0)
    {
      ui->magDisplay->setEnabled(false);
    }
    else
    {
      if (!ui->magDisplay->isEnabled())
      {
        ui->magDisplay->setEnabled(true);
      }
    }

    // RC
    if (data[9] == 0)
    {
      ui->rcDisplay->setEnabled(false);
    }
    else
    {
      if (!ui->rcDisplay->isEnabled())
      {
        ui->rcDisplay->setEnabled(true);
      }
    }

    // Gimbal
    if (data[10] == 0)
    {
      ui->gimbalDisplay->setEnabled(false);
    }
    else
    {
      if (!ui->gimbalDisplay->isEnabled())
      {
        ui->gimbalDisplay->setEnabled(true);
      }
    }

    // Status
    if (data[11] == 0)
    {
      ui->le_coreFlightStatus->setEnabled(false);
    }
    else
    {
      if (!ui->le_coreFlightStatus->isEnabled())
      {
        ui->le_coreFlightStatus->setEnabled(true);
      }
    }

    // Battery
    if (data[12] == 0)
    {
      ui->le_coreCapacity->setEnabled(false);
    }
    else
    {
      if (!ui->le_coreCapacity->isEnabled())
      {
        ui->le_coreCapacity->setEnabled(true);
      }
    }

    // Control Info
    if (data[13] == 0)
    {
      ui->le_coreControlDevice->setEnabled(false);
    }
    else
    {
      if (!ui->le_coreControlDevice->isEnabled())
      {
        ui->le_coreControlDevice->setEnabled(true);
      }
    }
  }
}

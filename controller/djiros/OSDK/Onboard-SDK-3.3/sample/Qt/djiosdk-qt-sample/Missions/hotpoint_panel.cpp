#include "hotpoint_panel.hpp"
#include "ui_hotpoint_panel.h"

using namespace DJI::OSDK;

HotpointPanel::HotpointPanel(QWidget* parent, Vehicle* vehicle)
  : QFrame(parent)
  , ui(new Ui::HotpointPanel)
{
  ui->setupUi(this);
  this->vehicle = vehicle;
  vehicle->missionManager->init(DJI_MISSION_TYPE::HOTPOINT);
}

HotpointPanel::~HotpointPanel()
{
  delete ui;
}

void
HotpointPanel::hotpointReadCallback(Vehicle* vehicle, RecvContainer recvFrame,
                                    UserData userData)
{
  HotpointPanel* sdk = (HotpointPanel*)userData;
  HotpointMission::getHotpointSettingsCallback(vehicle, recvFrame, vehicle->missionManager->hpMission);
  DSTATUS("Refreshing hotpoint data");
  sdk->ui->le_hp_la->setText(
    QString::number(vehicle->missionManager->hpMission->getData().latitude));
  sdk->ui->le_hp_lo->setText(
    QString::number(vehicle->missionManager->hpMission->getData().longitude));
  sdk->ui->le_hp_al->setText(
    QString::number(vehicle->missionManager->hpMission->getData().height));
  sdk->ui->le_hp_pa->setText(
    QString::number(vehicle->missionManager->hpMission->getData().yawRate));
  sdk->ui->le_hp_ra->setText(
    QString::number(vehicle->missionManager->hpMission->getData().radius));
  sdk->ui->cb_hp_cl->setChecked(
    vehicle->missionManager->hpMission->getData().clockwise ? true : false);
  sdk->ui->cb_hp_yaw->setCurrentIndex(
    vehicle->missionManager->hpMission->getData().yawMode);
}

void
HotpointPanel::on_btn_hotPoint_start_clicked()
{
  vehicle->missionManager->hpMission->setHotPoint(
    ui->le_hp_lo->text().toFloat() * DEG2RAD,
    ui->le_hp_la->text().toFloat() * DEG2RAD, ui->le_hp_al->text().toFloat());
  vehicle->missionManager->hpMission->setClockwise(ui->cb_hp_cl->isChecked());
  vehicle->missionManager->hpMission->setYawRate(
    ui->le_hp_pa->text().toFloat());
  vehicle->missionManager->hpMission->setRadius(ui->le_hp_ra->text().toFloat());
  vehicle->missionManager->hpMission->setYawMode(
    (HotpointMission::YawMode)ui->cb_hp_yaw->currentIndex());
  vehicle->missionManager->hpMission->start();
}

void
HotpointPanel::on_btn_hp_setPal_clicked()
{
  vehicle->missionManager->hpMission->updateYawRate(
    ui->le_hp_pa->text().toFloat(), ui->cb_hp_cl->isChecked());
}

void
HotpointPanel::on_btn_hotPoint_current_clicked()
{
  ui->le_hp_lo->setText(QString::number(
    vehicle->broadcast->getGlobalPosition().longitude / DEG2RAD));
  ui->le_hp_la->setText(QString::number(
    vehicle->broadcast->getGlobalPosition().latitude / DEG2RAD));
  ui->le_hp_al->setText(
    QString::number(vehicle->broadcast->getGlobalPosition().altitude));
}

void
HotpointPanel::on_btn_hotPoint_stop_clicked()
{
  vehicle->missionManager->hpMission->stop();
}

void
HotpointPanel::on_btn_hp_data_clicked()
{
  vehicle->missionManager->hpMission->getHotpointSettings(hotpointReadCallback, this);
}

void
HotpointPanel::on_btn_hp_setRadius_clicked()
{
  vehicle->missionManager->hpMission->updateRadius(
    ui->le_hp_ra->text().toFloat());
}

void
HotpointPanel::on_btn_hp_setYaw_clicked()
{
  ui->cb_hp_yaw->setCurrentIndex(1);
  vehicle->missionManager->hpMission->resetYaw();
}

void
HotpointPanel::on_btn_hp_pause_clicked(bool checked)
{
  if (checked)
  {
    vehicle->missionManager->hpMission->pause();
    ui->btn_hp_pause->setText("Resume");
  }
  else
  {
    vehicle->missionManager->hpMission->resume();
    ui->btn_hp_pause->setText("Pause");
  }
}

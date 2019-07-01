#include "mfio_panel.hpp"
#include "ui_mfio_panel.h"

using namespace DJI::OSDK;

MFIOPanel::MFIOPanel(QWidget* parent, Vehicle* vehicle)
  : QWidget(parent)
  , ui(new Ui::MFIOPanel)
{
  ui->setupUi(this);
  this->vehicle = vehicle;
}

MFIOPanel::~MFIOPanel()
{
  delete ui;
}

void
MFIOPanel::on_btn_init_clicked()
{
  vehicle->mfio->config(
    (DJI::OSDK::MFIO::MODE)ui->cb_mode->currentIndex(),
    (DJI::OSDK::MFIO::CHANNEL)ui->cb_channel->currentIndex(),
    QString(ui->le_val->text()).toUInt(), QString(ui->le_freq->text()).toUInt(),
    0, 0);

  //! Remove this channel from the set/get comboBoxes if it was already there
  if (inputMap.contains(ui->cb_channel->currentIndex()))
  {
    ui->cb_channel_get->removeItem(
      inputMap.indexOf(ui->cb_channel->currentIndex()));
    inputMap.remove(inputMap.indexOf(ui->cb_channel->currentIndex()));
  }
  if (outputMap.contains(ui->cb_channel->currentIndex()))
  {
    ui->cb_channel_set->removeItem(
      outputMap.indexOf(ui->cb_channel->currentIndex()));
    outputMap.remove(outputMap.indexOf(ui->cb_channel->currentIndex()));
  }

  if ((DJI::OSDK::MFIO::MODE)ui->cb_mode->currentIndex() ==
        (DJI::OSDK::MFIO::MODE_ADC) ||
      (DJI::OSDK::MFIO::MODE)ui->cb_mode->currentIndex() ==
        (DJI::OSDK::MFIO::MODE_PWM_IN) ||
      (DJI::OSDK::MFIO::MODE)ui->cb_mode->currentIndex() ==
        (DJI::OSDK::MFIO::MODE_GPIO_IN))
  {
    ui->cb_channel_get->addItem(
      QString("Channel_F%1").arg(ui->cb_channel->currentIndex() + 1));
    inputMap.push_back(ui->cb_channel->currentIndex());
  }
  else
  {
    ui->cb_channel_set->addItem(
      QString("Channel_F%1").arg(ui->cb_channel->currentIndex() + 1));

    outputMap.push_back(ui->cb_channel->currentIndex());
  }
}

void
MFIOPanel::on_cb_mode_currentIndexChanged(const QString& arg1)
{
  if (arg1 != "PWMO")
  {
    ui->le_freq->setText("0");
    ui->le_freq->setEnabled(false);
  }
  else
  {
    ui->label_2->setText("On Time (us) :");
    ui->le_freq->setEnabled(true);
  }

  if (arg1 == "GPO")
  {
    ui->label_2->setText("Value :");
  }

  if (arg1 == "PWMI" || arg1 == "ADC" || arg1 == "GPI")
  {
    ui->le_val->setText("0");
    ui->le_val->setEnabled(false);
  }
  else
  {
    ui->le_val->setEnabled(true);
  }
}

void
MFIOPanel::on_btn_get_clicked()
{
  vehicle->mfio->getValue(
    (DJI::OSDK::MFIO::CHANNEL)inputMap[ui->cb_channel_get->currentIndex()],
    MFIOPanel::getValueCallback, this);
  QThread::msleep(200);
}

void
MFIOPanel::on_btn_set_clicked()
{
  vehicle->mfio->setValue(
    (DJI::OSDK::MFIO::CHANNEL)outputMap[ui->cb_channel_set->currentIndex()],
    ui->le_val_set->text().toUInt());
}

void
MFIOPanel::getValueCallback(Vehicle* vehicle, RecvContainer recvFrame,
                            UserData data)
{
  MFIOPanel* mfioPanel = (MFIOPanel*)data;

  uint16_t ack_length =
    recvFrame.recvInfo.len - static_cast<uint16_t>(Protocol::PackageMin);
  uint8_t* ackPtr = recvFrame.recvData.raw_ack_array;

  uint8_t  result;
  uint32_t value;

  memcpy((uint8_t*)&result, ackPtr, 1);
  memcpy((uint8_t*)&value, ackPtr + 1, 4);

  if (result != 0)
  {
    mfioPanel->ui->le_val_get->setText(
      QString("Failed to get value: error %1").arg(result));
  }
  mfioPanel->ui->le_val_get->setText(QString("%1").arg(value));
}

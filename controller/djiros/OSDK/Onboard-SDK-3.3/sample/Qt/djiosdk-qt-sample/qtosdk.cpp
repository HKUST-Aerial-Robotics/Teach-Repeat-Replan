#include "qtosdk.hpp"
#include "ui_camera_gimbal_control_panel.h"
#include "ui_flight_control_panel.h"
#include "ui_hotpoint_panel.h"
#include "ui_qtosdk.h"
#include "ui_qwaypoints.h"

using namespace DJI;
using namespace DJI::OSDK;

qtOsdk::qtOsdk(QWidget* parent)
  : QMainWindow(parent)
  , ui(new Ui::qtOsdk)
{
  ui->setupUi(this);
  refreshPort();
  vehicle = 0;
}

qtOsdk::~qtOsdk()
{
  delete ui;
  if (vehicle)
    delete vehicle;
}

void
qtOsdk::readAppIDKey()
{
  QFile f(":/UserConfig.txt");
  if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
    DSTATUS ("No UserConfig.txt file found");
  else
  {
    while (!f.atEnd())
    {
      QByteArray line = f.readLine();
      if (line.startsWith("ID:"))
        ui->appIDInput->setText(line.remove(0, 3));
      else if (line.startsWith("KEY:"))
        ui->keyInput->setText(line.remove(0, 4));
    }
    f.close();
  }
}

void
qtOsdk::refreshPort()
{
  ui->portSelection->clear();
  QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
  QStringList            list;
  for (int i = 0; i < ports.length(); ++i)
  {
    list.append(ports[i].portName());
  }
  if (ports.length()== 0)
      list.append("Connect Serial");

  ui->portSelection->addItems(list);
}

void
qtOsdk::on_initVehicle_clicked()
{
  if (ui->portSelection->currentText() == "Connect Serial")
    return;

  readAppIDKey();

  vehicle = new Vehicle(ui->portSelection->currentText().toStdString().c_str(),
                        ui->baudRateInput->text().toInt(), true);
  if (vehicle)
  {
    emit changeInitButton("Vehicle Initialized", true);
    QThread::msleep(100);
    // Initialize all the other parts of the SDK
    initComponents();
  }
  else
  {
    emit changeInitButton("Init Failed", false);
  }
}

void
qtOsdk::on_activateButton_clicked()
{
  Vehicle::ActivateData activateData;
  activateData.ID = static_cast<uint32_t>(ui->appIDInput->text().toUInt());
  QByteArray key  = ui->keyInput->text().toLocal8Bit();
  char       keyArray[65];
  activateData.encKey = keyArray;
  strcpy(activateData.encKey, key.toStdString().c_str());
  activateData.version = vehicle->getFwVersion();
  vehicle->activate(&activateData, activateCallback, this);
}

void
qtOsdk::activateCallback(Vehicle* vehicle, RecvContainer recvFrame,
                         UserData userData)
{
  qtOsdk*        sdk = (qtOsdk*)userData;
  ACK::ErrorCode ack_data;
  if (recvFrame.recvInfo.len - Protocol::PackageMin <= 2)
  {
    ack_data.data = recvFrame.recvData.ack;
    ack_data.info = recvFrame.recvInfo;

    if (ACK::getError(ack_data))
    {
      emit sdk->changeActivateButton(
        QString("Activation Error: %1").arg(ack_data.data), false);
      ACK::getErrorCodeMessage(ack_data, __func__);
    }
    else
    {
      emit sdk->changeActivateButton(QString("Activation Successful"), true);
    }
  }
  else
  {
    emit sdk->changeActivateButton(QString("ACK Decode Error"), false);
  }

  // Do the stuff the OSDK callback does, since it is private and we cannot call
  // it here
  if (ack_data.data == OpenProtocol::ErrorCode::ActivationACK::SUCCESS &&
      vehicle->getAccountData().encKey)
  {
    vehicle->protocolLayer->setKey(vehicle->getAccountData().encKey);
  }
}

void
qtOsdk::on_obtainCtrlButton_clicked()
{
  if (ui->obtainCtrlButton->text() == "Release Control")
    vehicle->releaseCtrlAuthority(qtOsdk::setControlCallback, this);
  else
    vehicle->obtainCtrlAuthority(qtOsdk::setControlCallback, this);
}

void
qtOsdk::setControlCallback(Vehicle* vehicle, RecvContainer recvFrame,
                           UserData userData)
{
  qtOsdk*        sdk = (qtOsdk*)userData;
  ACK::ErrorCode ack;
  ack.data = OpenProtocol::ErrorCode::CommonACK::NO_RESPONSE_ERROR;

  if (recvFrame.recvInfo.len - Protocol::PackageMin <= sizeof(uint16_t))
  {
    ack.data = recvFrame.recvData.ack;
    ack.info = recvFrame.recvInfo;
  }
  else
  {
    DERROR("ACK is exception, sequence %d\n", recvFrame.recvInfo.seqNumber);
    return;
  }

  if (ack.data ==
      OpenProtocol::ErrorCode::ControlACK::SetControl::RC_MODE_ERROR)
  {
    if (sdk)
    {
      sdk->ui->ctrlStatus->setText("Switch to mode P");
    }
    else
      DERROR("SDK not initialized.");
  }
  if (ack.data ==
      OpenProtocol::ErrorCode::ControlACK::SetControl::OBTAIN_CONTROL_SUCCESS)
  {
    if (sdk)
    {
      emit sdk->changeControlAuthorityStatus("Obtained Control");
      sdk->ui->obtainCtrlButton->setText("Release Control");
    }
    else
      DERROR("SDK not initialized.");
  }
  if (ack.data ==
      OpenProtocol::ErrorCode::ControlACK::SetControl::RELEASE_CONTROL_SUCCESS)
  {
    if (sdk)
    {
      emit sdk->changeControlAuthorityStatus("Released Control");

      sdk->ui->obtainCtrlButton->setText("Obtain Control");
    }
    else
      DERROR("SDK not initialized.");
  }
  if (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                    OBTAIN_CONTROL_IN_PROGRESS)
  {
    vehicle->obtainCtrlAuthority(qtOsdk::setControlCallback, sdk);
    emit sdk->changeControlAuthorityStatus("Obtaining Control...");
  }
  if (ack.data == OpenProtocol::ErrorCode::ControlACK::SetControl::
                    RELEASE_CONTROL_IN_PROGRESS)
  {
    vehicle->releaseCtrlAuthority(qtOsdk::setControlCallback, sdk);
    emit sdk->changeControlAuthorityStatus("Releasing Control...");
  }
}

void
qtOsdk::ctrlStatusChanged(QString valueToDisplay)
{
  this->ui->ctrlStatus->setText(valueToDisplay);
}

void
qtOsdk::initComponents()
{
  this->flightControl = new FlightControlPanel(0, this->vehicle);
  ui->componentTabs->addTab(flightControl, QString("Flight Control"));
  QObject::connect(flightControl, SIGNAL(changeCommandStatus(QString)),
                   flightControl, SLOT(commandStatusChanged(QString)));

  this->cameraGimbalControl = new CameraGimbalControl(0, this->vehicle);
  ui->componentTabs->addTab(cameraGimbalControl,
                            QString("Camera/Gimbal Control"));

  this->broadcast = new Broadcast(0, this->vehicle);
  ui->componentTabs->addTab(broadcast, QString("Telemetry: Broadcast"));

  this->subscribe = new SubscribePanel(0, this->vehicle);
  ui->componentTabs->addTab(subscribe, QString("Telemetry: Subscribe"));

  this->waypoints = new QWaypoints(0, this->vehicle);
  ui->componentTabs->addTab(waypoints, QString("GPS Missions"));

  this->hotpointPanel = new HotpointPanel(0, this->vehicle);
  waypoints->getMissionUi()->tabWidget->addTab(hotpointPanel,
                                               QString("Hotpoints"));

  this->mfioPanel = new MFIOPanel(0, this->vehicle);
  ui->componentTabs->addTab(mfioPanel, QString("MFIO"));
}

void
qtOsdk::initFinished(QString initStatus, bool initResult)
{
  if (initResult)
  {
    ui->initVehicle->setStyleSheet("background-color: qlineargradient(x1: 0, "
                                   "y1: 0, x2: 0, y2: 1, stop: 0 #44a8f2, "
                                   "stop: 1 #44a8f2); color:white");
    ui->initVehicle->setText(initStatus);
    ui->hwVersionDisplay->setText(QString(vehicle->getHwVersion()));
    Version::FirmWare fwVersion = vehicle->getFwVersion();
    uint8_t           ver1      = (fwVersion >> 24) & 0x000000ff;
    uint8_t           ver2      = (fwVersion >> 16) & 0x000000ff;
    uint8_t           ver3      = (fwVersion >> 8) & 0x000000ff;
    uint8_t           ver4      = fwVersion & 0x000000ff;
    ui->fwVersionDisplay->setText(
      QString("%1.%2.%3.%4").arg(ver1).arg(ver2).arg(ver3).arg(ver4));
  }
  else
  {
    ui->initVehicle->setStyleSheet("background-color: qlineargradient(x1: 0, "
                                   "y1: 0, x2: 0, y2: 1, stop: 0 red, stop: 1 "
                                   "red); color:white");
    ui->initVehicle->setText(initStatus);
  }
}

void
qtOsdk::activateFinished(QString activateStatus, bool activateResult)
{
  if (!activateResult)
  {
    ui->activateButton->setText(activateStatus);
    ui->activateButton->setStyleSheet("background-color: qlineargradient(x1: "
                                      "0, y1: 0, x2: 0, y2: 1, stop: 0 "
                                      "#B4B4B4, stop: 1 #B4B4B4); color:white");
  }
  else
  {
    ui->activateButton->setText(activateStatus);
    ui->activateButton->setStyleSheet("background-color:qlineargradient(x1: 0, "
                                      "y1: 0, x2: 0, y2: 1, stop: 0 #44a8f2, "
                                      "stop: 1 #44a8f2); color:white");
  }
}

void
qtOsdk::on_componentTabs_currentChanged(int index)
{
  for (int i = 0; i < ui->componentTabs->count(); i++)
    if (i != index)
      ui->componentTabs->widget(i)->setSizePolicy(QSizePolicy::Ignored,
                                                  QSizePolicy::Ignored);

  ui->componentTabs->widget(index)->setSizePolicy(QSizePolicy::Preferred,
                                                  QSizePolicy::Preferred);
  ui->componentTabs->widget(index)->resize(
    ui->componentTabs->widget(index)->sizeHint());
  ui->componentTabs->widget(index)->adjustSize();
  resize(sizeHint());
  adjustSize();
}

void qtOsdk::on_portSelection_activated(const QString &arg1)
{
    refreshPort();
}

#include "subscribe_panel.hpp"
#include "ui_subscribe_panel.h"

#include <QDebug>
#include <QTableWidget>

#define INLINE_NAME(X) #X

using namespace DJI::OSDK;

SubscribePanel::SubscribePanel(QWidget* parent, Vehicle* vehiclePtr)
  : QWidget(parent)
  , ui(new Ui::SubscribePanel)
{
  ui->setupUi(this);
  vehicle = vehiclePtr;

  //! @note init table;
  ui->tableWidget->setColumnCount(vehicle->subscribe->MAX_NUMBER_OF_PACKAGE +
                                  2);
  ui->tableWidget->setRowCount(Telemetry::TOTAL_TOPIC_NUMBER + 2);

  for (int i = 0; i < ui->tableWidget->rowCount(); ++i)
    for (int j = 0; j < ui->tableWidget->columnCount(); ++j)
    {
      QTableWidgetItem* t = new QTableWidgetItem();
      if (j == 0 && i == 0)
        t->setText("Name");
      if (j == 0 && i == 1)
        t->setText("TimeStamp");
      if (j == 0 && i > 1)
      {
        switch (i - 2)
        {
          // clang-format off
            case 0  :t->setText(INLINE_NAME(QUATERNION                ));  break;
            case 1  :t->setText(INLINE_NAME(ACCELERATION_GROUND       ));  break;
            case 2  :t->setText(INLINE_NAME(ACCELERATION_BODY         ));  break;
            case 3  :t->setText(INLINE_NAME(ACCELERATION_RAW          ));  break;
            case 4  :t->setText(INLINE_NAME(VELOCITY                  ));  break;
            case 5  :t->setText(INLINE_NAME(ANGULAR_RATE_FUSED        ));  break;
            case 6  :t->setText(INLINE_NAME(ANGULAR_RATE_RAW          ));  break;
            case 7  :t->setText(INLINE_NAME(ALTITUDE_FUSIONED         ));  break;
            case 8  :t->setText(INLINE_NAME(ALTITUDE_BAROMETER        ));  break;
            case 9  :t->setText(INLINE_NAME(HEIGHT_HOMEPOINT          ));  break;
            case 10 :t->setText(INLINE_NAME(HEIGHT_FUSION             ));  break;
            case 11 :t->setText(INLINE_NAME(GPS_FUSED                 ));  break;
            case 12 :t->setText(INLINE_NAME(GPS_DATE                  ));  break;
            case 13 :t->setText(INLINE_NAME(GPS_TIME                  ));  break;
            case 14 :t->setText(INLINE_NAME(GPS_POSITION              ));  break;
            case 15 :t->setText(INLINE_NAME(GPS_VELOCITY              ));  break;
            case 16 :t->setText(INLINE_NAME(GPS_DETAILS               ));  break;
            case 17 :t->setText(INLINE_NAME(RTK_POSITION              ));  break;
            case 18 :t->setText(INLINE_NAME(RTK_VELOCITY              ));  break;
            case 19 :t->setText(INLINE_NAME(RTK_YAW                   ));  break;
            case 20 :t->setText(INLINE_NAME(RTK_POSITION_INFO         ));  break;
            case 21 :t->setText(INLINE_NAME(RTK_YAW_INFO              ));  break;
            case 22 :t->setText(INLINE_NAME(COMPASS                   ));  break;
            case 23 :t->setText(INLINE_NAME(RC                        ));  break;
            case 24 :t->setText(INLINE_NAME(GIMBAL_ANGLES             ));  break;
            case 25 :t->setText(INLINE_NAME(GIMBAL_STATUS             ));  break;
            case 26 :t->setText(INLINE_NAME(STATUS_FLIGHT             ));  break;
            case 27 :t->setText(INLINE_NAME(STATUS_DISPLAYMODE        ));  break;
            case 28 :t->setText(INLINE_NAME(STATUS_LANDINGGEAR        ));  break;
            case 29 :t->setText(INLINE_NAME(STATUS_MOTOR_START_ERROR  ));  break;
            case 30 :t->setText(INLINE_NAME(BATTERY_INFO              ));  break;
            case 31 :t->setText(INLINE_NAME(CONTROL_DEVICE            ));  break;
            case 32 :t->setText(INLINE_NAME(HARD_SYNC                 ));  break;
            case 33 :t->setText(INLINE_NAME(GPS_SIGNAL_LEVEL          ));  break;
            case 34 :t->setText(INLINE_NAME(GPS_CONTROL_LEVEL         ));  break;
            // clang-format on
        }
      }
      if (j == 1 && i == 0)
        t->setText(QString("Value"));
      if (j > 1 && i > 0)
        t->setCheckState(Qt::Unchecked);
      if (j > 1 && i == 0)
        t->setText(QString("Package%1").arg(j - 2));

      ui->tableWidget->setItem(i, j, t);
    }

  //  //! @note init package column
  //  for (int i = 0; i < vehicle->subscribe->MAX_NUMBER_OF_PACKAGE; ++i) {
  //    ;//ui->cb_pkg->addItem(QString::number(i));
  //  }
  //  //! @note init rows
  //  for (int i = 0; i < Telemetry::TOTAL_TOPIC_NUMBER; ++i)
  //    ;

  ui->tableWidget->horizontalHeader()->setSectionResizeMode(
    QHeaderView::ResizeToContents);
  ui->tableWidget->verticalHeader()->setSectionResizeMode(
    QHeaderView::ResizeToContents);
}

SubscribePanel::~SubscribePanel()
{
  delete ui;
}

void
SubscribePanel::display(Telemetry::TopicName topicName, uint32_t id)
{
  switch (topicName)
  {
    case Telemetry::TOPIC_QUATERNION:
    {
      Telemetry::Quaternion q =
        vehicle->subscribe->getValue<Telemetry::TOPIC_QUATERNION>();
      ui->tableWidget->item(2 + (int)Telemetry::TOPIC_QUATERNION, 1)
        ->setText(QString("q0: %1\nq1: %2\nq2: %3\nq3: %4")
                    .arg(q.q0, 0, 'f', 3)
                    .arg(q.q1, 0, 'f', 3)
                    .arg(q.q2, 0, 'f', 3)
                    .arg(q.q3, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_ACCELERATION_GROUND:
    {
      Telemetry::Vector3f ag =
        vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_GROUND>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("x: %1\ny: %2\nz: %3")
                    .arg(ag.x, 0, 'f', 3)
                    .arg(ag.y, 0, 'f', 3)
                    .arg(ag.z, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_ACCELERATION_BODY:
    {
      Telemetry::Vector3f ab =
        vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_BODY>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("x: %1\ny: %2\nz: %3")
                    .arg(ab.x, 0, 'f', 3)
                    .arg(ab.y, 0, 'f', 3)
                    .arg(ab.z, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_ACCELERATION_RAW:
    {
      Telemetry::Vector3f ar =
        vehicle->subscribe->getValue<Telemetry::TOPIC_ACCELERATION_RAW>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("x: %1\ny: %2\nz: %3")
                    .arg(ar.x, 0, 'f', 3)
                    .arg(ar.y, 0, 'f', 3)
                    .arg(ar.z, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_VELOCITY:
    {
      Telemetry::Velocity v =
        vehicle->subscribe->getValue<Telemetry::TOPIC_VELOCITY>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("x: %1\ny: %2\nz: %3\nhealth: %4")
                    .arg(v.data.x, 0, 'f', 3)
                    .arg(v.data.y, 0, 'f', 3)
                    .arg(v.data.z, 0, 'f', 3)
                    .arg(v.info.health));
    }
    break;
    case Telemetry::TOPIC_ANGULAR_RATE_FUSIONED:
    {
      Telemetry::Vector3f pf =
        vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_FUSIONED>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("x: %1\ny: %2\nz: %3")
                    .arg(pf.x, 0, 'f', 3)
                    .arg(pf.y, 0, 'f', 3)
                    .arg(pf.z, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_ANGULAR_RATE_RAW:
    {
      Telemetry::Vector3f pr =
        vehicle->subscribe->getValue<Telemetry::TOPIC_ANGULAR_RATE_RAW>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("x: %1\ny: %2\nz: %3")
                    .arg(pr.x, 0, 'f', 3)
                    .arg(pr.y, 0, 'f', 3)
                    .arg(pr.z, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_ALTITUDE_FUSIONED:
    {
      float32_t d =
        vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_FUSIONED>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("%1").arg(d, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_ALTITUDE_BAROMETER:
    {
      float32_t d =
        vehicle->subscribe->getValue<Telemetry::TOPIC_ALTITUDE_BAROMETER>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("%1").arg(d, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_HEIGHT_HOMEPOINT:
    {
      float32_t d =
        vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_HOMEPOINT>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("%1").arg(d, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_HEIGHT_FUSION:
    {
      float32_t d =
        vehicle->subscribe->getValue<Telemetry::TOPIC_HEIGHT_FUSION>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("%1").arg(d, 0, 'f', 3));
    }
    break;
    case Telemetry::TOPIC_GPS_FUSED:
    {
      Telemetry::TypeMap<Telemetry::TOPIC_GPS_FUSED>::type gpsPos =
        vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_FUSED>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("Lon: %1\n Lat : %2\n Alt : %3\n satNum: %4")
                    .arg(gpsPos.longitude)
                    .arg(gpsPos.latitude)
                    .arg(gpsPos.altitude)
                    .arg(gpsPos.visibleSatelliteNumber));
    }
    break;
    case Telemetry::TOPIC_GPS_DATE:
    {
      uint32_t d = vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_DATE>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("%1").arg(d));
    }
    break;
    case Telemetry::TOPIC_GPS_TIME:
    {
      uint32_t d = vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_TIME>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("%1").arg(d));
    }
    break;
    case Telemetry::TOPIC_GPS_POSITION:
    {
      Telemetry::Vector3d p =
        vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_POSITION>();
      ui->tableWidget->item(2 + (int)topicName, 1)
        ->setText(QString("longitude: %1\n latitude : %2\n hmsl : %3")
                    .arg(p.x)
                    .arg(p.y)
                    .arg(p.z));
    }
    break;
    //! @todo implement
    // clang-format off
    case Telemetry::TOPIC_GPS_VELOCITY              :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_GPS_VELOCITY>::type gpsVel = vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_VELOCITY>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("VelN: %1\n VelE : %2\n VelD : %3")
                        .arg(gpsVel.x)
                        .arg(gpsVel.y)
                        .arg(gpsVel.z));
    }
    break;
    case Telemetry::TOPIC_GPS_DETAILS               :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_GPS_DETAILS>::type gpsDetail = vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_DETAILS>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("hdop: %1\n pdop : %2\n fix : %3\n gnssStatus : %4\n hacc : %5\n sacc : %6\n usedGPS : %7\n usedGLN : %8\n numSat : %9\n counter : %10")
                        .arg(gpsDetail.hdop)
                        .arg(gpsDetail.pdop)
                        .arg(gpsDetail.fix)
                        .arg(gpsDetail.gnssStatus)
                        .arg(gpsDetail.hacc)
                        .arg(gpsDetail.sacc)
                        .arg(gpsDetail.usedGPS)
                        .arg(gpsDetail.usedGLN)
                        .arg(gpsDetail.NSV)
                        .arg(gpsDetail.GPScounter));
    }
    break;
    case Telemetry::TOPIC_RTK_POSITION              :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_RTK_POSITION>::type rtkDetail = vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("Lat: %1\n Lon : %2\n HMSL : %3")
                        .arg(rtkDetail.latitude)
                        .arg(rtkDetail.longitude)
                        .arg(rtkDetail.HFSL));
    }
    break;
    case Telemetry::TOPIC_RTK_VELOCITY              :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_RTK_VELOCITY>::type rtkVel = vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_VELOCITY>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("VelN: %1\n VelE : %2\n VelD : %3")
                        .arg(rtkVel.x)
                        .arg(rtkVel.y)
                        .arg(rtkVel.z));
    }
    break;
    case Telemetry::TOPIC_RTK_YAW                   :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_RTK_YAW>::type rtkYaw = vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("Yaw: %1")
                        .arg(rtkYaw));
    }
    break;
    case Telemetry::TOPIC_RTK_POSITION_INFO         :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_RTK_POSITION_INFO>::type rtkPosInfo = vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_POSITION_INFO>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("Position Flag: %1")
                        .arg((int)rtkPosInfo));
    }
    break;
    case Telemetry::TOPIC_RTK_YAW_INFO              :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_RTK_YAW_INFO>::type rtkYawInfo = vehicle->subscribe->getValue<Telemetry::TOPIC_RTK_YAW_INFO>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("Yaw Flag: %1")
                        .arg((int)rtkYawInfo));
    }
    break;
    case Telemetry::TOPIC_COMPASS                   :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_COMPASS>::type compass = vehicle->subscribe->getValue<Telemetry::TOPIC_COMPASS>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("X: %1\n Y : %2\n Z : %3")
                        .arg(compass.x)
                        .arg(compass.y)
                        .arg(compass.z));
    }
    break;
    case Telemetry::TOPIC_RC                        :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_RC>::type rc = vehicle->subscribe->getValue<Telemetry::TOPIC_RC>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("Roll: %1\n Pitch : %2\n Yaw : %3\n Thr: %4\n Mode: %5\n Gear: %6")
                        .arg(rc.roll)
                        .arg(rc.pitch)
                        .arg(rc.yaw)
                        .arg(rc.throttle)
                        .arg(rc.mode)
                        .arg(rc.gear));
    }
    break;
    case Telemetry::TOPIC_GIMBAL_ANGLES             :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_ANGLES>::type gimbal = vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_ANGLES>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("Roll: %1\n Pitch: %2\n Yaw : %3")
                        .arg(gimbal.x)
                        .arg(gimbal.y)
                        .arg(gimbal.z));
    }
    break;
    case Telemetry::TOPIC_GIMBAL_STATUS             :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_GIMBAL_STATUS>::type gimbalStatus = vehicle->subscribe->getValue<Telemetry::TOPIC_GIMBAL_STATUS>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("mountStatus: %1\n isBusy : %2\n pitchLimit : %3\n rollLimit : %4\n yawLimit : %5\n calibrating: %6\n "
                            "prevCalibrationResult: %7\n installDirection : %8\n disabled_mvo : %9\n gear_show_unable : %10\n gyroFault: %11\n"
                            " escPitchFault : %12\n escRollFault : %13\n escYawFault : %14\n droneDataFault : %15\n initUnfinished : %16\n "
                            "FWUpdating : %17\n")
                        .arg(gimbalStatus.mountStatus)
                        .arg(gimbalStatus.isBusy)
                        .arg(gimbalStatus.pitchLimited)
                        .arg(gimbalStatus.rollLimited)
                        .arg(gimbalStatus.yawLimited)
                        .arg(gimbalStatus.calibrating)
                        .arg(gimbalStatus.prevCalibrationgResult)
                        .arg(gimbalStatus.installedDirection)
                        .arg(gimbalStatus.disabled_mvo)
                        .arg(gimbalStatus.gear_show_unable)
                        .arg(gimbalStatus.gyroFalut)
                        .arg(gimbalStatus.escPitchFault)
                        .arg(gimbalStatus.escRollFault)
                        .arg(gimbalStatus.escYawFault)
                        .arg(gimbalStatus.droneDataFault)
                        .arg(gimbalStatus.initUnfinished)
                        .arg(gimbalStatus.FWUpdating));
    }
    break;
    case Telemetry::TOPIC_STATUS_FLIGHT             :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_STATUS_FLIGHT>::type flightStatus = vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_FLIGHT>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("status: %1")
                        .arg((int)flightStatus));
    }
    break;
    case Telemetry::TOPIC_STATUS_DISPLAYMODE        :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_STATUS_DISPLAYMODE>::type modeStatus = vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_DISPLAYMODE>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("mode: %1")
                        .arg((int)modeStatus));
    }
    break;
    case Telemetry::TOPIC_STATUS_LANDINGGEAR        :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_STATUS_LANDINGGEAR>::type gearStatus = vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_LANDINGGEAR>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("gear: %1")
                        .arg((int)gearStatus));
    }
    break;
    case Telemetry::TOPIC_STATUS_MOTOR_START_ERROR  :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_STATUS_MOTOR_START_ERROR>::type motorError = vehicle->subscribe->getValue<Telemetry::TOPIC_STATUS_MOTOR_START_ERROR>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("motorError: %1")
                        .arg((int)motorError));
    }
    break;
    case Telemetry::TOPIC_BATTERY_INFO              :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_BATTERY_INFO>::type batt = vehicle->subscribe->getValue<Telemetry::TOPIC_BATTERY_INFO>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("Capacity: %1\n Voltage: %2\n Current : %3\n Percentage: %4")
                        .arg(batt.capacity)
                        .arg(batt.voltage)
                        .arg(batt.current)
                        .arg(batt.percentage));
    }
    break;
    case Telemetry::TOPIC_CONTROL_DEVICE            :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_CONTROL_DEVICE>::type ctrlDev = vehicle->subscribe->getValue<Telemetry::TOPIC_CONTROL_DEVICE>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("CtrlMode: %1\n DeviceStatus: %2\n FlightStatus : %3\n vrcStatus: %4")
                        .arg(ctrlDev.controlMode)
                        .arg(ctrlDev.deviceStatus)
                        .arg(ctrlDev.flightStatus)
                        .arg(ctrlDev.vrcStatus));
    }
    break;
    case Telemetry::TOPIC_HARD_SYNC             :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_HARD_SYNC>::type hardSync = vehicle->subscribe->getValue<Telemetry::TOPIC_HARD_SYNC>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("time2p5ms: %1\n time1ns : %2\n resetTime2p5ms : %3\n index : %4\n flag : %5\n "
                            "q0: %6\n q1: %7\n q2 : %8\n q3 : %9\n accX : %10\n accY: %11\n"
                            "accZ : %12\n gyroX : %13\n gyroY : %14\n gyroZ : %15\n")
                        .arg(hardSync.ts.time2p5ms)
                        .arg(hardSync.ts.time1ns)
                        .arg(hardSync.ts.resetTime2p5ms)
                        .arg(hardSync.ts.index)
                        .arg(hardSync.ts.flag)
                        .arg(hardSync.q.q0)
                        .arg(hardSync.q.q1)
                        .arg(hardSync.q.q2)
                        .arg(hardSync.q.q3)
                        .arg(hardSync.a.x)
                        .arg(hardSync.a.y)
                        .arg(hardSync.a.z)
                        .arg(hardSync.w.x)
                        .arg(hardSync.w.y)
                        .arg(hardSync.w.z));

    }
    break;
    case Telemetry::TOPIC_GPS_SIGNAL_LEVEL  :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_GPS_SIGNAL_LEVEL>::type gpsLevel = vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_SIGNAL_LEVEL>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("signalLevel: %1")
                        .arg((int)gpsLevel));
    }
    break;
    case Telemetry::TOPIC_GPS_CONTROL_LEVEL  :
    {
      Telemetry::TypeMap<Telemetry::TOPIC_GPS_CONTROL_LEVEL>::type gpsLevel = vehicle->subscribe->getValue<Telemetry::TOPIC_GPS_CONTROL_LEVEL>();
      ui->tableWidget->item(2 + (int)topicName, 1)
          ->setText(QString("controlLevel: %1")
                        .arg((int)gpsLevel));
    }
    break;
    default: break;
      // clang-format on
  }
}

void
SubscribePanel::on_btn_match_clicked()
{
  vehicle->subscribe->verify();
}

void
SubscribePanel::pkg0UnpackCallback(Vehicle*      vehiclePtr,
                                   RecvContainer rcvContainer,
                                   UserData      userData)
{
  SubscribePanel* subscribePanel = (SubscribePanel*)userData;
  int             numTopics      = subscribePanel->pkg0Indices.size();
  for (int i = 0; i < numTopics; i++)
  {
    subscribePanel->display(
      static_cast<Telemetry::TopicName>(subscribePanel->pkg0Indices[i] - 2), 0);
  }
}

void
SubscribePanel::pkg1UnpackCallback(Vehicle*      vehiclePtr,
                                   RecvContainer rcvContainer,
                                   UserData      userData)
{
  SubscribePanel* subscribePanel = (SubscribePanel*)userData;
  int             numTopics      = subscribePanel->pkg1Indices.size();
  for (int i = 0; i < numTopics; i++)
  {
    subscribePanel->display(
      static_cast<Telemetry::TopicName>(subscribePanel->pkg1Indices[i] - 2), 0);
  }
}

void
SubscribePanel::pkg2UnpackCallback(Vehicle*      vehiclePtr,
                                   RecvContainer rcvContainer,
                                   UserData      userData)
{
  SubscribePanel* subscribePanel = (SubscribePanel*)userData;
  int             numTopics      = subscribePanel->pkg2Indices.size();
  for (int i = 0; i < numTopics; i++)
  {
    subscribePanel->display(
      static_cast<Telemetry::TopicName>(subscribePanel->pkg2Indices[i] - 2), 0);
  }
}

void
SubscribePanel::pkg3UnpackCallback(Vehicle*      vehiclePtr,
                                   RecvContainer rcvContainer,
                                   UserData      userData)
{
  SubscribePanel* subscribePanel = (SubscribePanel*)userData;
  int             numTopics      = subscribePanel->pkg3Indices.size();
  for (int i = 0; i < numTopics; i++)
  {
    subscribePanel->display(
      static_cast<Telemetry::TopicName>(subscribePanel->pkg3Indices[i] - 2), 0);
  }
}

void
SubscribePanel::pkg4UnpackCallback(Vehicle*      vehiclePtr,
                                   RecvContainer rcvContainer,
                                   UserData      userData)
{
  SubscribePanel* subscribePanel = (SubscribePanel*)userData;
  int             numTopics      = subscribePanel->pkg4Indices.size();
  for (int i = 0; i < numTopics; i++)
  {
    subscribePanel->display(
      static_cast<Telemetry::TopicName>(subscribePanel->pkg4Indices[i] - 2), 0);
  }
}

#include <QDebug>

void
SubscribePanel::on_tableWidget_itemChanged(QTableWidgetItem* item)
{
  if (item->row() > 1 && item->column() > 1)
    if (item->checkState() == Qt::Checked)
    {
      // display(item->row() - 2, item->column() - 2);
      ui->tableWidget->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
      ui->tableWidget->verticalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    }
    else
    {
      item->setText("");
    }
}

void
SubscribePanel::on_startPkg0_clicked()
{
  int  pkgIndex        = 0;
  int  freq            = 0;
  bool enableTimestamp = false;
  if (ui->freqPkg0->currentIndex() != 0)
  {
    freq = freqEnum[ui->freqPkg0->currentIndex() - 1];
  }
  else
  {
    ui->startPkg0->setText("Select Freq!");
    return;
  }
  // First, find out how many topics are to be in the package.
  int numChecked = 0;
  for (int i = 2; i < Telemetry::TOTAL_TOPIC_NUMBER; i++)
  {
    if (ui->tableWidget->item(i, pkgIndex + 2)->checkState() == Qt::Checked)
    {
      numChecked++;
      pkg0Indices.push_back(i);
    }
  }
  if (numChecked == 0)
  {
    ui->startPkg0->setText("Select Topics!");
    return;
  }
  // Then, create a list. We need to do a second pass for this :-(
  Telemetry::TopicName topicList[Telemetry::TOTAL_TOPIC_NUMBER];
  for (int i = 0; i < numChecked; i++)
  {
    topicList[i] = static_cast<Telemetry::TopicName>(pkg0Indices[i] - 2);
  }
  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numChecked, topicList, enableTimestamp, freq);

  if (pkgStatus)
    vehicle->subscribe->startPackage(pkgIndex);
  QThread::msleep(100);
  vehicle->subscribe->registerUserPackageUnpackCallback(0, pkg0UnpackCallback,
                                                        this);
}

void
SubscribePanel::on_stopPkg0_clicked()
{
  vehicle->subscribe->removePackage(0);
}

void
SubscribePanel::on_startPkg1_clicked()
{
  int  pkgIndex        = 1;
  int  freq            = 0;
  bool enableTimestamp = false;
  if (ui->freqPkg1->currentIndex() != 0)
  {
    freq = freqEnum[ui->freqPkg1->currentIndex() - 1];
  }
  else
  {
    ui->startPkg1->setText("Select Freq!");
    return;
  }
  // First, find out how many topics are to be in the package.
  int numChecked = 0;
  for (int i = 2; i < Telemetry::TOTAL_TOPIC_NUMBER; i++)
  {
    if (ui->tableWidget->item(i, pkgIndex + 2)->checkState() == Qt::Checked)
    {
      numChecked++;
      pkg1Indices.push_back(i);
    }
  }
  if (numChecked == 0)
  {
    ui->startPkg1->setText("Select Topics!");
    return;
  }
  // Then, create a list. We need to do a second pass for this :-(
  Telemetry::TopicName topicList[Telemetry::TOTAL_TOPIC_NUMBER];
  for (int i = 0; i < numChecked; i++)
  {
    topicList[i] = static_cast<Telemetry::TopicName>(pkg1Indices[i] - 2);
  }
  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numChecked, topicList, enableTimestamp, freq);

  if (pkgStatus)
    vehicle->subscribe->startPackage(pkgIndex);
  QThread::msleep(100);
  vehicle->subscribe->registerUserPackageUnpackCallback(
    pkgIndex, pkg1UnpackCallback, this);
}

void
SubscribePanel::on_stopPkg1_clicked()
{
  vehicle->subscribe->removePackage(1);
}

void
SubscribePanel::on_startPkg2_clicked()
{
  int  pkgIndex        = 2;
  int  freq            = 0;
  bool enableTimestamp = false;
  if (ui->freqPkg2->currentIndex() != 0)
  {
    freq = freqEnum[ui->freqPkg2->currentIndex() - 1];
  }
  else
  {
    ui->startPkg2->setText("Select Freq!");
    return;
  }
  // First, find out how many topics are to be in the package.
  int numChecked = 0;
  for (int i = 2; i < Telemetry::TOTAL_TOPIC_NUMBER; i++)
  {
    if (ui->tableWidget->item(i, pkgIndex + 2)->checkState() == Qt::Checked)
    {
      numChecked++;
      pkg2Indices.push_back(i);
    }
  }
  if (numChecked == 0)
  {
    ui->startPkg2->setText("Select Topics!");
    return;
  }
  // Then, create a list. We need to do a second pass for this :-(
  Telemetry::TopicName topicList[Telemetry::TOTAL_TOPIC_NUMBER];
  for (int i = 0; i < numChecked; i++)
  {
    topicList[i] = static_cast<Telemetry::TopicName>(pkg2Indices[i] - 2);
  }
  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numChecked, topicList, enableTimestamp, freq);

  if (pkgStatus)
    vehicle->subscribe->startPackage(pkgIndex);
  QThread::msleep(100);
  vehicle->subscribe->registerUserPackageUnpackCallback(
    pkgIndex, pkg2UnpackCallback, this);
}

void
SubscribePanel::on_stopPkg2_clicked()
{
  vehicle->subscribe->removePackage(2);
}

void
SubscribePanel::on_startPkg3_clicked()
{
  int  pkgIndex        = 3;
  int  freq            = 0;
  bool enableTimestamp = false;
  if (ui->freqPkg3->currentIndex() != 0)
  {
    freq = freqEnum[ui->freqPkg3->currentIndex() - 1];
  }
  else
  {
    ui->startPkg3->setText("Select Freq!");
    return;
  }
  // First, find out how many topics are to be in the package.
  int numChecked = 0;
  for (int i = 2; i < Telemetry::TOTAL_TOPIC_NUMBER; i++)
  {
    if (ui->tableWidget->item(i, pkgIndex + 2)->checkState() == Qt::Checked)
    {
      numChecked++;
      pkg3Indices.push_back(i);
    }
  }
  if (numChecked == 0)
  {
    ui->startPkg3->setText("Select Topics!");
    return;
  }
  // Then, create a list. We need to do a second pass for this :-(
  Telemetry::TopicName topicList[Telemetry::TOTAL_TOPIC_NUMBER];
  for (int i = 0; i < numChecked; i++)
  {
    topicList[i] = static_cast<Telemetry::TopicName>(pkg3Indices[i] - 2);
  }
  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numChecked, topicList, enableTimestamp, freq);

  if (pkgStatus)
    vehicle->subscribe->startPackage(pkgIndex);
  QThread::msleep(100);
  vehicle->subscribe->registerUserPackageUnpackCallback(
    pkgIndex, pkg3UnpackCallback, this);
}

void
SubscribePanel::on_stopPkg3_clicked()
{
  vehicle->subscribe->removePackage(3);
}

void
SubscribePanel::on_startPkg4_clicked()
{
  int  pkgIndex        = 4;
  int  freq            = 0;
  bool enableTimestamp = false;
  if (ui->freqPkg4->currentIndex() != 0)
  {
    freq = freqEnum[ui->freqPkg4->currentIndex() - 1];
  }
  else
  {
    ui->startPkg4->setText("Select Freq!");
    return;
  }
  // First, find out how many topics are to be in the package.
  int numChecked = 0;
  for (int i = 2; i < Telemetry::TOTAL_TOPIC_NUMBER; i++)
  {
    if (ui->tableWidget->item(i, pkgIndex + 2)->checkState() == Qt::Checked)
    {
      numChecked++;
      pkg4Indices.push_back(i);
    }
  }
  if (numChecked == 0)
  {
    ui->startPkg4->setText("Select Topics!");
    return;
  }
  // Then, create a list. We need to do a second pass for this :-(
  Telemetry::TopicName topicList[Telemetry::TOTAL_TOPIC_NUMBER];
  for (int i = 0; i < numChecked; i++)
  {
    topicList[i] = static_cast<Telemetry::TopicName>(pkg4Indices[i] - 2);
  }
  bool pkgStatus = vehicle->subscribe->initPackageFromTopicList(
    pkgIndex, numChecked, topicList, enableTimestamp, freq);

  if (pkgStatus)
    vehicle->subscribe->startPackage(pkgIndex);
  QThread::msleep(100);
  vehicle->subscribe->registerUserPackageUnpackCallback(
    pkgIndex, pkg4UnpackCallback, this);
}

void
SubscribePanel::on_stopPkg4_clicked()
{
  vehicle->subscribe->removePackage(4);
}

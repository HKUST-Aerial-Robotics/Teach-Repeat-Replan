#ifndef SUBSCRIBEPANNEL_H
#define SUBSCRIBEPANNEL_H

#include <QTableWidgetItem>
#include <QWidget>
#include <dji_vehicle.hpp>

namespace Ui
{
class SubscribePanel;
}

class SubscribePanel : public QWidget
{
  Q_OBJECT

public:
  int freqEnum[7] = { 0, 1, 10, 50, 100, 200, 400 };

public:
  explicit SubscribePanel(QWidget*            parent     = 0,
                          DJI::OSDK::Vehicle* vehiclePtr = 0);
  ~SubscribePanel();

  void display(DJI::OSDK::Telemetry::TopicName topicName, uint32_t id);

private slots:
  void on_btn_match_clicked();
  //  void on_btn_reset_clicked();
  //  void on_btn_subscribe_clicked();
  //  void on_btn_remove_clicked();
  //  void on_btn_pause_clicked();
  //  void on_btn_resume_clicked();
  void on_tableWidget_itemChanged(QTableWidgetItem* item);

  void on_startPkg0_clicked();

  void on_stopPkg0_clicked();

  void on_startPkg1_clicked();

  void on_stopPkg1_clicked();

  void on_startPkg2_clicked();

  void on_stopPkg2_clicked();

  void on_startPkg3_clicked();

  void on_stopPkg3_clicked();

  void on_stopPkg4_clicked();

  void on_startPkg4_clicked();

public:
  static void pkg0UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);
  static void pkg1UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);
  static void pkg2UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);
  static void pkg3UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);
  static void pkg4UnpackCallback(DJI::OSDK::Vehicle*      vehiclePtr,
                                 DJI::OSDK::RecvContainer rcvContainer,
                                 DJI::OSDK::UserData      userData);

private:
  Ui::SubscribePanel* ui;
  QVector<int>        pkg0Indices;
  QVector<int>        pkg1Indices;
  QVector<int>        pkg2Indices;
  QVector<int>        pkg3Indices;
  QVector<int>        pkg4Indices;

  DJI::OSDK::Vehicle* vehicle;
};

#endif // SUBSCRIBEPANNEL_H

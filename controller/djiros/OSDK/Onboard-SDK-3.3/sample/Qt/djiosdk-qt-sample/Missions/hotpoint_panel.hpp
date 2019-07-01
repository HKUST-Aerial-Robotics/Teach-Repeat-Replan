#ifndef HOTPOINT_PANEL_H
#define HOTPOINT_PANEL_H

#include <QFrame>
#include <dji_vehicle.hpp>

#define DEG2RAD 0.01745329252

namespace Ui
{
class HotpointPanel;
}

class HotpointPanel : public QFrame
{
  Q_OBJECT

public:
  explicit HotpointPanel(QWidget* parent = 0, DJI::OSDK::Vehicle* vehicle = 0);
  ~HotpointPanel();

public:
  static void hotpointReadCallback(DJI::OSDK::Vehicle*      This,
                                   DJI::OSDK::RecvContainer recvFrame,
                                   DJI::OSDK::UserData      userData);

private slots:
  void on_btn_hotPoint_start_clicked();
  void on_btn_hotPoint_stop_clicked();
  void on_btn_hotPoint_current_clicked();
  void on_btn_hp_pause_clicked(bool checked);
  void on_btn_hp_setPal_clicked();
  void on_btn_hp_setRadius_clicked();
  void on_btn_hp_setYaw_clicked();
  void on_btn_hp_data_clicked();

private:
  DJI::OSDK::Vehicle* vehicle;
  Ui::HotpointPanel*  ui;
};

#endif // HOTPOINT_PANEL_H

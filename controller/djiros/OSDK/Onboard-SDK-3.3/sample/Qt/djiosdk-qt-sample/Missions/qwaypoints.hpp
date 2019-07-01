#ifndef WAYPOINTS_H
#define WAYPOINTS_H

#include <QStandardItemModel>

#include "dji_vehicle.hpp"
#include "sdk_widgets.hpp"
#include <QWidget>

namespace Ui
{
class MissionWidget;
}

class QWaypoints : public QWidget
{
  Q_OBJECT

public:
  explicit QWaypoints(QWidget* parent = 0, DJI::OSDK::Vehicle* vehicle = 0);
  ~QWaypoints();

  void initWayPoint();
  void wpAddPoint();
  void wpRemovePoint();

  Ui::MissionWidget* getMissionUi()
  {
    return this->ui;
  }
private slots:
  void on_btn_waypoint_init_clicked();

  void on_cb_waypoint_point_currentIndexChanged(int index);
  void on_le_waypoint_number_editingFinished();
  void on_btn_waypoint_action_clicked();
  void on_btn_waypoint_reset_clicked();
  void on_btn_waypoint_removeAction_clicked();
  void on_btn_waypoint_viewPoint_clicked();
  void on_btn_wp_ivset_clicked();
  void on_btn_wp_ivRead_clicked();
  void on_btn_waypoint_add_clicked();
  void on_btn_waypoint_remove_clicked();

  void on_btn_wp_pr_clicked(bool checked);
  void on_le_wp_exec_editingFinished();
  void on_btn_wp_loadAll_clicked();
  void on_btn_wp_start_stop_clicked(bool checked);
  void on_btn_wp_loadOne_clicked();
  void wpDataChanged(const QModelIndex& topLeft, const QModelIndex& bottomRight,
                     const QVector<int>& roles);

  //  void on_btn_webTool_clicked(bool checked);

  void on_btn_AbortWaypoint_clicked();

private:
  QStandardItemModel* initAction();

  Ui::MissionWidget* ui;

  DJI::OSDK::WayPointSettings      wayPointDataTmp;
  DJI::OSDK::WayPointInitSettings* wpInitSettings;

  DJI::OSDK::Vehicle*         vehicle;
  QStandardItemModel*         waypointData;
  QStandardItemModel*         currentAction;
  QStandardItemModel*         nullAction;
  QStandardItemModel*         persistentActionPtr;
  QList<QStandardItemModel*>* actionData;
};

#endif // WAYPOINTS_H

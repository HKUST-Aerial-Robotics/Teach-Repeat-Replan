#ifndef MFIOPANNEL_H
#define MFIOPANNEL_H

#include "dji_vehicle.hpp"
#include <QWidget>

namespace Ui
{
class MFIOPanel;
}

class MFIOPanel : public QWidget
{
  Q_OBJECT

public:
  explicit MFIOPanel(QWidget* parent = 0, DJI::OSDK::Vehicle* vehicle = 0);
  ~MFIOPanel();

  static void getValueCallback(DJI::OSDK::Vehicle*      vehicle,
                               DJI::OSDK::RecvContainer recvFrame,
                               DJI::OSDK::UserData      data);

private slots:
  void on_btn_init_clicked();

  void on_cb_mode_currentIndexChanged(const QString& arg1);

  void on_btn_get_clicked();

  void on_btn_set_clicked();

private:
  Ui::MFIOPanel*      ui;
  DJI::OSDK::Vehicle* vehicle;
  QVector<int>        outputMap;
  QVector<int>        inputMap;
};

#endif // MFIOPANNEL_H

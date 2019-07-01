#include "flight_control_panel.hpp"
#include "qtosdk.hpp"
#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QMetaType>
#include <stdio.h>

// Declarations
void initVehicleConnections(qtOsdk* sdk);
QByteArray readTextFile(const QString& file_path);

int
main(int argc, char* argv[])
{

  QString      style_sheet = readTextFile(":/stylesheets/material-blue.qss");
  QApplication a(argc, argv);
  a.setStyleSheet(style_sheet);

  qRegisterMetaType<QVector<int> >("QVector<int>");
  qtOsdk w;
  w.setWindowTitle("DJI Onboard SDK");
  w.setWindowIcon(QIcon(":/images/dji_logo_gray.png"));
  initVehicleConnections(&w);
  w.show();
  return a.exec();
}

void
initVehicleConnections(qtOsdk* sdk)
{
  QObject::connect(sdk, SIGNAL(changeControlAuthorityStatus(QString)), sdk,
                   SLOT(ctrlStatusChanged(QString)));
  QObject::connect(sdk, SIGNAL(changeInitButton(QString, bool)), sdk,
                   SLOT(initFinished(QString, bool)));
  QObject::connect(sdk, SIGNAL(changeActivateButton(QString, bool)), sdk,
                   SLOT(activateFinished(QString, bool)));
}

QByteArray
readTextFile(const QString& file_path)
{
  QFile      input_file(file_path);
  QByteArray input_data;

  if (input_file.open(QIODevice::Text | QIODevice::Unbuffered |
                      QIODevice::ReadOnly))
  {
    input_data = input_file.readAll();
    input_file.close();
    return input_data;
  }
  else
  {
    return QByteArray();
  }
}

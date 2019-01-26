#include <QApplication>
#include "ros/ros.h"
#include "mainwindow.h"

int main(int argc, char ** argv)
{
  QApplication app(argc, argv);
  ros::Time::init();
  ros::init(argc, argv, "deecamp_ui");

  MainWindow win;
  win.show();

  return app.exec();
}
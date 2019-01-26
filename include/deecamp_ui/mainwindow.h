#pragma once

#include <QMainWindow>
#include <thread>

class MainWindow : public QMainWindow
{
  Q_OBJECT
 public:
  MainWindow(QWidget* parent = nullptr);
  ~MainWindow();
  void startRos();
  void joinRos();

 private:
  std::thread ros_thread_;

};
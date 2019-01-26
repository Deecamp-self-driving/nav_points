
#include "mainwindow.h"

#include <QPushButton>
#include <QGroupBox>
#include <QBoxLayout>
#include <ros/ros.h>

MainWindow::MainWindow(QWidget* parent)
{
  auto garage_box = new QGroupBox("Choose Garage");
  // auto garage_1 = new QPushButton("Garage 1", this);
  // auto garage_2 = new QPushButton("Garage 2", this);
  // auto garage_3 = new QPushButton("Garage 3", this);
  // auto garage_4 = new QPushButton("Garage 4", this);
  // auto button_layout = new QVBoxLayout();
  // button_layout->addWidget(garage_1);
  // button_layout->addWidget(garage_2);
  // button_layout->addWidget(garage_3);
  // button_layout->addWidget(garage_4); 
  // garage_box->setLayout(button_layout);

  garage_box->setStyleSheet("background-image: url(:/home/chli/deecp_ws/src/nav_points/pics/map.jpg)");
  // QPalette palette;
  // QPixmap pixmap("/home/chli/deecp_ws/src/nav_points/pics/map.jpg");
  // palette.setBrush(backgroundRole(),QBrush(pixmap));
  // garage_box->setPalette(palette);  


  setCentralWidget(garage_box);



}

MainWindow::~MainWindow()
{
  joinRos();
}

void
MainWindow::startRos()
{
  ros_thread_ = std::thread([&]() {
    ros::Rate r(30);
    while(ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
  });
}

void
MainWindow::joinRos()
{
  ros_thread_.join();
}
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QEvent>
#include <QApplication>
#include <QtQuick/QQuickView>
#include <QtQuick/QQuickItem>
#include <QDebug>
#include <vector>
#include "ros/ros.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/controlInfo.h"
#include "minho_team_ros/teleop.h"
#include <iostream>
#include <sstream>
#include <boost/shared_ptr.hpp>
#include <stdio.h>
#include <stdlib.h>

using namespace ros;
using minho_team_ros::robotInfo; //Namespace for robot information msg - PUBLISHING
using minho_team_ros::controlInfo; //Namespace for control information msg - SUBSCRIBIN
using minho_team_ros::teleop; //Namespace for teleop information msg - SUBSCRIBING

#define ROS_MASTER_IP "http://172.16.49."
#define ROS_MASTER_PORT ":11311"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int robot_id,bool real_robot, QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_hs_lin_valueChanged(int value);

    void on_hs_ang_valueChanged(int value);

    void onUpdate();

    void updateThrusts();

    void keyPressEvent(QKeyEvent *event);

    void keyReleaseEvent(QKeyEvent *event);
    
    void robotInfoCallback(const minho_team_ros::robotInfo::ConstPtr& msg);
    
    bool event( QEvent * pEvent )
	{
		if ( pEvent->type() == QEvent::WindowDeactivate ){
		    for(int i = 0;i<7;i++) { thrust_activation_[i] = false; thrust_[i] = 0; }
		}
		return QMainWindow::event( pEvent );
	}
private:
    Ui::MainWindow *ui;
    bool teleop_activated_;
    QTimer *_update_;
    int robot_id_;
    int max_lin_, max_ang_;
    bool kick_request_, is_pass_;
    bool dribblers_state_;

    std::vector<float> thrust_;
    std::vector<bool> thrust_activation_;
    
    ros::Publisher control_pub_, teleop_pub_;
    ros::Subscriber robot_sub_;
    ros::NodeHandle *_node_;
    ros::AsyncSpinner *spinner;
};

#endif // MAINWINDOW_H

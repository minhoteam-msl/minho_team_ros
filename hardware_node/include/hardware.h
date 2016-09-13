#ifndef HARDWARE_H
#define HARDWARE_H

#include <QObject>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>

#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/controlInfo.h"
#include "minho_team_ros/teleop.h"

using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - PUBLISHING
using minho_team_ros::controlInfo; //Namespace for control information msg - SUBSCRIBING
using minho_team_ros::teleop; //Namespace for teleop msg - SUBSCRIBING

#define cfgFolderPath "catkin_ws/src/minho_team_ros/hardware_node/config/"

using namespace std;

/*class serialQueue :: public QObject
{
   explicit serialQueue(QObject *parent=0);
private:
   vector<QString> writeQueue;  
public:
   void enqueue(QString data); 
signals:
   void elementAddedToQueue();     
}*/

class hardware : public QObject
{
   Q_OBJECT
public:
   explicit hardware(QObject *parent = 0); // Constructor
private slots:
   // Serial Port
   void openSerialPort(QString name); // Opens serial port for UART communication
   void closeSerialPort(); // Closes serial port for UART communication
   void handleError(QSerialPort::SerialPortError error); // Handles serial port errors
   // Configuration and Initializations
   void configFilePaths(); // Configuration of generic file paths
   void initIMULinearization(); // Initializes IMU Linearization Table
   void initVariables(); // Initialization of variables and serial port
   public slots:
   // Serial Use 
   void writeSerial(QString data); // Writes data to the opened serial port
   void readSerialData(); // Reads data received from serial port
   void writeSerialQueue(); // Writes serial command on queue
   void addCommandToQueue(const controlInfo::ConstPtr& msg); //Adds command to write queue to the serial port
   // ROS
   void setROSPublisher(ros::Publisher *pub); // Sets ROS publisher for publishing topic
   // Hardware data filtering
   int correctImuAngle(int angle); // Linearizes imu angle given the defined linearization table
private:
   // Configuration data
   QString confFilePath;
   QString imuLinFilePath;
   // Serial Port data
   QSerialPort *serial;
   int port;
   bool serialOpen;
   vector<QString> writeQueue;
   QTimer *writingTimer;
   //Linearization of IMU
   vector<int> trueVals,imuVals;
   vector<double> m,b;
   // ROS data
   ros::Publisher *ros_publisher;
   bool is_teleop_active;
public slots:
   void controlInfoCallback(const controlInfo::ConstPtr& msg);
   void teleopCallback(const teleop::ConstPtr& msg);   
   

};

#endif // HARDWARE_H

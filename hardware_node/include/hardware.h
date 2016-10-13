#ifndef HARDWARE_H
#define HARDWARE_H

#include <QObject>
#include <QThread>
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

/// \brief serialQueue wapper class to implement easy management
/// of a QString vector
class serialQueue : public QObject
{
   Q_OBJECT
public:
   /// \brief class constructor
   explicit serialQueue(QObject *parent=0);
private:
   /// \brief queue of data (QStrings) to send over UART
   vector<QString> queue;  
public:
   /// \brief function to add data to the queue
   /// \params: data - QString to be added to queue
   void enqueue(QString data); 
   /// \brief function to clear queue
   void flush();
   /// \brief function to pop (return and delete) the front
   /// elemente of the FIFO queue
   QString pop_front();
signals:
   /// \brief signal to warn that elements have been added to 
   /// the queue
   void elementAddedToQueue();     
};

class hardware : public QObject
{
   Q_OBJECT
public:
   /// \brief class constructor
   explicit hardware(QObject *parent = 0); // Constructor
private slots:
   // Serial Port
   /// \brief function to open UART communication with UART device (hardware platform)
   /// \params [in] : name -> name of the UART device to be opened
   void openSerialPort(QString name);
   /// \brief function to close UART communication
   void closeSerialPort();
   /// \brief function to handle (print to stdout) errors ocurred in UART communication
   void handleError(QSerialPort::SerialPortError error);
   
   // Configuration and Initializations
   /// \brief function to configure absolute paths for configuration files
   void configFilePaths();
   /// \brief function to initialize data to linearize (linear error correction) 
   /// the IMU (orientation) value received by hardwareInfo message over ROS coms
   void initIMULinearization();
   /// \brief function to initialize variables and to connect signals and slots.
   /// This function also detects the UART devices, removing known devices like
   /// the Hokuyo (LIDAR) to avoid wrongful connections. If everything checks out
   /// it opens the UART coms and starts watch dog
   void initVariables();
   
   public slots:
   // Serial Use 
   /// \brief function to write data to serial UART communication
   /// \params [in] : data -> string of data to be written
   void writeSerial(QString data);
   /// \brief callback function to data received on UART communication by
   /// hardware platform. This function collects and parses all the data
   /// and publishes it over ROS in hardwareInfo.
   void readSerialData();
   /// \brief function to write existing items waiting on the queue, also making
   /// a check point for the watch dog (safety timeout)
   void writeSerialQueue();
   /// \brief function to parse a controlInfo message, which contains actuation 
   /// commands and to enqueue the data to be sent over UART
   /// \params [in] : msg -> controlInfo message to be parsed and sent
   void addCommandToQueue(const controlInfo::ConstPtr& msg);
   
   // ROS
   /// \brief function to set the ROS publisher for hardwareInfo created in main_hardware.cpp
   /// \params [in] : pub -> pointer to the publisher to be used  
   void setROSPublisher(ros::Publisher *pub);
   
   // Hardware data filtering
   /// \brief function to linearize (linear error correction) the imu value received 
   /// by hardware platform over UART, given the tables initialzed in initIMULinearization()
   /// \params [in] : angle -> original imu angle to be corrected
   /// \params [out] : corrected value given linearization table
   int correctImuAngle(int angle);
   /// \brief function to implement timeout safety to avoid uncontrolled robot if something
   /// goes wrong. Fired by a timer, checks if data has been sent at least with a frequency
   /// 10Hz (100ms period)
   void watch_dog_bark();
private:
   // Configuration data
   /// \brief config folder path (to have absolut path)
   QString confFilePath;
   /// \brief imu.lin absolute file path
   QString imuLinFilePath;
   // Serial Port data
   /// \brief serial port object to use read/write to communicate over UART
   QSerialPort *serial;
   /// \brief defines wether the UART is opened or not
   bool serialOpen;
   /// \brief serialQueue object to queue data to UART
   serialQueue queue;
   /// \brief QTimer to implement safety to avoid uncontrolled robot
   QTimer *watch_dog;
   //Linearization of IMU
   /// \brief linearizarion table for imu's value error correction
   vector<int> trueVals,imuVals;
   vector<double> m,b;
   // ROS data
   /// \brief ROS hardwareInfo publisher pointer
   ros::Publisher *ros_publisher;
   /// \brief defines wether teleop is active or not
   bool is_teleop_active;
   /// \brief defines wether the robot has the ball or not, to allow kicks
   bool has_ball;
   /// \brief acts like a "timestamp" for the watch dog
   bool barking;
public slots:
   // ROS Callbacks
   /// \brief ROS callback to receive controlInfo messages. This function only applies
   /// received data if teleop state and message's teleop match.
   /// \params [in] : msg -> controlInfo message to be used/applied
   void controlInfoCallback(const controlInfo::ConstPtr& msg);
   /// \brief ROS callback to receive controlInfo messages. This function
   /// sets the current state of teleop, on or off
   /// \params [in] : msg -> teleop message to be used/applied
   void teleopCallback(const teleop::ConstPtr& msg);     
};

#endif // HARDWARE_H

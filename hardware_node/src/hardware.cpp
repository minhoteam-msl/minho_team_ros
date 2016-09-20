#include "hardware.h"
#define DEBUG_DISPLAY 1

// *********************
hardware::hardware(QObject *parent) : QObject(parent) // Constructor
{
   configFilePaths();
   initIMULinearization();
   serial = new QSerialPort(0);
   watch_dog = new QTimer();
   initVariables();
}
// *********************

// Serial Port
// *********************
void hardware::openSerialPort(QString name)
{
   serial->setPortName(name);
   serial->setBaudRate(QSerialPort::Baud115200);
   serial->setDataBits(QSerialPort::Data8);
   serial->setParity(QSerialPort::NoParity);
   serial->setStopBits( QSerialPort::OneStop);
   //serial->setFlowControl(QSerialPort::SoftwareControl);
   serial->setFlowControl(QSerialPort::NoFlowControl);

   if (serial->open(QIODevice::ReadWrite) && serial->isOpen() && serial->isWritable() && serial->isReadable()){
      serial->flush();
      ROS_INFO("Serial Port opened: %s",name.toStdString().c_str());
      serialOpen = true;
   }
   else { 
      ROS_ERROR("Error Opening Serial Port: %s",serial->errorString().toStdString().c_str());
      serialOpen = false;
   }    

}

void hardware::closeSerialPort()
{
   if (serialOpen){
      serial->write("0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n");
      serialOpen = false;
   }
   
   serial->close();
}

void hardware::handleError(QSerialPort::SerialPortError error)
{
   if (error == QSerialPort::ResourceError) {
      if(DEBUG_DISPLAY) ROS_ERROR("Serial Error %s",serial->errorString().toStdString().c_str());
      closeSerialPort();
   }
}
// *********************

// Configuration and Initializations
// *********************
void hardware::configFilePaths()
{
   QString home = getenv("HOME");
   imuLinFilePath = home+"/"+QString(cfgFolderPath)+"imu.lin";
}


void hardware::initIMULinearization() // Initializes IMU Linearization Table
{
   QFile file(imuLinFilePath);
   if(!file.open(QIODevice::ReadOnly)) {
      ROS_ERROR("Error reading imu.lin.");
      exit(1);
   }
   QTextStream in(&file);

   //parse file
   trueVals.clear(); imuVals.clear(); m.clear(); b.clear();
   QString truevalues = in.readLine(); truevalues = truevalues.right(truevalues.size()-truevalues.indexOf('=')-1);
   QString imuvalues = in.readLine(); imuvalues = imuvalues.right(imuvalues.size()-imuvalues.indexOf('=')-1);
   QStringList tv = truevalues.split(","); QStringList iv = imuvalues.split(",");

   if(tv.size()<=0 || iv.size()<=0 || tv.size()!=iv.size()){
      ROS_ERROR("Error in imu.lin.");
      exit(2);
   }
   
   for(int i=0;i<tv.size();i++){
      trueVals.push_back(tv[i].toInt());
      imuVals.push_back(iv[i].toInt());
   }
   //compute m's and b's for linearization
   for(int i=0;i<trueVals.size()-1;i++){
      m.push_back((double)(trueVals[i+1]-trueVals[i])/(double)(imuVals[i+1]-imuVals[i]));
      b.push_back((double)trueVals[i+1]-m[i]*((double)imuVals[i+1]));
   }

   file.close();
}

void hardware::initVariables() // Initialization of variables and serial port
{	
   serialOpen = false;
   ros_publisher = NULL;    
   is_teleop_active = false;
   barking = false;
   connect(serial, static_cast<void(QSerialPort::*)(QSerialPort::SerialPortError)>(&QSerialPort::error),
   [=](QSerialPort::SerialPortError error){
      if(error!=0){
         serialOpen = false;
         ROS_ERROR("Error Handling (%d): %s",error,serial->errorString().toStdString().c_str()); 
         exit(0);
      }
   });
   connect(&queue, SIGNAL(elementAddedToQueue()),this, SLOT(writeSerialQueue()));
   connect(serial, SIGNAL(readyRead()), this, SLOT(readSerialData()));
   connect(watch_dog, SIGNAL(timeout()),this,SLOT(watch_dog_bark()));

   int availablePorts = 0;
   foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()){
      if(info.manufacturer()!="Hokuyo Data Flex for USB"){
         availablePorts++;
         openSerialPort(info.portName());
      }
   }

   if(availablePorts==0){
      ROS_ERROR("No Serial Ports available");
      exit(0);
   }
   
   watch_dog->start(200);
}

// *********************

// Serial Use
// *********************
void hardware::writeSerial(QString data)
{
   if (serialOpen) {
      if(DEBUG_DISPLAY) ROS_INFO("Written %s",data.toStdString().c_str());
      serial->write(data.toLocal8Bit()+'\n');
   }
}

void hardware::readSerialData()
{
   if(serial->canReadLine()){
      hardwareInfo msg;
      QString read = serial->readLine();
      read = read.replace("\n","");
      QStringList list_read = read.split(",");
      if(list_read.length()==9){
         msg.imu_value = correctImuAngle(list_read.at(0).toInt());// Bussola
         msg.free_wheel_activated = list_read.at(1).toInt();// FreeWheel
         msg.ball_sensor = list_read.at(2).toInt();// HaveBall
         msg.encoder_1 = list_read.at(3).toInt();// Enc1
         msg.encoder_2 = list_read.at(4).toInt();// Enc2
         msg.encoder_3 = list_read.at(5).toInt();// Enc3
         msg.battery_pc = list_read.at(6).toFloat();// BatteryPC
         msg.battery_camera = list_read.at(7).toFloat();// BatteryCAM
         msg.battery_main = list_read.at(8).toFloat();// BatteryMAIN
         // publish msg to ROS environment
         if(ros_publisher!=NULL)ros_publisher->publish(msg);
         if(DEBUG_DISPLAY) ROS_INFO("Correct data published");
      } else ROS_ERROR("Error String %s",read.toStdString().c_str());
   } 
}

void hardware::writeSerialQueue()
{
   if(serialOpen) writeSerial(queue.pop_front());
   barking = true;	
}

void hardware::addCommandToQueue(const controlInfo::ConstPtr& msg)
{
   QString cmd = "";
   cmd += QString::number((int)msg->linear_velocity)+","+QString::number((int)msg->angular_velocity)
      +","+QString::number((int)msg->movement_direction);
   queue.enqueue(cmd);
}
// *********************

// ROS
// *********************
void hardware::setROSPublisher(ros::Publisher *pub)
{
   ros_publisher = pub;
}

void hardware::controlInfoCallback(const controlInfo::ConstPtr& msg)
{
   if((is_teleop_active&&msg->is_teleop) || (!is_teleop_active&&!msg->is_teleop)){
      if(serialOpen) {
         addCommandToQueue(msg);
      }
   }
}

void hardware::teleopCallback(const teleop::ConstPtr& msg)
{
   is_teleop_active = msg->set_teleop;
}

void hardware::watch_dog_bark()
{
   if(!barking){ // watch dog timed out
      queue.flush();
      if(serialOpen) { serial->write("0,0,0\n");
         if(DEBUG_DISPLAY) ROS_WARN("Watchdog triggered safety stop");
         if(!serial->waitForBytesWritten(150)) ROS_ERROR("Error sending data.");
      }
   }
   barking = false;
}
// *********************

// Hardware data filtering
// *********************
int hardware::correctImuAngle(int angle)
{
   int counter = 0;
   while(angle>imuVals[counter] && counter<imuVals.size()) counter++;
   return (int)b[counter-1]+m[counter-1]*angle;
}
// ********************* 

serialQueue::serialQueue(QObject *parent) : QObject(parent) // Constructor
{
   queue.clear();
}

void serialQueue::enqueue(QString data)
{
   queue.push_back(data);
   emit elementAddedToQueue();
}
 
void serialQueue::flush()
{
   queue.clear();
}

QString serialQueue::pop_front()
{
   if(queue.size()>0){
      QString element = queue.at(0); //FIFO
      queue.erase(queue.begin()); 
      return element;
   } else return "";
}









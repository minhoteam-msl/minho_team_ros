#include <Omni3MD.h>
// ##### INCLUDES #####
// ####################
#include <StandardCplusplus.h>
#include <string>
#include <sstream>
#include <vector>
#include <ros.h>
#include <Wire.h>    //required by Omni3MD.cpp
#include <TimerOne.h>
#include <Servo.h> 
#include <EEPROM.h>
// ####################


// ##### ROS MSG INCLUDES #####
// ############################
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/controlInfo.h"
#include "minho_team_ros/teleop.h"
#include "minho_team_ros/imuConfig.h"
#include "minho_team_ros/omniConfig.h"
// ############################

// ##### ROS SRV INCLUDES #####
// ############################
#include "minho_team_ros/requestResetEncoders.h"
#include "minho_team_ros/requestResetIMU.h"
#include "minho_team_ros/requestIMULinTable.h"
#include "minho_team_ros/requestOmniProps.h"
#include "minho_team_ros/requestKick.h"
// ############################

using minho_team_ros::requestResetEncoders;
using minho_team_ros::requestResetIMU;
using minho_team_ros::requestOmniProps;
using minho_team_ros::requestIMULinTable;
using minho_team_ros::requestKick;
using namespace std;

// ##### MOTOR/KICK PINS #####
// ###########################
#define DRIBLLER1 5  
#define MM1 4 
#define DRIBLLER2 6                      
#define MM2 7 
#define KICKPIN 8
#define BALLPIN A3
#define SERRVOPIN 9
// ###########################

// ##### BUZZER/LED PINS #####
// ###########################
#define Buzzer 53
#define Red 23
#define Green 24
#define Blue 25
// ###########################

// ##### BATTERY READ PINS #####
// #############################
#define CamBattery A0
#define PcBattery A1
#define FreeWheelButton A2
// #############################

// ##### OMNI3MD VARIABLES #####
// #############################
#define OMNI3MD_ADDRESS 0x30        //default factory address
#define BROADCAST_ADDRESS 0x00      //i2c broadcast address
#define M1  1            //Motor1
#define M2  2            //Motor2
#define M3  3            //Motor3
// #############################

// ##### EEPROM ADDRESSES #####
// ############################
#define PMSB 0
#define PLSB 1
#define IMSB 2
#define ILSB 3
#define DMSB 4
#define DLSB 5
#define BMSB 6
#define BLSB 7
#define NMSB 8
#define NLSB 9
#define MMSB 10
#define MLSB 11
#define IMULINSTEP 12
// EEPROM bytes from addr 12 to (360/read(IMULINSTEP))*2 are reserved
// ############################

// ##### COMMON FUNCTIONS #####
// ############################
/// \brief reads Omni3MD encoders through I2C API
void readEncoders();
/// \brief set PID and RAMP parameters of Omni3MD
void setupOmni();
/// \brief sets up pin types to control motor board (dribblers), kick and others
void setupMotorsBoard();
/// \brief writes current PID, BNM parameters to EEPROM
void writePIDtoEEPROM();
/// \brief reads PID,BNM parameters from EEPROM
void readPIDfromEEPROM();
/// \brief writes IMU linearization table to EEPROM
void writeIMUtoEEPROM();
/// \brief reads IMU linearization table from EEPROM
void readIMUfromEEPROM();
/// \brief timer ISR to stop the pulse sent to kick board
void timerIsr();
/// \brief function to reset (put to 0) the values of the 3 encoders of Omni3MD
void resetEncoders();
/// \brief function to publish data into ROS network with the hardware data read
void publishData();
// ############################

// ##### ROS CALLBACKS AND SERVICES #####
// ######################################
/// \brief callback function for controlInfo topic. This message contains
/// control information for the platform, as for its movement and dribbling action
/// \param msg - controlInfo message containing information to control the robot
void controlInfoCallback(const minho_team_ros::controlInfo& msg);
/// \brief callback function for teleop topic. This message enables
/// or disables teleop in the platform
/// \param msg - teleop message containing information about teleop state
void teleopCallback(const minho_team_ros::teleop& msg);
/// \brief service callback for requestResetEncoders to reset encoders values
/// \param req - empty data
/// \param res - empty data
void resetEncodersService(const requestResetEncoders::Request &req, requestResetEncoders::Response &res);
/// \brief service callback for requestResetIMU to reset imu geo-0º reference. This function sends
/// a character to the other arduino, that is responsible for acquiring data from the IMU
/// \param req - empty data
/// \param res - empty data
void resetIMUReferenceService(const requestResetIMU::Request &req, requestResetIMU::Response &res);
/// \brief service callback for requestOmniProps service, in order to set or get the configuration
/// for the Omni3MD control parameters
/// \param req - request data, containing the new configuration and a boolean variable to flag
/// if the operation is a set or get
/// \param res . response data containing the current configuration of the PID and BNM values
void OmniPropsService(const requestOmniProps::Request &req, requestOmniProps::Response &res);
/// \brief service callback for requestIMUTable service, in order to set or get the configuration
/// for the imu linearization table
/// \param req - request data, containing the new configuration and a boolean variable to flag
/// if the operation is a set or get
/// \param res . response data containing the current configuration of the IMU linearization table
void IMUTableService(const requestIMULinTable::Request &req, requestIMULinTable::Response &res);
/// \brief function to actuate kicker, in order to kick the ball. Only kicks if
/// the robot detects that has the ball inside
/// \param req - request data received in requestKick service
/// \param res - response data, flaggin if the kick was taken or not
void kickService(const requestKick::Request &req, requestKick::Response &res);
// ######################################

// ##### IMU LINEARIZATION FUNCTIONS #####
// #######################################
/// \brief sets up vectors b and m, to compute the linearization functions on the go
/// correct IMU's "natural" error with standard and equal values
void setupIMULinearization();
/// \brief sets up vectors b and m, to compute the linearization functions on the go
/// correct IMU's "natural" error, given read values from the EEPROM
void updateIMULinearization();
/// \brief given defined b and m vectors, corrects the value given by the IMU
/// \param angle - value read from the IMU, to be corrected by linearization table
/// \return - corrected value
int correctImuAngle(int angle);
// #######################################

// ##### DATA DECLARATION AND INITIALIZATION #####
// ###############################################
// ###############################################

// ##### OMNI3MD #####
// ###################
Omni3MD omni; //declaration of object variable to control the Omni3MD
float temperature=0;   // temperature reading
byte firm_int=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_dec=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_dev=0;       // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte ctrl_rate=0;      // the control rate for your motors defined at calibration (in times per second)
int enc1_max;          // maximum count for encoder 1 at calibration, for the defined control rate
int enc2_max;          // maximum count for encoder 2 at calibration, for the defined control rate
int enc3_max;          // maximum count for encoder 3 at calibration, for the defined control rate
int P = 851;
int I = 71;
int D = 101;
int B = 700;
int N = 1300;
int M = 351;
int enc1_old = 0;
int enc2_old = 0;
int enc3_old = 0;
int baterryLimitTime = 3000;
unsigned long baterryTimeStamp = 0;
boolean batteryLow = false;
int baterryLowLimitTime = 250;
unsigned long baterryLowTimeStamp = 0;
// ###################

// ##### EVENT TIMING DATA #####
// #############################
int dataSendLimitTime = 20;
unsigned long dataSendTimeStamp = 0;
int encoderLimitTime = 20;
unsigned long encoderTimeStamp = 0;
unsigned long loopTime = 0;
// #############################

// ##### IMU DATA #####
// ####################
float bussolaValorNew = 0,bussolaValor = 0,bussolaValorOld = 0;
// ####################

// ##### KICK DATA #####
// #####################
int kickTime = 0;
int maxKick = 35;
// #####################

// ##### BATTERY MEASUREMENT DATA #####
// ####################################
float maxVoltsBatteryPC = 13,maxVoltsBatteryCam = 8;
float maxBatteryPC = 567,maxBatteryCam = 537;
boolean batteryLowPC = false,batteryLowCam = false;
int baterryLowPCLimitTime = 450;
unsigned long baterryLowPCTimeStamp = 0;
int baterryLowCamLimitTime = 550;
unsigned long baterryLowCamTimeStamp = 0;
// ####################################

// ##### SAFETY TIMEOUT DATA #####
// ###############################
int comunicationTimeOutLimitTime = 200;
unsigned long comunicationTimeOutTimeStamp = 0;
// ###############################

// ##### TELEOP DATA #####
// #######################
bool teleop_active = false;
// ##################################

// ##### IMU LINEARIZATION DATA #####
// ##################################
int step = 45;
vector<unsigned int> imu_values,real_values;
vector<double> b,m;
unsigned int *vals;
// ##################################

// ##### ROS PUBLISHERS, SUBSCRIBERS AND SERVICES & DATA #####
// ###########################################################
ros::NodeHandle  nh;
minho_team_ros::hardwareInfo hwinfo_msg;
ros::Publisher hardware_info_pub("hardwareInfo", &hwinfo_msg);
ros::Subscriber<minho_team_ros::controlInfo> control_info_sub("controlInfo" , controlInfoCallback);
ros::Subscriber<minho_team_ros::teleop> teleop_info_sub("teleop" , teleopCallback);

ros::ServiceServer<requestResetEncoders::Request, requestResetEncoders::Response> server_resetEnc("requestResetEncoders",&resetEncodersService);
ros::ServiceServer<requestResetIMU::Request, requestResetIMU::Response> server_resetIMU("requestResetIMU",&resetIMUReferenceService);
ros::ServiceServer<requestOmniProps::Request, requestOmniProps::Response> server_OmniProps("requestOmniProps",&OmniPropsService);
ros::ServiceServer<requestIMULinTable::Request, requestIMULinTable::Response> server_IMULinTable("requestIMULinTable",&IMUTableService);
ros::ServiceServer<requestKick::Request, requestKick::Response> server_Kick("requestKick",&kickService);
// ###########################################################

void setup() {
  setupOmni();
  setupMotorsBoard();
  setupIMULinearization();
  resetEncoders();
  readPIDfromEEPROM();
  readIMUfromEEPROM();
  
  pinMode(KICKPIN, OUTPUT);
  digitalWrite(KICKPIN, LOW);
  
  Timer1.attachInterrupt( timerIsr ); // Timer to kick
  Timer1.stop();

  Serial.begin(57600);
  Serial.flush();
  Serial1.begin(9600);
  Serial1.flush();
  
  // On signal
  tone(Buzzer, 3000,500);
  delay(300);
  tone(Buzzer, 4000,250);
  
  nh.initNode();
  nh.advertise(hardware_info_pub);
  nh.subscribe(control_info_sub);
  nh.subscribe(teleop_info_sub);
  nh.advertiseService(server_resetEnc);
  nh.advertiseService(server_resetIMU);
  nh.advertiseService(server_OmniProps);
  nh.advertiseService(server_IMULinTable);
  nh.advertiseService(server_Kick);
  
  while (!nh.connected()){
    nh.spinOnce();
  }
 
  // Ready Signal
  nh.logwarn("MinhoTeam's ArduinoBox connected to ROS");

  tone(Buzzer, 4000,50);
  delay(150);
  tone(Buzzer, 4000,50);
  delay(300);
  tone(Buzzer, 4000,100);
  delay(300);
  tone(Buzzer, 4000,100);
}

void loop() {
  
  // Detect free_wheel button
  if(analogRead(FreeWheelButton)>500 && !hwinfo_msg.free_wheel_activated){
    omni.stop_motors();
    analogWrite(DRIBLLER1, 0); analogWrite(DRIBLLER2, 0);
    hwinfo_msg.free_wheel_activated = true;
  } else if(analogRead(FreeWheelButton)<500 && hwinfo_msg.free_wheel_activated) hwinfo_msg.free_wheel_activated = false;
  
  // Safety timeout
  if(millis()-comunicationTimeOutTimeStamp>comunicationTimeOutLimitTime){
    omni.stop_motors();
    analogWrite(DRIBLLER1, 0); analogWrite(DRIBLLER2, 0);
    comunicationTimeOutTimeStamp = millis();
  }
  // Read IMU Value
  if(Serial1.available()>0){
    String S1 = Serial1.readStringUntil('\n');
    char buffer[10];
    S1.toCharArray(buffer, 10);
    hwinfo_msg.imu_value = correctImuAngle(atof(buffer));
  }
  // Read Batteries
  if(millis()-baterryTimeStamp>baterryLimitTime){
    hwinfo_msg.battery_camera = analogRead(CamBattery) * maxVoltsBatteryCam / maxBatteryCam;
    hwinfo_msg.battery_pc = analogRead(PcBattery) * maxVoltsBatteryPC / maxBatteryPC;
    
    hwinfo_msg.battery_main = omni.read_battery();
    hwinfo_msg.battery_main -=3;//Erro de leitura
    
    if(hwinfo_msg.battery_main<20 && hwinfo_msg.battery_main>4) batteryLow = true;
    else batteryLow = false;
    
    if(hwinfo_msg.battery_pc<10.5 && hwinfo_msg.battery_pc>5) batteryLowPC = true;
    else batteryLowPC = false;
    
    if(hwinfo_msg.battery_camera<5.5 && hwinfo_msg.battery_camera>3) batteryLowCam = true;
    else batteryLowCam = false;
    
    baterryTimeStamp = millis();
  }
  
  // Read Encoders
  if(millis()-encoderTimeStamp>encoderLimitTime){
    readEncoders();
    encoderTimeStamp = millis();
  }
  
  // Publish Data
  if(millis()-dataSendTimeStamp>dataSendLimitTime){
    int ball = analogRead(BALLPIN);
    if(ball>340&&ball<370) hwinfo_msg.ball_sensor = 1;
    else hwinfo_msg.ball_sensor = 0;
    publishData();
    dataSendTimeStamp = millis();
  }
  
  // Master Battery low level sound warning
  if(batteryLow){
    if(millis()-baterryLowTimeStamp>baterryLowLimitTime){
      tone(Buzzer, 4000,baterryLowLimitTime/2);
      baterryLowTimeStamp = millis();
    }
  }
  
  // PC Battery low level sound warning
  if(batteryLowPC){
    if(millis()-baterryLowPCTimeStamp>baterryLowPCLimitTime){
      tone(Buzzer, 4500,baterryLowPCLimitTime/2);
      baterryLowPCTimeStamp = millis();
    }
  }
  
  // Cam Battery low level sound warning
  if(batteryLowCam){
    if(millis()-baterryLowCamTimeStamp>baterryLowCamLimitTime){
      tone(Buzzer, 5000,baterryLowCamLimitTime/2);
      baterryLowCamTimeStamp = millis();
    }
  }
  
  //Update ROS queues
  nh.spinOnce();
}

/// \brief reads Omni3MD encoders through I2C API
void readEncoders()
{
  omni.read_encoders(&hwinfo_msg.encoder_1,&hwinfo_msg.encoder_2,&hwinfo_msg.encoder_3);
}

/// \brief sets up pin types to control motor board (dribblers), kick and others
void setupMotorsBoard()
{
  //Setup Hardware control Pins
  pinMode(MM1, OUTPUT);   
  pinMode(MM2, OUTPUT); 
  pinMode(DRIBLLER1,OUTPUT);
  pinMode(DRIBLLER2,OUTPUT);

  pinMode(BALLPIN,INPUT);
  pinMode(PcBattery,INPUT);
  pinMode(CamBattery,INPUT);
  pinMode(FreeWheelButton,INPUT);
  
  digitalWrite(MM1,LOW);   
  digitalWrite(MM2, LOW);  
  
  analogWrite(DRIBLLER1, 0);   //PWM Speed Control
  analogWrite(DRIBLLER2, 0);   //PWM Speed Control
}

/// \brief set PID and RAMP parameters of Omni3MD
void setupOmni()
{
    omni.i2c_connect(OMNI3MD_ADDRESS);  // set i2c connection
    delay(10);                          // pause 10 milliseconds
    omni.stop_motors();                 // stops all motors
    delay(10);
    omni.read_firmware(&firm_int,&firm_dec,&firm_dev); // read firmware version value
    delay(5);
    ctrl_rate=omni.read_control_rate();   // read the control rate value
    delay(5);
    enc1_max=omni.read_enc1_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    delay(5);
    enc2_max=omni.read_enc2_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    delay(5);
    enc3_max=omni.read_enc3_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    delay(5);
    
    omni.set_PID(P,I,D); // Adjust paramenters for PID control [word Kp, word Ki, word Kd]
    delay(15);                 // 15ms pause required for Omni3MD eeprom writing

    omni.set_ramp(B,N,M);   // set acceleration ramp and limiar take off parameter gain[word ramp_time, word slope, word Kl] 
    delay(15);                 // 10ms pause required for Omni3MD eeprom writing
    delay(1000);
}

/// \brief writes current PID, BNM parameters to EEPROM
void writePIDtoEEPROM()
{
  EEPROM.write(PMSB,P>>8);
  EEPROM.write(PLSB,P);
  EEPROM.write(IMSB,I>>8);
  EEPROM.write(ILSB,I);
  EEPROM.write(DMSB,D>>8);
  EEPROM.write(DLSB,D);
  EEPROM.write(BMSB,B>>8);
  EEPROM.write(BLSB,B);
  EEPROM.write(NMSB,N>>8);
  EEPROM.write(NLSB,N);
  EEPROM.write(MMSB,M>>8);
  EEPROM.write(MLSB,M);
}

/// \brief reads PID,BNM parameters from EEPROM
void readPIDfromEEPROM()
{
  P = (EEPROM.read(PMSB)<<8)|(EEPROM.read(PLSB)); 
  I = (EEPROM.read(IMSB)<<8)|(EEPROM.read(ILSB)); 
  D = (EEPROM.read(DMSB)<<8)|(EEPROM.read(DLSB));
  B = (EEPROM.read(BMSB)<<8)|(EEPROM.read(BLSB)); 
  N = (EEPROM.read(NMSB)<<8)|(EEPROM.read(NLSB)); 
  M = (EEPROM.read(MMSB)<<8)|(EEPROM.read(MLSB));
}

/// \brief writes IMU linearization table to EEPROM
void writeIMUtoEEPROM()
{
  EEPROM.write(IMULINSTEP,step);
  int base_address = IMULINSTEP+1;
  int k = 0;
  for(int i=0;i<imu_values.size();i++){
    EEPROM.write(base_address+k,imu_values[i]>>8);   
    EEPROM.write(base_address+k+1,imu_values[i]);
    k+=2;
  } 
}

/// \brief reads IMU linearization table from EEPROM
void readIMUfromEEPROM()
{
  step = EEPROM.read(IMULINSTEP);
  imu_values.clear(); 
  real_values.clear();
  
  int base_address = IMULINSTEP+1;
  for(int i=0;i<=360;i+=step) { real_values.push_back(i); }
  int k = 0;
  for(int j=0;j<real_values.size();j++) { imu_values.push_back((EEPROM.read(base_address+k)<<8)|(EEPROM.read(base_address+k+1))); k+=2;}
 
  b.clear(); 
  m.clear();
  for(int i=0;i<real_values.size()-1;i++){
    m.push_back((double)(real_values[i+1]-real_values[i])/(double)(imu_values[i+1]-imu_values[i]));
    b.push_back((double)real_values[i+1]-m[i]*((double)imu_values[i+1]));
  }
  
}

/// \brief timer ISR to stop the pulse sent to kick board
void timerIsr()
{
    digitalWrite(KICKPIN, LOW);
    Timer1.stop();
}

/// \brief function to reset (put to 0) the values of the 3 encoders of Omni3MD
void resetEncoders()
{
   omni.set_enc_value(M1,0); // resets to zero the encoder value [byte encoder, word encValue]
   delay(1); // waits 1ms for Omni3MD to process information
   omni.set_enc_value(M2,0);
   delay(1); // waits 1ms for Omni3MD to process information
   omni.set_enc_value(M3,0);
   delay(1); // waits 1ms for Omni3MD to process information
}

/// \brief function to publish data into ROS network with the hardware data read
void publishData()
{
   hardware_info_pub.publish(&hwinfo_msg);
   enc1_old = hwinfo_msg.encoder_1;
   enc2_old = hwinfo_msg.encoder_2;
   enc3_old = hwinfo_msg.encoder_3;
}

/// \brief callback function for controlInfo topic. This message contains
/// control information for the platform, as for its movement and dribbling action
/// \param msg - controlInfo message containing information to control the robot
void controlInfoCallback(const minho_team_ros::controlInfo& msg)
{
  if((teleop_active && msg.is_teleop)||(!teleop_active && !msg.is_teleop)){
    comunicationTimeOutTimeStamp = millis();

    int movement_dir = 360-msg.movement_direction; //Ao contrario do simulador
    if(!hwinfo_msg.free_wheel_activated) omni.mov_omni(msg.linear_velocity
                                                      ,msg.angular_velocity
                                                      ,movement_dir);
    else omni.stop_motors();
    
    //dribler1
    if(msg.dribbler_on) { digitalWrite(MM1,HIGH); digitalWrite(MM2,HIGH); analogWrite(DRIBLLER1, 255); analogWrite(DRIBLLER2, 255); }
    else { digitalWrite(MM1,LOW); digitalWrite(MM2,LOW); analogWrite(DRIBLLER1, 0); analogWrite(DRIBLLER2, 0);}
    // Velocities to apply to dribblers
    //if(direct_dribler1!="2") analogWrite(DRIBLLER1, vel_dribler1.toInt());     //PWM Speed Control  
    //if(direct_dribler2!="2") analogWrite(DRIBLLER2, vel_dribler2.toInt());     //PWM Speed Control  
  }  
}

/// \brief callback function for teleop topic. This message enables
/// or disables teleop in the platform
/// \param msg - teleop message containing information about teleop state
void teleopCallback(const minho_team_ros::teleop& msg)
{
  teleop_active = msg.set_teleop;  
}

/// \brief service callback for requestResetEncoders to reset encoders values
/// \param req - empty data
/// \param res - empty data
void resetEncodersService(const requestResetEncoders::Request &req, requestResetEncoders::Response &res)
{
  omni.set_enc_value(M1,0); // resets to zero the encoder value [byte encoder, word encValue]
  delay(1); // waits 1ms for Omni3MD to process information
  omni.set_enc_value(M2,0);
  delay(1); // waits 1ms for Omni3MD to process information
  omni.set_enc_value(M3,0);
  delay(1); // waits 1ms for Omni3MD to process information   
} 

/// \brief service callback for requestResetIMU to reset imu geo-0º reference. This function sends
/// a character to the other arduino, that is responsible for acquiring data from the IMU
/// \param req - empty data
/// \param res - empty data
void resetIMUReferenceService(const requestResetIMU::Request &req, requestResetIMU::Response &res)
{
  Serial1.write("r");
  delay(10);
  Serial1.write("r");
}

/// \brief service callback for requestOmniProps service, in order to set or get the configuration
/// for the Omni3MD control parameters
/// \param req - request data, containing the new configuration and a boolean variable to flag
/// if the operation is a set or get
/// \param res . response data containing the current configuration of the PID and BNM values
void OmniPropsService(const requestOmniProps::Request &req, requestOmniProps::Response &res)
{
  if(req.is_set){ // set
    if(req.omniConf.P>0) P = req.omniConf.P; 
    if(req.omniConf.I>0) I = req.omniConf.I; 
    if(req.omniConf.D>0) D = req.omniConf.D;
    if(req.omniConf.B>0) B = req.omniConf.B; 
    if(req.omniConf.N>0) N = req.omniConf.N; 
    if(req.omniConf.M>0) M = req.omniConf.M;
    
    omni.set_PID(P,I,D); // Adjust paramenters for PID control [word Kp, word Ki, word Kd]
    delay(15);                 // 15ms pause required for Omni3MD eeprom writing
    omni.set_ramp(B,N,M);   // set acceleration ramp and limiar take off parameter gain[word ramp_time, word slope, word Kl] 
    delay(15);                 // 10ms pause required for Omni3MD eeprom writing  
    
    writePIDtoEEPROM();
    delay(10);
  }
  // send meaningful data back in any situation
  res.omniConf.P = P; res.omniConf.I = I; res.omniConf.D = D;
  res.omniConf.B = B; res.omniConf.N = N; res.omniConf.M = M;
}

/// \brief service callback for requestIMUTable service, in order to set or get the configuration
/// for the imu linearization table
/// \param req - request data, containing the new configuration and a boolean variable to flag
/// if the operation is a set or get
/// \param res . response data containing the current configuration of the IMU linearization table
void IMUTableService(const requestIMULinTable::Request &req, requestIMULinTable::Response &res)
{
  bool changing_step = false;
  if(req.is_set && req.imuConf.step>0){ // set
    //Read from EEPROM configs and generate real_values and imu_values
    if(req.imuConf.step>0) changing_step = true;
    if((360/req.imuConf.step) == req.imuConf.imu_values_length-1){
      if(changing_step) step = req.imuConf.step;
      imu_values.clear(); real_values.clear();
      for(int i=0;i<=360;i+= step) { real_values.push_back(i); }
      for(int j=0;j<real_values.size();j++) { imu_values.push_back(req.imuConf.imu_values[j]); }
      
      updateIMULinearization();
      writeIMUtoEEPROM();
    } else {
      nh.logerror("Wrong configuration for IMU Linearization Table");
    }
  } 
  
  res.imuConf.step = step;
  free(vals);
  vals = (unsigned int*)malloc(sizeof(unsigned int)*(360/step));
  for(int i=0;i<imu_values.size();i++) vals[i] = imu_values[i];
  res.imuConf.imu_values = vals;
  res.imuConf.imu_values_length = imu_values.size();
  // send meaningful data back in any situation
}

/// \brief function to actuate kicker, in order to kick the ball. Only kicks if
/// the robot detects that has the ball inside
/// \param req - request data received in requestKick service
/// \param res - response data, flaggin if the kick was taken or not
void kickService(const requestKick::Request &req, requestKick::Response &res)
{   
  int maxTime = maxKick;
  if(req.kick_is_pass) maxTime = 10;
  int kickTime = (req.kick_strength*maxTime)/100;
  
  if(kickTime>0 && hwinfo_msg.ball_sensor==1)
  {
    if(kickTime>maxTime)kickTime = maxTime;
    if(kickTime<0)kickTime = 0;
    
    digitalWrite(KICKPIN, HIGH);
    Timer1.initialize(kickTime*1000);
    res.kicked = true;
  } else res.kicked = false;
}

/// \brief sets up vectors b and m, to compute the linearization functions on the go
/// correct IMU's "natural" error with standard and equal values
void setupIMULinearization()
{
  // Read from EEPROM
  imu_values.clear(); real_values.clear();
  for(int i=0;i<=360;i+= step) { real_values.push_back(i); imu_values.push_back(i); }
  b.clear(); m.clear();
  for(int i=0;i<real_values.size()-1;i++){
    m.push_back((double)(real_values[i+1]-real_values[i])/(double)(imu_values[i+1]-imu_values[i]));
    b.push_back((double)real_values[i+1]-m[i]*((double)imu_values[i+1]));
  }
  vals = (unsigned int*)malloc(sizeof(unsigned int)*(360/step));
}

/// \brief sets up vectors b and m, to compute the linearization functions on the go
/// correct IMU's "natural" error, given read values from the EEPROM
void updateIMULinearization()
{
  b.clear(); m.clear();
  for(int i=0;i<real_values.size()-1;i++){
    m.push_back((double)(real_values[i+1]-real_values[i])/(double)(imu_values[i+1]-imu_values[i]));
    b.push_back((double)real_values[i+1]-m[i]*((double)imu_values[i+1]));
  }    
}

/// \brief given defined b and m vectors, corrects the value given by the IMU
/// \param angle - value read from the IMU, to be corrected by linearization table
/// \return - corrected value
int correctImuAngle(int angle)
{
  int counter = 0;
  while(angle>imu_values[counter] && counter<imu_values.size()) counter++;
  return (int)b[counter-1]+m[counter-1]*angle;
}

/*
 Omnidireccional 3 motor PID control using Omni3MD (www.botnroll.com)
 List of available routines to control Omni3MD board;
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.0.4 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.73.00 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.


Purpose:
 This example lists all Omni3MD routines and has commented information about them.
 This examples lists all parameters that can be read from Omni3MD with commented information about them.
 This example is not intended to be uploaded to your arduino.
 
 Routines defined at Omni3MD.h:
       //setup routines
      void i2c_connect(byte omniAddress);
      void calibrate(byte way1,byte way2,byte way3);
      void set_i2c_timeout (byte timeout); //timeout x 100 miliseconds to receive i2C Commands
      void set_i2c_address (byte newAddress);
      void set_PID(int Kp, int Ki, int Kd);
      void set_ramp(int ramp_time, int Kl);
      void set_prescaler(byte encoder, byte value);
      void set_enc_value(byte encoder, int encValue);
      void set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr);
      
      //reading routines
      float read_temperature();
      float read_battery();
      float read_firmware(); // Until Omni-3MD firmware version 1.72
      void read_firmware(byte*,byte*,byte*); // Since Omni-3MD firmware version 1.73.00
      byte read_control_rate();
      int read_enc1();
      int read_enc2();
      int read_enc3();
      int read_enc1_max();
      int read_enc2_max();
      int read_enc3_max();
      void read_encoders(int*,int*,int*);
      void read_mov_data(int*,int*,int*,float*,float*);
      void read_all_data(int*,int*,int*,float*,float*,float*,byte*,int*,int*,int*);
      
      //movement routines
      void mov_omni(byte linear_speed,int rotational_speed,int direction);
      void mov_dif_si(double linear_speed,double rotational_speed);
      void mov_pos(byte motor,int speed,int encPosition,byte stoptorque);
      void mov_lin3m_pid(int speed1,int speed2,int speed3);
      void mov_lin1m_pid(byte motor,int speed);
      void mov_lin3m_nopid(int speed1,int speed2,int speed3);
      void mov_lin1m_nopid(byte motor,int speed);
      void stop_motors();
      void save_position();
 
 Omni3MD board features: 
 - Holonomic movement integrated control of 3 motors with PID control;
 - Diferencial movement (SI units) with PID control;
 - Linear control of 3 independent motors with or without PID control;
 - Independent positional control using encoder presets and readings;
 - Encoder readings;
 - Battery reading;
 - Temperature reading;
 
 The circuit:
 * Omni3MD board attached to Arduino analog input 4 (SDA) and 5 (SCL), GND and 5V DC.
 
 This example was created by José Cruz (www.botnroll.com)
 on 26 September 2012
 Updated on 10 May 2013
 
 This code example is in the public domain. 
 http://www.botnroll.com
*/
 
#include <Wire.h>                   //required by Omni3MD.cpp
#include <Omni3MD.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        //default factory address
#define BROADCAST_ADDRESS 0x00      //i2c broadcast address

Omni3MD omni;                       //declaration of object variable to control the Omni3MD

/*** variable declarations ***/
//Variables to read from Omni3MD
int enc1=0;            // encoder1 reading, this is the encoder incremental count for the defined prescaler (positional control)
int enc2=0;            // encoder2 reading, this is the encoder incremental count for the defined prescaler (positional control)
int enc3=0;            // encoder3 reading, this is the encoder incremental count for the defined prescaler (positional control)
float battery=0;       // battery reading
float temperature=0;   // temperature reading
byte firm_int=0;      // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_dec=0;      // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte firm_dev=0;      // the firmware version of your Omni3MD board (visit http://www.botnroll.com/omni3md/ to check if your version is updated)
byte ctrl_rate;        // the control rate for your motors defined at calibration (in times per second)
int enc1_max;          // maximum count for encoder 1 at calibration, for the defined control rate
int enc2_max;          // maximum count for encoder 2 at calibration, for the defined control rate
int enc3_max;          // maximum count for encoder 3 at calibration, for the defined control rate


void setup()
{
  //setup routines
      omni.i2c_connect(OMNI3MD_ADDRESS);  //set i2c connection
      delay(5);                 // pause 5 milliseconds
  
      omni.calibrate(1,1,1);    // send the calibration command to configure the OMmni3MD board
      delay(95);                // wait 95s for calibration to end
  
      omni.set_i2c_timeout(0);  // safety parameter -> I2C communication must occur every [byte timeout] x 100 miliseconds for motor movement                             
      delay(5);                 // 5ms pause required for Omni3MD eeprom writing
  
      omni.set_i2c_address(OMNI3MD_ADDRESS); // Change Omni3MD I2C address [byte newAddress]
      delay(5);                              // 5ms pause required for Omni3MD eeprom writing
  
      omni.set_PID(250,400,125); // Adjust paramenters for PID control [word Kp, word Ki, word Kd]
      delay(15);                 // 15ms pause required for Omni3MD eeprom writing
  
      omni.set_ramp(500,1500,1000);   // set acceleration ramp and limiar take off parameter gain[int time, int slope, int Kl] (Ramp response is more agressive for lower values of slope) kl->gain for the calibration threshold value (used when moves starting from speed=0) 
      delay(10);                 // 10ms pause required for Omni3MD eeprom writing
  
       //set_prescaler(byte encoder, byte value)  value: 0 - 1 pulse; 1 - 10 pulses; 2 - 100 pulses; 3 - 1000 pulses; 4 - 10000 pulses  (requires 10ms)
      omni.set_prescaler(1, 2);  //sets the prescaler to 100; encoder count will increment by 1 each 100 pulses [byte encoder, byte value]
      delay(10);                 // 10ms pause required for Omni3MD eeprom writing
  
      omni.set_enc_value(1,0);   // presets the encoder value [byte encoder, word encValue]
      //no delay required
      
      omni.set_differential(97.5,37.5,16,14);  //set diferencial movement SI parameters (Robot axis radius(mm),Wheel radius(mm),motor gearbox reduction,encoder cpr(1 channel))
      delay(20);                               // 20ms pause required for Omni3MD eeprom writing
    
    
    
  //reading routines
      temperature=omni.read_temperature();  // read temperature value
      battery=omni.read_battery();          // read battery value
      omni.read_firmware(&firm_int,&firm_dec,&firm_dev); // read firmware version value
      ctrl_rate=omni.read_control_rate();   // read the control rate value
      enc1=omni.read_enc1();                // read encoder1 count value for the defined prescaler (positional control)
      enc2=omni.read_enc2();                // read encoder1 count value for the defined prescaler (positional control)
      enc3=omni.read_enc3();                // read encoder1 count value for the defined prescaler (positional control)
      enc1_max=omni.read_enc1_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
      enc2_max=omni.read_enc2_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
      enc3_max=omni.read_enc3_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
  
      // read all encoders at once (in a single I2C request)
      omni.read_encoders(&enc1,&enc2,&enc3);
  
      // read all encoders, battery and temperature at once (in a single I2C request)
      omni.read_mov_data(&enc1,&enc2,&enc3,&battery,&temperature);
      
      // read all Omni3MD variables at once (in a single I2C request)
      omni.read_all_data(&enc1,&enc2,&enc3,&battery,&temperature,&firm_int,&firm_dec,&firm_dev,&ctrl_rate,&enc1_max,&enc2_max,&enc3_max);



  //movement routines
      omni.mov_omni(0,-30,0);         //[byte linear_speed,int rotational_speed,int direction]
      omni.mov_dif_si(0.52,0.0);      //[double linear_speed,double rotational_speed]
      omni.mov_pos(1,30,500,1);       // move motor 1, CW at speed 30% until encoder reaches 500 counts and then brake [byte motor,int speed,int encPosition,byte stoptorque]
      omni.mov_lin3m_pid(30,-50,0);   //[int speed1,int speed2,int speed3]
      omni.mov_lin1m_pid(1,60);       //[byte motor,int speed]
      omni.mov_lin3m_nopid(-60,0,60); //[int speed1,int speed2,int speed3]
      omni.mov_lin1m_nopid(1,100);    //[byte motor,int speed]
      omni.stop_motors();             //stop all motors at once
      omni.save_position();           //save encoders positional data to eeprom
      delay(15);                      // 15ms pause required for Omni3MD eeprom writing
}
void loop()
{
}

/*
 Omnidireccional 3 motor PID control using Omni3MD (www.botnroll.com)
 1 motor linear motion with Omni3MD board;
 - this code assumes you are using 1,2 or 3 DC motors without encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.0.4 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.73.00 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.


Purpose:
 This example demonstrates linear movimentation of motors:
 
 Send movimentation orders: 
   Function: mov_lin1m_nopid( motor, speed)
   - motor: the motor to operate (1, 2 or 3);
   - speed: the desired speed for the movement (-100 to +100). 
   Ex: omni.mov_lin1m_nopid(2,35);
   The motor runs at the desired speed, speed=0 stop the motor without holding torque.

 Stop all motors without holding torque at the same time: 
   Function: stop_motors()
   Ex: omni.stop_motors();
   All motor will stop without holding torque.

 
 Important routines in this example:
 
      //setup routines
      void i2c_connect(byte omniAddress);
      void set_i2c_timeout (byte timeout); //timeout x 100 miliseconds to receive i2C Commands
            
      //movement routines
      void mov_lin1m_nopid(byte motor,int speed);

 
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
 on 27 September 2012
 Updated on 10 May 2013

 This code example is in the public domain. 
 http://www.botnroll.com
*/
 
#include <Wire.h>                   // required by Omni3MD.cpp
#include <Omni3MD.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        // default factory address
#define M1  1            //Motor1
#define M2  2            //Motor2
#define M3  3            //Motor3

Omni3MD omni;          //declaration of object variable to control the Omni3MD

//Variables to read from Omni3MD
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
    Serial.begin(115200);               // set baud rate to 115200bps for printing values in serial monitor. Press (ctrl+shift+m) after uploading
    omni.i2c_connect(OMNI3MD_ADDRESS);  // set i2c connection
    delay(10);                          // pause 10 milliseconds

    omni.stop_motors();                 // stops all motors
    delay(10);

    omni.set_i2c_timeout(0); // safety parameter -> I2C communication must occur every [byte timeout] x 100 miliseconds for motor movement                             
    delay(5);                 // 5ms pause required for Omni3MD eeprom writing
    
    omni.read_firmware(&firm_int,&firm_dec,&firm_dev); // read firmware version value
    Serial.print("Firmware:");  
    Serial.print(firm_int);             // prints firmware value
    Serial.print(".");  
    Serial.print(firm_dec);             // prints firmware value
    Serial.print(".");  
    Serial.println(firm_dev);             // prints firmware value
    ctrl_rate=omni.read_control_rate();   // read the control rate value
    Serial.print("Control_Rate:");
    Serial.println(ctrl_rate);            // prints control rate value
    enc1_max=omni.read_enc1_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    Serial.print("Encoder1_max:");
    Serial.println(enc1_max);              // prints encoder1 maximum calibration value
    enc2_max=omni.read_enc2_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    Serial.print("Encoder2_max:");
    Serial.println(enc2_max);              // prints encoder2 maximum calibration value 
    enc3_max=omni.read_enc3_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    Serial.print("Encoder3_max:");
    Serial.println(enc3_max);              // prints encoder3 maximum calibration value
    battery=omni.read_battery();          // read battery value
    Serial.print("Battery:");
    Serial.println(battery);              // prints battery value
    temperature=omni.read_temperature();  // read temperature value
    Serial.print("Temperature:");
    Serial.println(temperature);          // prints temperature value
    delay(2000);   
}


void loop()
{
    //Variables for motion control
    int speed1=60;
    int speed2=70;
    int speed3=80;

    
    //Send movement instructions    
    omni.mov_lin1m_nopid(M1,speed1);    // move motor1 at speed1
    delay(1);                           // wait 1ms for Omni3MD to process information
    omni.mov_lin1m_nopid(M2,speed2);    // move motor2 at speed2
    delay(1);                           // wait 1ms for Omni3MD to process information
    omni.mov_lin1m_nopid(M3,speed3);    // move motor3 at speed3
    delay(1);                           // wait 1ms for Omni3MD to process information  
    delay(4000);
    
    omni.stop_motors();       // stop all motors
    delay(2000);

    omni.mov_lin1m_nopid(M1,-speed3);   // move motor1 at speed1
    delay(1);                           // wait 1ms for Omni3MD to process information
    omni.mov_lin1m_nopid(M2,-speed3);   // move motor2 at speed2
    delay(1);                           // wait 1ms for Omni3MD to process information
    omni.mov_lin1m_nopid(M3,-speed3);   // move motor3 at speed3
    delay(1);                           // wait 1ms for Omni3MD to process information
    delay(3000);
 
    omni.mov_lin1m_nopid(M1,0);       // stop motor (acceleration ramp)
    delay(1);                         // wait 1ms for Omni3MD to process information
    delay(2000);
    
    omni.mov_lin1m_nopid(M2,0);       // stop motor (acceleration ramp)
    delay(1);                         // wait 1ms for Omni3MD to process information
    delay(2000);
   
    omni.mov_lin1m_nopid(M3,0);       // stop motor (acceleration ramp)
    delay(1);                         // wait 1ms for Omni3MD to process information
    delay(2000);
 
}

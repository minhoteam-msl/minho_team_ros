/*
 Omnidireccional 3 motor PID control using Omni3MD (www.botnroll.com)
 3 motors linear control with Omni3MD board;
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.0.4 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.73.00 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.


Purpose:
 This example demonstrates omnidireccional movimentation of 3 motors with PID control:
 
 Send movimentation orders: 
   Function: mov_omni(linear_speed,rotational_speed,direction)
   - linear_speed: the desired linear speed for the movement (0 to +100).
   - rotational_speed: the desired rotational speed for the movement (-100 to +100).
   - direction: the desired direction for the movement (0 to +360).
   Ex: omni.mov_lin3m_pid(35,0,90);
   The 3 motors will run at the desired speed.

 Stop all motors without holding torque: 
   Function: stop_motors()
   All motors will stop without holding torque.
   PID Integral and Differencial errors will be cleared for all motors.
   Ex: omni.stop_motors();

 Note:
 You can use encoder readings with omnidirectional movement
 In this example Encoder values are printed in serial monitor as motors are running.
 
 
 IMPORTANT!
 Before you upload this example, you must calibrate and configure PID control parameters for you system. 
 
   Follow this steps:
 
   1-Use the calibration routine, setting the function parameters to 0 or 1, to define the desired postive rotation for each motor.
     Ex: omni.calibrate(1,1,1);
     
   3-Set the PID control parameters kp, ki and kd
     Function: set_PID( Kp, Ki, Kd )
     - kp: Proportional gain. For kp=1 input the value 1000, for kp=0.5 input the value 500. Positive values only!
     - ki: Integral gain. For ki=1 input the value 1000, for ki=0.5 input the value 500. Positive values only!
     - kd: Diferential gain. For kd=1 input the value 1000, for kl=0.5 input the value 500. Positive values only!
     Ex:omni.set_PID(700,400,200);
     
   Optionally you can adjust the acceleration ramp and the kl parameter:
     Function: set_ramp( ramp_time , kl )
     - ramp_time: defines the time your robot accelerates from zero to the maximum speed (ramp inclination). 
       Values should be inputed in milliseconds. For 1 second you should input the value 1000 as a parameter. Positive values only!
       The value 0 disables this function. 32 seconds is the maximum possible time for the ramp.
     - kl: is used in the particular situation when a motor receives an order to move and its previous order was "stopped". 
       In this situation a Power*kl is loaded for the integral error of the PID control. Positive values only!
       The value 0 disables this function.
       This value is collected during calibration and corresponds to the power needed to win the motor&gearbox inertia.
 
 
 Important routines in this example:
      //setup routines
      void i2c_connect(byte omniAddress);
      void calibrate(byte way1,byte way2,byte way3);
      void set_PID(int Kp, int Ki, int Kd);
      void set_ramp(int ramp_time, int Kl);

      
      //movement routines
      mov_omni(linear_speed,rotational_speed,direction);

 
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
 on 29 March 2013
 Updated on 10 May 2013

 This code example is in the public domain. 
 http://www.botnroll.com
*/
 
#include <Wire.h>                   // required by Omni3MD.cpp
#include <Omni3MD.h>

//constants definitions
#define OMNI3MD_ADDRESS 0x30        // default factory address

Omni3MD omni;          //declaration of object variable to control the Omni3MD
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
    Serial.begin(115200);               // set baud rate to 115200bps for printing values in serial monitor. Press (ctrl+shift+m) after uploading
    omni.i2c_connect(OMNI3MD_ADDRESS);  // set i2c connection
    delay(10);                          // pause 10 milliseconds

    omni.stop_motors();                 // stops all motors
    delay(10);

    omni.set_i2c_timeout(0); // safety parameter -> I2C communication must occur every [byte timeout] x 100 miliseconds for motor movement                             
    delay(5);                 // 5ms pause required for Omni3MD eeprom writing

    omni.set_PID(250,400,125); // Adjust paramenters for PID control [word Kp, word Ki, word Kd]
    delay(15);                 // 15ms pause required for Omni3MD eeprom writing

    omni.set_ramp(500,1500,1000);   // set acceleration ramp and limiar take off parameter gain[word ramp_time, word slope, word Kl] 
    delay(15);                 // 10ms pause required for Omni3MD eeprom writing
    
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
    omni.stop_motors();
}

//Print values in serial monitor
void print_enc_values()
{
     Serial.print("enc1:");
     Serial.print(enc1);           // prints encoder 1 positional value
     Serial.print(" enc2:");
     Serial.print(enc2);           // prints encoder 2 positional value
     Serial.print(" enc3:");
     Serial.println(enc3);           // prints encoder 3 positional value
}

void wait(int time)
{
    Serial.println();
    for(int i=0;i<time;i++)  //10->1s
    {
        omni.read_encoders(&enc1,&enc2,&enc3);
        print_enc_values();  //Print values in serial monitor
        delay(200);
    }
}


void loop()
{
    //Variables for motion control
    int lin_speed=50;
    int rot_speed=0;
    int dir=180;
    omni.mov_omni(lin_speed,rot_speed,dir);    //move motors
    Serial.print("LinSpeed=");
    Serial.print(lin_speed);
    Serial.print("  RotSpeed=");
    Serial.print(rot_speed);
    Serial.print("  Direction=");
    Serial.println(dir);
    delay(2000);

    omni.stop_motors();      //stop motors without PID control (freely)
//    omni.mov_omni(0,0,0);    //stop motors with PID control
    delay(1000);

    lin_speed=50;
    rot_speed=0;
    dir=90;
    omni.mov_omni(lin_speed,rot_speed,dir);    //move motors
    Serial.print("LinSpeed=");
    Serial.print(lin_speed);
    Serial.print("  RotSpeed=");
    Serial.print(rot_speed);
    Serial.print("  Direction=");
    Serial.println(dir);
    delay(2000);

    omni.stop_motors();      //stop motors without PID control (freely)
//    omni.mov_omni(0,0,0);    //stop motors with PID control
    delay(1000);

    lin_speed=50;
    rot_speed=0;
    dir=0;
    omni.mov_omni(lin_speed,rot_speed,dir);    //move motors
    Serial.print("LinSpeed=");
    Serial.print(lin_speed);
    Serial.print("  RotSpeed=");
    Serial.print(rot_speed);
    Serial.print("  Direction=");
    Serial.println(dir);
    delay(2000);

//    omni.stop_motors();      //stop motors without PID control (freely)
    omni.mov_omni(0,0,0);    //stop motors with PID control
    delay(1000);

    lin_speed=50;
    rot_speed=0;
    dir=270;
    omni.mov_omni(lin_speed,rot_speed,dir);    //move motors
    Serial.print("LinSpeed=");
    Serial.print(lin_speed);
    Serial.print("  RotSpeed=");
    Serial.print(rot_speed);
    Serial.print("  Direction=");
    Serial.println(dir);
    delay(2000);

//    omni.stop_motors();      //stop motors without PID control (freely)
    omni.mov_omni(0,0,0);    //stop motors with PID control
    delay(1000);
    
}




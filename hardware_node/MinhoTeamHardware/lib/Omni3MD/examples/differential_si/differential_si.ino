/*
 Omnidireccional 3 motor PID control using Omni3MD (www.botnroll.com)
 List of available routines to control Omni3MD board;
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.0.4 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.73.00 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.


Purpose:
 This example demonstrates differential movimentation with SI units (m/s for linear speed and rad/s for rotational speed).
 The robot movement will look like an 8.
 
 IMPORTANT!
 Before you upload this example, you must have your robot configured for differential movement with PID control.

   Follow this steps:
   
   1-Use the calibration routine, setting the function parameters to 0 or 1, so the robot is moving forward while calibrating.
     Calibrate your robot with the wheels off the floor.
     Ex: omni.calibrate(1,0,0);
     
   2-Set the differential parameters for your robot: 
     Function: set_differential( axis_radius , whell_radius , gearbox_factor , encoder_cpr )
     - axis_radius: Robot axis radius(mm) -> Half the distance between the two wheels.
     - whell_radius: Wheel radius(mm)
     - gearbox_factor: Motor gearbox reduction -> if your motor gearbox as the reduction of 35:1 you should input value 35 for this parameter.
     - encoder_cpr: The pulses per 1 motor revolution in quadrature configuration (both channels)
        -> Check you encoder datasheet and look for the CPR(counts per rotation) or PPR(pulses per rotation). Calculate the pulses for quadrature.
     Ex: omni.set_differential(97.5,37.5,16,64);
     
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
   
   You can use encoder readings with differential movement.
 
 
 Important routines in this example:
      //setup routines
      void i2c_connect(byte omniAddress);
      void calibrate(byte way1,byte way2,byte way3);
      void set_PID(int Kp, int Ki, int Kd);
      void set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr);
      void set_ramp(int ramp_time, int Kl);
      
      //movement routines
      void mov_dif_si(double linear_speed,double rotational_speed);

 
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

Omni3MD omni;    // [byte omniAddress]

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

//Variables for motion control
double lin_speed_si=0.0;    //Linear speed in m/s
double rot_speed_si=0.0;    //Rotational speed in rad/s

void setup()
{
    //setup routines
    Serial.begin(115200);               // set baud rate to 115200bps for printing values in serial monitor. Press (ctrl+shift+m) after uploading
    omni.i2c_connect(OMNI3MD_ADDRESS);  // set i2c connection
    delay(10);                          // pause 10 milliseconds

    omni.stop_motors();                 // stops all motors
    delay(10);
/*
    omni.calibrate(1,0,0);     // send the calibration command to configure the OMmni3MD board
    delay(95);                 // wait 95s for calibration to end
*/
    omni.set_i2c_timeout(20);  // safety parameter -> I2C communication must occur every [byte timeout] x 100 miliseconds for motor movement                             
    delay(5);                  // 5ms pause required for Omni3MD eeprom writing

    omni.set_PID(700,400,200); // Adjust paramenters for PID control [word Kp, word Ki, word Kd]
    delay(15);                 // 15ms pause required for Omni3MD eeprom writing

    omni.set_ramp(500,1200,500);   // set acceleration ramp and limiar take off parameter gain[word ramp_time, word slope, word Kl] 
    delay(15);                 // 10ms pause required for Omni3MD eeprom writing

    omni.set_differential(97.5,37.5,29,60);  //set diferencial movement SI parameters (Robot axis radius(mm),Wheel radius(mm),motor gearbox reduction,encoder cpr(quadrature))
    delay(20);    
    
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
    Serial.println(enc1_max);             // prints encoder1 maximum calibration value
    enc2_max=omni.read_enc2_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    Serial.print("Encoder2_max:");
    Serial.println(enc2_max);             // prints encoder2 maximum calibration value 
    enc3_max=omni.read_enc3_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
    Serial.print("Encoder3_max:");
    Serial.println(enc3_max);             // prints encoder3 maximum calibration value
    battery=omni.read_battery();          // read battery value
    Serial.print("Battery:");
    Serial.println(battery);              // prints battery value
    temperature=omni.read_temperature();  // read temperature value
    Serial.print("Temperature:");
    Serial.println(temperature);          // prints temperature value
    delay(2000);   
}

//Print values in serial monitor
void print_enc_values()
{
     Serial.print("enc1:");
     Serial.print(enc1);           // prints encoder 1 positional value
     Serial.print(" enc2:");
     Serial.print(enc2);           // prints encoder 2 positional value
     Serial.print(" enc3:");
     Serial.println(enc3);         // prints encoder 3 positional value
}

void wait(int time)
{
    Serial.println();
    for(int i=0;i<time;i++)  //5*200->1s
    {
        omni.read_encoders(&enc1,&enc2,&enc3);
        print_enc_values();  //Print values in serial monitor
        delay(200);          //5*200->1s
    }
}

void loop()
{
  lin_speed_si=0.5;
  rot_speed_si=0.0;
  omni.mov_dif_si(lin_speed_si,rot_speed_si);   //Move forward
  wait(15); //wait 3 seconds while printing in the serial monitor
  
  rot_speed_si=1.725;
  omni.mov_dif_si(lin_speed_si,rot_speed_si);   //Add CCW rotation (Positive Rotation)
  wait(15); //wait 2.4 seconds while printing in the serial monitor

  lin_speed_si=0.5;
  rot_speed_si=0.0;
  omni.mov_dif_si(lin_speed_si,rot_speed_si);   //Move forward
  wait(15); //wait 3 seconds while printing in the serial monitor
  
  rot_speed_si=-1.725;
  omni.mov_dif_si(lin_speed_si,rot_speed_si);   //Add CW rotation (Negative rotation)
  wait(15); //wait 2.4 seconds while printing in the serial monitor
}

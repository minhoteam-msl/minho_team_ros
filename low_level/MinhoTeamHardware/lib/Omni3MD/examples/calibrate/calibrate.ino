/*
 Omnidireccional 3 motor PID control using Omni3MD (www.botnroll.com)
 List of available routines to control Omni3MD board;
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.0.4 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.73.00 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.


Purpose:
 This example explains the calibration routine and all necessary setup for the different types of movement.
 Calibrating for Omnidirectional movement and Differential movement are not the same:
 In Omnidirectional movement calibration, all motors must make the robot rotate counter clockwise (CCW).
 For Differential movement calibration, motor1 and motor3 must make the robot move forward.
 The calibration command send three parameters to the Omni3MD, one for each motor, (0 and 1) are the possible values for the parameters.
 0 makes the motor rotate in one direction (CW for example), and 1 makes the motor rotate the other way around (CCW)
 Each parameter defines the positve direction of the respective motor. 


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


void setup()
{
  //setup routines
      omni.i2c_connect(OMNI3MD_ADDRESS);  // set i2c connection
      delay(5);                           // pause 5 milliseconds
      omni.stop_motors();                 // stop all motors at once
      omni.calibrate(1,1,1);              // send the calibration command with parameters, to configure the OMmni3MD board
      delay(95);                          // wait 95s for calibration to end
  /*
  Important:
    In Omnidirectional movement calibration, all motors must make the robot rotate counter clockwise (CCW).
    For Differential movement calibration, motor1 and motor3 must make the robot move forward.
    Adjust the calibration routine parameters to acheive the necessary requirements for each type of calibration.
  */      
}

void loop()
{
  
}

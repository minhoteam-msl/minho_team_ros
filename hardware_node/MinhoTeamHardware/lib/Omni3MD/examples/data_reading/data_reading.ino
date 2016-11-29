/*
 Omnidireccional 3 motor PID control using Omni3MD (www.botnroll.com)
 List of available routines to control Omni3MD board;
 - this code assumes you are using 1,2 or 3 DC motors with encoders attached to Omni3MD board;
 - this example was developed using Arduino 1.0.4 open-source Arduino environment
 - this example is compatible with Omni3MD firmware version 1.73.00 or higher. Visit http://www.botnroll.com/omni3md/ to check if your version is updated.


Purpose:
 This example demonstrates all routines for reading values from Omni3MD.
 You can read a single value in a single I2C request.
 You can read multiple values in a single I2C request.
 Multiple values can be read in one I2C request, for example, to read all three encoders at the same time. 
 This increases precision and communication is faster than reading the same single values in three requests.
 Values are printed in the serial monitor. Press (ctrl+shift+m) after uploading to open the serial monitor.
 
 
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
byte ctrl_rate=0;        // the control rate for your motors defined at calibration (in times per second)
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
}
void print_enc_values()
{
     Serial.print("enc1:");
     Serial.print(enc1);           // prints encoder 1 positional value
     Serial.print(" enc2:");
     Serial.print(enc2);           // prints encoder 2 positional value
     Serial.print(" enc3:");
     Serial.println(enc3);           // prints encoder 3 positional value
}
void print_mov_data_values()
{
     Serial.print("enc1:");
     Serial.print(enc1);           // prints encoder 1 positional value
     Serial.print(" enc2:");
     Serial.print(enc2);           // prints encoder 2 positional value
     Serial.print(" enc3:");
     Serial.print(enc3);           // prints encoder 3 positional value
     Serial.print("  Bat:");
     Serial.print(battery);        // prints battery value
     Serial.print(" Temp:");
     Serial.println(temperature);    // prints temperature value
}
void print_all_data_values()
{
     Serial.print("enc1:");
     Serial.print(enc1);           // prints encoder 1 positional value
     Serial.print(" enc2:");
     Serial.print(enc2);           // prints encoder 2 positional value
     Serial.print(" enc3:");
     Serial.print(enc3);           // prints encoder 3 positional value
     Serial.print("  Bat:");
     Serial.print(battery);        // prints battery value
     Serial.print(" Temp:");
     Serial.print(temperature);    // prints temperature value
     Serial.print(" Firmware:");  
     Serial.print(firm_int);             // prints firmware value
     Serial.print(".");  
     Serial.print(firm_dec);             // prints firmware value
     Serial.print(".");  
     Serial.print(firm_dev);             // prints firmware value
     Serial.print(" Ctrl_rate:");
     Serial.print(ctrl_rate);      // prints control rate value
     Serial.print("  enc1_max:");
     Serial.print(enc1_max);      // prints encoder1 maximum calibration value
     Serial.print(" enc2_max:");
     Serial.print(enc2_max);      // prints encoder2 maximum calibration value 
     Serial.print(" enc3_max:");  
     Serial.println(enc3_max);    // prints encoder3 maximum calibration value
}



void loop()
{
  /**reading a single value in a single request**/
    //reading and printing static values that don't change while program runs
      omni.read_firmware(&firm_int,&firm_dec,&firm_dev); // read firmware version value
      Serial.print("Firmware:");  
      Serial.print(firm_int);             // prints firmware value
      Serial.print(".");  
      Serial.print(firm_dec);             // prints firmware value
      Serial.print(".");  
      Serial.println(firm_dev);             // prints firmware value
      
      delay(5);
      ctrl_rate=omni.read_control_rate();   // read the control rate value
      Serial.print("Control_Rate:");
      Serial.println(ctrl_rate);            // prints control rate value
      delay(5);
      enc1_max=omni.read_enc1_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
      Serial.print("Encoder1_max:");
      Serial.println(enc1_max);              // prints encoder1 maximum calibration value
      delay(5);
      enc2_max=omni.read_enc2_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
      Serial.print("Encoder2_max:");
      Serial.println(enc2_max);              // prints encoder2 maximum calibration value 
      delay(5);
      enc3_max=omni.read_enc3_max();        // read encoder1 maximum value at calibration (usefull for detecting a faulty encoder)
      Serial.print("Encoder3_max:");
      Serial.println(enc3_max);              // prints encoder3 maximum calibration value
      delay(5);
      
    //reading values that change with movement
      temperature=omni.read_temperature();  // read temperature value
      battery=omni.read_battery();          // read battery value
      enc1=omni.read_enc1();                // read encoder1 count value for the defined prescaler (positional control)
      enc2=omni.read_enc2();                // read encoder1 count value for the defined prescaler (positional control)
      enc3=omni.read_enc3();                // read encoder1 count value for the defined prescaler (positional control)
      print_mov_data_values();              // prints movement data values



  /**reading multiple values in a single request**/
    //reading encoders values all at once
      omni.read_encoders(&enc1,&enc2,&enc3);
      print_enc_values();
      delay(5);
      
    //reading movement data values all at once
      omni.read_mov_data(&enc1,&enc2,&enc3,&battery,&temperature);
      print_mov_data_values();
      delay(5);
      
    //reading all data values at once
      omni.read_all_data(&enc1,&enc2,&enc3,&battery,&temperature,&firm_int,&firm_dec,&firm_dev,&ctrl_rate,&enc1_max,&enc2_max,&enc3_max);
      print_all_data_values();
      delay(5);
      Serial.println("A pause for you to read...");// Comment this last four line of code and look at the serial monitor...
      Serial.println("Suggestion: Comment the last four lines of code and open the serial monitor again"); 
      Serial.println();
      delay(2000);  // A pause for you to read
}

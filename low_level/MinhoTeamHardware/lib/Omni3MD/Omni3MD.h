/*
  Omni3MD.h - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
  Created by Nino Pereira, April 28, 2011.
  Last Update by José Cruz, July 23, 2013.
  Released into the public domain.
*/

#ifndef Omni3MD_h
#define Omni3MD_h

#include "Arduino.h"

#define KEY1 0xAA // key used in critical commands
#define KEY2 0x55 // key used in critical commands


    /*User Commands*/
    /*Read Firmware version*/
    #define COMMAND_FIRMWARE_INT	0xFE //Read firmware value (integer value)
    #define COMMAND_FIRMWARE_DEC	0xFD //Read firmware value (decimal value)
    /*Write Commands->Don't require response from Omni3MD */
    #define COMMAND_STOP		0xFC //Stop motors
    #define COMMAND_CALIBRATE 		0xFB //Calibrate
    #define COMMAND_MOV_OMNI            0xFA //Omnidirectional movement
    #define COMMAND_MOV_LIN3M_PID       0xF8 //Linear independent motion of 3 motors with PID control
    #define COMMAND_MOV_LIN3M_NOPID     0xF7 //Linear independent motion of 3 motors without PID control
    #define COMMAND_MOV_LIN1M_PID       0xF6 //Linear independent motion of 1 motor with PID control
    #define COMMAND_MOV_LIN1M_NOPID     0xF5 //Linear independent motion of 1 motor without PID control
    #define COMMAND_GAIN_CFG		0xF4 //Configuration of kp, ki and kd parameters for PID control
    #define COMMAND_I2C_ADD		0xF2 //Change Omni3MD I2C address
    #define COMMAND_TIMEOUT_I2C         0xF1 //Configuration of the break communication protection timeout
    #define COMMAND_PRESCALER_CFG       0xF0 //Set the encoders prescaler values
    #define COMMAND_POS_ENC_PRESET      0xEF //Preset/reset the encoders count
    #define COMMAND_MOV_POS		0xEE //Positional movement
    #define COMMAND_SAVE_POS		0xED //Save the encoders counting to Omni3MD eeprom
    #define COMMAND_RAMP_CFG            0xEC //Configuration of the acceleration ramp and limiar take off parameter
    #define COMMAND_MOV_DIF_SI          0xEB //Differential movement with SI units (m/s, rad/s)
    #define COMMAND_DIF_SI_CFG          0xEA //Configuration of the differential movement with SI units
    /*Read Commands-> requests to Omni3MD */
    #define COMMAND_ENC1_INC		0xD8 //Read encoder1 positional value
    #define COMMAND_ENC2_INC            0xD7 //Read encoder2 positional value
    #define COMMAND_ENC3_INC		0xD6 //Read encoder3 positional value
    #define COMMAND_BAT                 0xD5 //Read battery voltage
    #define COMMAND_TEMP		0xD4 //Read Omni3MD temperature
    #define COMMAND_CTRL_RATE		0xD3 //Read the PID control rate
    #define COMMAND_ENC1_MAX		0xD2 //Read encoder1 maximum value at calibration
    #define COMMAND_ENC2_MAX		0xD1 //Read encoder1 maximum value at calibration
    #define COMMAND_ENC3_MAX            0xD0 //Read encoder1 maximum value at calibration


class Omni3MD
{
  public:
        //Omni3MD(byte omniAddress);

        //setup routines
        void i2c_connect(byte omniAddress);
        void set_i2c_address (byte newAddress);
        void set_i2c_timeout (byte timeout); //timeout x 100 miliseconds to receive i2C Commands
        void calibrate(boolean way1,boolean way2,boolean way3);
        void set_PID(int Kp, int Ki, int Kd);
        void set_ramp(int time, int slope, int Kl);
        void set_enc_value(byte encoder, int encValue);
        void set_prescaler(byte encoder, byte value);
        void set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr);

        //reading routines
        float read_temperature();
        float read_battery();
        float read_firmware();
        void read_firmware(byte*,byte*,byte*);
        byte read_control_rate();
        int read_enc1();
        int read_enc2();
        int read_enc3();
        int read_enc1_max();
        int read_enc2_max();
        int read_enc3_max();
        void read_encoders(int*,int*,int*);
        void read_mov_data(int*,int*,int*,float*,float*);
        void read_all_data(int*,int*,int*,float*,float*,byte*,byte*,byte*,byte*,int*,int*,int*);

        //movement routines
        void mov_omni(byte linear_speed,int rotational_speed,int direction);
        void mov_dif_si(double linear_speed,double rotational_speed);
        void mov_pos(byte motor,int speed,int encPosition,boolean stoptorque);
        void mov_lin3m_pid(int speed1,int speed2,int speed3);
        void mov_lin1m_pid(byte motor,int speed);
        void mov_lin3m_nopid(int speed1,int speed2,int speed3);
        void mov_lin1m_nopid(byte motor,int speed);
        void stop_motors();
        void save_position();
    
  private:
        byte _omniAddress;
        byte i2cRequestByte(byte addressValue, byte command);
        int  i2cRequestWord(byte addressValue, byte command);
        void i2cRequestData(byte adreessValue, byte command);
        void i2cSendData(byte addressValue, byte command, byte buffer[], byte numBytes);
};
#endif



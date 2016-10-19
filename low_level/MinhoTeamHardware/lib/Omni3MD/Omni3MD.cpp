/*
  Omni3MD.cpp - Library for interfacing with omni-3md (3 motor driver controller) from www.botnroll.com
  Created by Nino Pereira, April 28, 2011.
  Updated by José Cruz, July 23, 2013.
  Released into the public domain.
*/


#include "Wire.h"
#include "Omni3MD.h"

/*Omni3MD::Omni3MD(byte omniAddress)
{
  _omniAddress = omniAddress;
}*/

///////////////////////////////////////////////////////////////////////
//private routines
///////////////////////////////////////////////////////////////////////
void Omni3MD::i2c_connect(byte omniAddress)
{
    _omniAddress = omniAddress>>1;
	Wire.begin(_omniAddress);						// join i2c bus (address optional for master)
}
byte Omni3MD::i2cRequestByte(byte addressValue, byte command)
{
  byte value=(byte)0xFF;
  Wire.beginTransmission(addressValue);		// transmit to device
  Wire.write(command);				// sends one byte
  Wire.endTransmission();                       // stop transmitting

  Wire.requestFrom((int)addressValue,(int)1);   // requests one byte
  if (Wire.available())
  {
    value=Wire.read();
  }
  return value;
}
int Omni3MD::i2cRequestWord(byte addressValue, byte command)
{
  byte value[2]={0,0};
  int i=0;
  Wire.beginTransmission(addressValue);		// transmit to device
  Wire.write(command);				// sends one byte
  Wire.endTransmission();                       // stop transmitting

  Wire.requestFrom((int)addressValue,(int)2);   // requests two bytes
  while (Wire.available())
  {
    value[i]=Wire.read();
    i++;
  }
  i=0;
  i=value[0];
  i=i<<8;
  i=i+value[1];
  return i;
}

void Omni3MD::i2cSendData(byte addressValue, byte command, byte buffer[], byte numBytes)
{
  byte value=0;
  Wire.beginTransmission(addressValue); // transmit to device #0x10
  Wire.write(command);        // sends one byte
  for (int k =0; k< numBytes;k++)
    Wire.write(buffer[k]);        // sends one byte
  Wire.endTransmission(); // stop transmitting 
}

///////////////////////////////////////////////////////////////////////
//setup routines
///////////////////////////////////////////////////////////////////////
void Omni3MD::calibrate(boolean way1,boolean way2,boolean way3)
{
  byte buffer[]={(byte)way1, (byte)way2,(byte)way3, KEY1, KEY2};
  i2cSendData(_omniAddress, COMMAND_CALIBRATE,buffer,sizeof(buffer));
}
void Omni3MD::set_i2c_timeout (byte timeout) //timeout x 100 miliseconds to receive i2C Commands
{
  byte buffer[]={timeout, KEY1, timeout, KEY2}; 
  i2cSendData(_omniAddress, COMMAND_TIMEOUT_I2C,buffer,sizeof(buffer));
}
void Omni3MD::set_i2c_address (byte newAddress)
{
  byte buffer[]={newAddress, KEY1, newAddress, KEY2};
  i2cSendData(_omniAddress, COMMAND_I2C_ADD,buffer,sizeof(buffer));
  _omniAddress=newAddress;
}
void Omni3MD::set_PID(int Kp, int Ki, int Kd)
{
  byte Kp_H=highByte(Kp);
  byte Kp_L=lowByte(Kp);
  byte Ki_H=highByte(Ki);
  byte Ki_L=lowByte(Ki);
  byte Kd_H=highByte(Kd);
  byte Kd_L=lowByte(Kd);
  byte buffer[]={Kp_H,Kp_L,Ki_H,Ki_L,Kd_H,Kd_L};
  i2cSendData(_omniAddress, COMMAND_GAIN_CFG, buffer, sizeof(buffer));
}
void Omni3MD::set_ramp(int time, int slope, int Kl)
{
  byte time_H=highByte(time);
  byte time_L=lowByte(time);
  byte slope_H=highByte(slope);
  byte slope_L=lowByte(slope);
  byte Kl_H=highByte(Kl);
  byte Kl_L=lowByte(Kl);
  byte buffer[]={time_H,time_L,slope_H,slope_L,Kl_H,Kl_L};
  i2cSendData(_omniAddress, COMMAND_RAMP_CFG, buffer, sizeof(buffer));
}
void Omni3MD::set_prescaler(byte encoder, byte value)
{
  byte buffer[]={encoder,value,KEY1,KEY2};
  i2cSendData(_omniAddress, COMMAND_PRESCALER_CFG,buffer,sizeof(buffer));
}
void Omni3MD::set_enc_value(byte encoder, int encValue)
{
  byte encValueHIGH=highByte(encValue);
  byte encValueLOW=lowByte(encValue);
  byte buffer[]={encoder,encValueHIGH,encValueLOW,KEY1,KEY2};
  i2cSendData(_omniAddress, COMMAND_POS_ENC_PRESET,buffer,sizeof(buffer));
}
void Omni3MD::set_differential(double axis_radius, double whell_radius, double gearbox_factor, double encoder_cpr)
{
  int axis=(int)(axis_radius*100);
  int wheel=(int)(whell_radius*100);
  int gearb=(int)(gearbox_factor*10);
  int cpr=(int)(encoder_cpr*10);
  byte axis_H=highByte(axis);
  byte axis_L=lowByte(axis);
  byte wheel_H=highByte(wheel);
  byte wheel_L=lowByte(wheel);
  byte gearb_H=highByte(gearb);
  byte gearb_L=lowByte(gearb);
  byte cpr_H=highByte(cpr);
  byte cpr_L=lowByte(cpr);
  byte buffer[]={axis_H,axis_L,wheel_H,wheel_L,gearb_H,gearb_L,cpr_H,cpr_L};
  i2cSendData(_omniAddress, COMMAND_DIF_SI_CFG, buffer, sizeof(buffer));
}
///////////////////////////////////////////////////////////////////////
//movement routines
///////////////////////////////////////////////////////////////////////
void Omni3MD::mov_omni(byte linear_speed,int rotational_speed,int direction)
{
  byte rot_H=highByte(rotational_speed);
  byte rot_L=lowByte(rotational_speed);
  byte dir_H=highByte(direction);
  byte dir_L=lowByte(direction);
  byte buffer[]={linear_speed,rot_H,rot_L,dir_H,dir_L};
  i2cSendData(_omniAddress, COMMAND_MOV_OMNI,buffer,sizeof(buffer));
}
void Omni3MD::mov_dif_si(double linear_speed,double rotational_speed)
{
  int lin_speed=(int)(linear_speed*1000);
  int rot_speed=(int)(rotational_speed*1000);
  byte vlin_H=highByte(lin_speed);
  byte vlin_L=lowByte(lin_speed);
  byte vrot_H=highByte(rot_speed);
  byte vrot_L=lowByte(rot_speed);
  byte buffer[]={vlin_H,vlin_L,vrot_H,vrot_L};
  i2cSendData(_omniAddress,COMMAND_MOV_DIF_SI,buffer,sizeof(buffer));
}
void Omni3MD::mov_pos(byte motor,int speed,int encPosition,boolean stoptorque)
{
  byte speed_H=highByte(speed);
  byte speed_L=lowByte(speed);
  byte encPosHIGH=highByte(encPosition);
  byte encPosLOW=lowByte(encPosition);
  byte buffer[]={ motor,speed_H,speed_L, encPosHIGH, encPosLOW, stoptorque};
  i2cSendData(_omniAddress, COMMAND_MOV_POS,buffer,sizeof(buffer));
}
void Omni3MD::mov_lin3m_pid(int speed1,int speed2,int speed3)
{
  byte speed1_H=highByte(speed1);
  byte speed1_L=lowByte(speed1);
  byte speed2_H=highByte(speed2);
  byte speed2_L=lowByte(speed2);
  byte speed3_H=highByte(speed3);
  byte speed3_L=lowByte(speed3);
  byte buffer[]={speed1_H,speed1_L,speed2_H,speed2_L,speed3_H,speed3_L};
  i2cSendData(_omniAddress, COMMAND_MOV_LIN3M_PID,buffer,sizeof(buffer));
}
void Omni3MD::mov_lin3m_nopid(int speed1,int speed2,int speed3)
{
  byte speed1_H=highByte(speed1);
  byte speed1_L=lowByte(speed1);
  byte speed2_H=highByte(speed2);
  byte speed2_L=lowByte(speed2);
  byte speed3_H=highByte(speed3);
  byte speed3_L=lowByte(speed3);
  byte buffer[]={speed1_H,speed1_L,speed2_H,speed2_L,speed3_H,speed3_L};
  i2cSendData(_omniAddress, COMMAND_MOV_LIN3M_NOPID,buffer,sizeof(buffer));
}
void Omni3MD::mov_lin1m_pid(byte motor, int speed)
{
  byte speed_H=highByte(speed);
  byte speed_L=lowByte(speed);
  byte buffer[]={motor,speed_H,speed_L};
  i2cSendData(_omniAddress, COMMAND_MOV_LIN1M_PID,buffer,sizeof(buffer));
}
void Omni3MD::mov_lin1m_nopid(byte motor, int speed)
{
  byte speed_H=highByte(speed);
  byte speed_L=lowByte(speed);
  byte buffer[]={motor,speed_H,speed_L};
  i2cSendData(_omniAddress, COMMAND_MOV_LIN1M_NOPID,buffer,sizeof(buffer));
}
void Omni3MD::stop_motors() 
{
  byte buffer[]={KEY1,KEY2};
  i2cSendData(_omniAddress, COMMAND_STOP,buffer,sizeof(buffer));
}	
void Omni3MD::save_position()
{
  byte buffer[]={KEY1,KEY2};
  i2cSendData(_omniAddress, COMMAND_SAVE_POS,buffer,sizeof(buffer));
}
//////////////////////////////////////////////////////////////////////////////////////
// Readings routines
//////////////////////////////////////////////////////////////////////////////////////
void Omni3MD::read_all_data(int *enc1,int *enc2,int *enc3,float *bat,float *temp,byte *firm1,byte *firm2,byte *firm3,byte *ctrl_rate,int *enc1_max,int *enc2_max,int *enc3_max)
{
  byte value[20]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  int i=0;

  Wire.beginTransmission(_omniAddress);		// transmit to device
  Wire.write(COMMAND_ENC1_INC);                 // sends one byte
  Wire.endTransmission();                       // stop transmitting

  Wire.requestFrom((int)_omniAddress,(int)20);   // requests 19 bytes
  while (Wire.available())
  {
    value[i]=Wire.read();
    i++;
  }
  i=value[0]&0x00FF;
  i=i<<8;
  i=i+value[1];
  *enc1=i;
  i=value[2]&0x00FF;
  i=i<<8;
  i=i+value[3];
  *enc2=i;
  i=value[4]&0x00FF;
  i=i<<8;
  i=i+value[5];
  *enc3=i;
  i=value[6]&0x00FF;
  i=i<<8;
  i=i+value[7];
  *bat=(float)(i/10.0);
  i=value[8]&0x00FF;
  i=i<<8;
  i=i+value[9];
  *temp=(float)(i/10.0);
  *firm1=value[10];
  *firm2=value[11];
  *firm3=value[12];
  *ctrl_rate=value[13];
  i=value[14]&0x00FF;
  i=i<<8;
  i=i+value[15];
  *enc1_max=i;
  i=value[16]&0x00FF;
  i=i<<8;
  i=i+value[17];
  *enc2_max=i;
  i=value[18]&0x00FF;
  i=i<<8;
  i=i+value[19];
  *enc3_max=i;
}
void Omni3MD::read_mov_data(int *enc1,int *enc2,int *enc3,float *bat,float *temp)
{
  byte value[10]={0,0,0,0,0,0,0,0,0,0};
  int i=0;

  Wire.beginTransmission(_omniAddress);		// transmit to device
  Wire.write(COMMAND_ENC1_INC);                 // sends one byte
  Wire.endTransmission();                       // stop transmitting

  Wire.requestFrom((int)_omniAddress,(int)10);   // requests 10 bytes
  while (Wire.available())
  {
    value[i]=Wire.read();
    i++;
  }
  i=value[0]&0x00FF;
  i=i<<8;
  i=i+value[1];
  *enc1=i;
  i=value[2]&0x00FF;
  i=i<<8;
  i=i+value[3];
  *enc2=i;
  i=value[4]&0x00FF;
  i=i<<8;
  i=i+value[5];
  *enc3=i;
  i=value[6]&0x00FF;
  i=i<<8;
  i=i+value[7];
  *bat=(float)(i/10.0);
  i=value[8]&0x00FF;
  i=i<<8;
  i=i+value[9];
  *temp=(float)(i/10.0);
}
void Omni3MD::read_encoders(int *enc1,int *enc2, int *enc3)
{
  byte value[6]={0,0,0,0,0,0};
  int i=0;

  Wire.beginTransmission(_omniAddress);		// transmit to device
  Wire.write(COMMAND_ENC1_INC);                 // sends one byte
  Wire.endTransmission();                       // stop transmitting

  Wire.requestFrom((int)_omniAddress,(int)6);   // requests 6 bytes
  while (Wire.available())
  {
    value[i]=Wire.read();
    i++;
  }
  i=value[0]&0x00FF;
  i=i<<8;
  i=i+value[1];
  *enc1=i;
  i=value[2]&0x00FF;
  i=i<<8;
  i=i+value[3];
  *enc2=i;
  i=value[4]&0x00FF;
  i=i<<8;
  i=i+value[5];
  *enc3=i;
}
float Omni3MD::read_temperature()
{
  return (float)(i2cRequestWord(_omniAddress, COMMAND_TEMP))/10.0;
}

float Omni3MD::read_battery()
{
  return (float)(i2cRequestWord(_omniAddress, COMMAND_BAT))/10.0;
}

int Omni3MD::read_enc1()
{
  return i2cRequestWord(_omniAddress, COMMAND_ENC1_INC);
}

int Omni3MD::read_enc2()
{
  return i2cRequestWord(_omniAddress, COMMAND_ENC2_INC);
}

int Omni3MD::read_enc3()
{
  return i2cRequestWord(_omniAddress, COMMAND_ENC3_INC);
}

int Omni3MD::read_enc1_max()
{
  return i2cRequestWord(_omniAddress, COMMAND_ENC1_MAX);
}

int Omni3MD::read_enc2_max()
{
  return i2cRequestWord(_omniAddress, COMMAND_ENC2_MAX);
}

int Omni3MD::read_enc3_max()
{
  return i2cRequestWord(_omniAddress, COMMAND_ENC3_MAX);
}

void Omni3MD::read_firmware(byte *firm1,byte *firm2,byte *firm3)
{
  byte value[3]={0,0,0};
  int i=0;
  Wire.beginTransmission(_omniAddress);		// transmit to device
  Wire.write(COMMAND_FIRMWARE_INT);             // sends one byte
  Wire.endTransmission();                       // stop transmitting

  Wire.requestFrom((int)_omniAddress,(int)3);   // requests 3 bytes
  while (Wire.available())
  {
    value[i]=Wire.read();
    i++;
  }
  *firm1=value[0];
  *firm2=value[1];
  *firm3=value[2];
}

float Omni3MD::read_firmware()
{
  byte firmI=i2cRequestByte(_omniAddress, COMMAND_FIRMWARE_INT);
  byte firmD=i2cRequestByte(_omniAddress, COMMAND_FIRMWARE_DEC);
  int firmwareValue=(firmI*100)+firmD;
  return (float)((firmwareValue/100.0));
}
byte Omni3MD::read_control_rate() // returns 10, 20, 40 (times per second control operation)
{
  return i2cRequestByte(_omniAddress, COMMAND_CTRL_RATE);
}
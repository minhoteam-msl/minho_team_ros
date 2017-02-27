#include "pid.h"

// Constructor of class receiving parameters
PID::PID(float p, float i, float d, float mi, float ma)
{
    kp=p;
    ki=i;
    kd=d;
    minOut=mi;
    maxOut=ma;
    lastError	= 0.0;
    integral	= 0.0;
    firstTime	= true;
}
// Constructor of class receiving a PID object
PID::PID(const PID &pid)
{
    kp = pid.kp;
    ki = pid.ki;
    kd = pid.kd;

    minOut	= pid.minOut;
    maxOut	= pid.maxOut;

    lastError	= pid.lastError;
    integral	= pid.integral;
    firstTime	= true;

}

///
/// Calculate the value that should be applied to minimize error
///
float PID::calc_pid(float value, float error)
{
    float out;

    if (firstTime)
    {
        integral=0;
        out = value + kp*error;
        firstTime=false;
    }
    else
    {
        integral += ki*error;
 	if(error*lastError<0)integral=0;
        if(integral>50)integral=50;
	if(integral<-50)integral=-50;
        out = value + (kp*error +kd*(error-lastError) + integral);
    }

    if(out>maxOut)out=maxOut;
    if(out<minOut)out=minOut;

    lastError=error;
    lastOut=out;

    return out;
}

///
/// Reset the PID calculations
///
void PID::reset()
{
    firstTime=true;
}

void PID::display()
{
    //cout<<"p(%.2f)"<<kp<<"i(%.2f)"<<ki<<"d(%.2f)"<<kd<<endl;
    //ROS_INFO("P: %.2f I: %.2f D: %.2f", kp, ki, kd);
}

///
/// Set the P,I,D values received and reset some important aspects of the controler
///
void PID::setPIDValues(float p, float i, float d)
{
    kp=p;
    ki=i;
    kd=d;
    lastError=0;
    integral=0.0;
    firstTime=true;
}

///
/// Function that returns error of controler
/// \return float
///
float PID::getError()
{
	return lastError;
}

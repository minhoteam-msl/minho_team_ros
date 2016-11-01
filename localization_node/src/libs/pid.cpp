#include "pid.h"

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
        if(integral>100)integral=100;
        out = value + (kp*error +kd*(error-lastError) + integral);
    }

    if(out>maxOut)out=maxOut;
    if(out<minOut)out=minOut;

    lastError=error;

    return out;
}

void PID::reset()
{
    kp=ki=1;
    kd=0;
    firstTime=true;
}

void PID::display()
{
    cout<<"p(%.2f)"<<kp<<"i(%.2f)"<<ki<<"d(%.2f)"<<kd<<endl;
}

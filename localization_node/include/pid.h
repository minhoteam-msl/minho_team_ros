#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <iostream>

using namespace std;


class PID
{
public:
    PID(float p=1, float i=1, float d=0, float mi=0, float ma=100);
    PID(const PID& pid);

    float calc_pid(float value,float error);

    inline void setP(float p){ kp=p; }
    inline void setI(float i){ ki=i; }
    inline void setD(float d){ kd=d; }
    inline void setMinOut( float mo ) { minOut = mo; }
    inline void setMaxOut( float mo ) { maxOut = mo; }

    inline float getP() { return kp; }
    inline float getI() { return ki; }
    inline float getD() { return kd; }
    inline float getMaxOut() { return maxOut; }
    inline float getMinOut() { return minOut; }


    void reset();
    void display();
private:
    float kp;
    float ki;
    float kd;
    float minOut;
    float maxOut;
    float lastError;
    float integral;
    bool  firstTime;

};

#endif // PID_H

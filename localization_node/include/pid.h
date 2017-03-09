#ifndef PID_H
#define PID_H

#include <stdio.h>
#include <iostream>

using namespace std;


class PID
{
public:
    // Constructors
    PID(float p=1, float i=1, float d=0, float mi=0, float ma=100);
    PID(const PID& pid);
    // Methods responsible for setting and getting gain valors
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
    //Methods responsible for executing operations
    float calc_pid(float value,float error);
    void reset();
    void display();
    void setPIDValues(float p, float i, float d);
    float getError();
    inline float getlastOut() { return lastOut;}
private:
    float kp;
    float ki;
    float kd;
    float minOut;
    float maxOut;
    float lastError;
    float integral;
    bool  firstTime;
    float lastOut;

};

#endif // PID_H

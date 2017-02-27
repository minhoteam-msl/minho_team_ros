#ifndef BLACKFLYCAM_H
#define BLACKFLYCAM_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <flycapture/FlyCapture2.h>
#include <thread>
#include <mutex>
#include <time.h>
#include "pid.h"
#include "ros/ros.h"
#include <QFile>
#include <QString>
#include <QTextStream>
#include <QStringList>
#include <QPoint>
#include "Utils/types.h"

using namespace FlyCapture2;
using namespace cv;
using namespace std;

enum property_cam{BRI=0,GAI=1,SHU=2,GAM=3,SAT=4,WB=5,EXP=6};


class BlackflyCam
{
public:
    explicit BlackflyCam(bool cal, int robot_id);
    ~BlackflyCam();

    //#### CAMERA INTERFACE FUNCTIONS ####
    //####################################
    bool connect();
    bool closeCamera();
    bool startCapture();
    bool stopCapture();
    void printCameraInfo();
    void setNewFrame(Image *pImage);
    bool frameAvailable();
    bool isCameraReady(); //Returns the state of the camera
    Mutex *getLockingMutex();
    Mat *getImage();
    float getFPS();
    inline void toggleCalibrate(){ calibrate = (!calibrate);}
    inline void toggleFirsttime(){ firsttime = (!firsttime);}
    //####################################
    //####################################


    //#### CAMERA PARAMETERS FUNCTIONS ####
    //#####################################

    //Brigtness(5.469->29.688)
    void setBrigtness(float value);
    float getBrigtness();
    bool getBrigtnessState();
    void setBrigtnessState(bool state);
    inline float getBrigtnessMax(){ return 29.688;}
    inline float getBrigtnessMin(){ return 5.469;}

    //Shutter(0.046->32.754)
    void setShutter(float value);
    float getShuttertime();
    bool getShutterState();
    void setShutterState(bool state);
    inline float getShuttertimeMax(){ return 32.754;}
    inline float getShuttertimeMin(){ return 0.046;}

    //Gain(0->23.991)
    void setGain(float value);
    float getGain();
    bool getGainState();
    void setGainState(bool state);
    inline float getGainMax(){ return 23.991;}
    inline float getGainMin(){ return 0;}

    //Auto-Exposure(-7.585->2.414)
    void setExposure(float value);
    float getExposure();
    bool getExposureState();
    void setExposureState(bool state);
    inline float getExposureMax(){ return 2.414;}
    inline float getExposureMin(){ return -7.585;}

    //Gamma(0.5->3.999)
    void setGamma(float value);
    float getGamma();
    bool getGammaState();
    void setGammaState(bool state);
    inline float getGammaMax(){ return 3.999;}
    inline float getGammaMin(){ return 0.5;}

    //Saturation(0->399.902)
    void setSaturation(float value);
    float getSaturation();
    bool getSaturationState();
    void setSaturationState(bool state);
    inline float getSaturationMax(){ return 399.902;}
    inline float getSaturationMin(){ return 0;}

    //White_Balance(0->1023) both
    void setWhite_Balance(float value,float value2);
    void setWhite_Balance_valueA(float value);
    void setWhite_Balance_valueB(float value);
    float getWhite_Balance_valueA();
    float getWhite_Balance_valueB();
    bool getWhite_BalanceState();
    void setWhite_BalanceState(bool state);
    inline float getWhite_BalanceMax(){ return 1023;}
    inline float getWhite_BalanceMin(){ return 0;}
    void setWhite_BalanceAuto(bool state);

    void setProps(int num);
    bool initProps(int robot_id);
    bool writePIDConfig();
    bool writePropConfig();
    void setPropControlPID(int id,float p, float i, float d, bool blue);
    Point2d getError(int prop_in_use);
    //#####################################
    //#####################################

    //#### CALIBRATION FUNCTIONS ####
    //###############################
    void calcLumiHistogram();
    void calcSatHistogram();
    Scalar averageRGB();
    float calcMean();
    Scalar averageUV();
    bool cameraCalibrate();
    void setRegions();
    void setWhiteROI(int x, int y, int h);
    void setBlackROI(int x, int y, int h);
    //###############################
    //###############################

    void setSatTarget(int val);
    void setLumiTarget(int val);
    int getSatTarget();
    int getLumiTarget();

private:
    //#### CAMERA INTERFACE DATA ####
    //###############################
    Camera *camera;
    CameraInfo camInfo;
    Image rawimage,imagecolor;
    FlyCapture2::Error error;
    Mat *frameBuffer,*buffer;
    Mat holder;
    bool imageready,newimage,calibrate,firsttime;
    float rowBytes;
    int frameCounter;
    float fps;
    Mutex mutex;
    struct timespec past,present;
    int rob_id;
    //###############################
    //###############################

    //#### CAMERA PARAMETERS DATA ####
    //################################
    QString pidPath, propertyPath;
    bool initPidValues(int robot_id);
    Property props[7];
    PID *ShuPID,*BrigPID,*SatPID,*GammaPID,*GainPID,*wRedPID,*wBluePID;
    //################################
    //################################

    //#### CALIBRATION DATA ####
    //##########################
    Rect roi_black, roi_white;
    std::vector<int> histvalue;
    Mat image_roi_white,image_roi_black,image;
    float msvError;
    int msv, satTarget, lumiTarget;
    float minError_Lumi,minError_Sat,minError_UV_1,minError_UV_2,minError_RGB;
    //##########################
    //##########################
};

#endif // BLACKFLYCAM_H

#ifndef BLACKFLYCAM_H
#define BLACKFLYCAM_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <flycapture/FlyCapture2.h>
#include "ros/ros.h"
#include <thread>
#include <mutex>
//#include "cameracalib.h"
//#include "properties.h"
#include <time.h>

using namespace FlyCapture2;
using namespace cv;
using namespace std;

class BlackflyCam
{
public:
    explicit BlackflyCam();
    ~BlackflyCam();

    bool connect();
    bool closeCamera();
    bool startCapture();
    bool stopCapture();
    void printCameraInfo();
    void setNewFrame(Image *pImage);
    bool frameAvailable();
    bool isCameraReady(); //Returns the sate of the camera
    Mutex *getLockingMutex();
    Mat *getImage();
    float getFPS();
    
private:
    Camera *camera;
    CameraInfo camInfo;
    Image rawimage,imagecolor;
    FlyCapture2::Error error;
    Mat *frameBuffer,*buffer;
    Mat holder;
    bool imageready,newimage,calibrate;
    float rowBytes;
    int frameCounter;
    float fps;
    Mutex mutex;
    //Properties *props;
   // CameraCalib *camcalib;
    
    struct timespec past,present;
};

#endif // BLACKFLYCAM_H

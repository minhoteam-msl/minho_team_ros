#ifndef BLACKFLYCAMERA_H
#define BLACKFLYCAMERA_H

#include "blackflydriver.h"

/* This class implements the driver of PGE BlackFly with QThreads
 * This code was made by Pedro Osório Silva @ MinhoTeam
*/

using namespace FlyCapture2;
using namespace cv;
using namespace std;

class BlackFlyCamera:public QObject
{
    Q_OBJECT

public:
    /* Image and class methods */
    BlackFlyCamera();
    bool startCamera(); //Starts camera connection
    void closeCamera(); //Closes camera connection and image capture thread
    bool startImaging(); //Starts image capture thread
    void printCameraInfo(); //Prints camera information
    Mat *getImage(bool *success); //Returns an image

    /* State return methods */
    bool isCameraReady(); //Returns the sate of the camera
    bool frameAvailable(); //Returns the existance of a new frame.
private:
    /* State Variables */
    bool cameraReady; //Flag that states the camera state
    bool newFrameAvailable; //Flags the existance of a new frame to be processed
    bool printDebug; //Flags qDebug() execution
    Mat frameBuffer, holder; //Main frame buffer image
    Mat *buffer; // Image buffer pointer
    /* Camera Variables */
    FlyCapture2::Error error; //Error variable
    Camera camera; //Camera object
    CameraInfo camInfo; //Camera Information
    BlackFlyDriver *driver; //Implements working thread for camera image acquisition
    QTimer *thread; //Thread for image acquisition
    QMutex mymutex; //Mutex to lock up image resources
};

#endif // BLACKFLYCAMERA_H

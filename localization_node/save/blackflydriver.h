#ifndef BLACKFLYDRIVER_H
#define BLACKFLYDRIVER_H

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <flycapture/FlyCapture2.h>
#include <QObject>
#include <QMutex>
#include <QTimer>
#include <QTime>
#include "ros/ros.h"

/* This class implements the driver of PGE BlackFly with QThreads
 * This code was made by Pedro Osório Silva @ MinhoTeam
*/

using namespace FlyCapture2;
using namespace cv;
using namespace std;
class BlackFlyDriver:public QObject
{
    Q_OBJECT
public:
    BlackFlyDriver(Camera *cam,bool *newFrame,QMutex *mutex,Mat *buffer);
    void readFrame(); // Reads a frame
    void doSetup(QTimer *thread,int requiredMS); // Sets up image reading thread
public slots:
    void doWork(); //Working thread that reads an image
private:
    /* Image Capture Variables */
    Camera *camera; //Camera Object
    Mat *frameBuffer; // Frame buffer pointer
    QMutex *imgMutex; // Buffer access mutex pointer
    Image rawImage,colorImage; // QImages
    QTimer *parentTimer;
    int requiredTiming;
    /* Other Variables */
    float rowBytes;
    bool *gotNewFrame;
    int frameCounter;
    FlyCapture2::Error error;
};

#endif // BLACKFLYDRIVER_H

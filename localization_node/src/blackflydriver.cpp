#include "blackflydriver.h"

// Constructor
BlackFlyDriver::BlackFlyDriver(Camera *cam, bool *newFrame, QMutex *mutex,Mat *buffer)
{
    //We should point all pointers to the right memory places
    camera = cam;
    gotNewFrame = newFrame;
    frameCounter = 0;
    imgMutex = mutex;
    frameBuffer = buffer;
    requiredTiming = 0;
}

// Reads a frame from BlackFly PGE camera
void BlackFlyDriver::readFrame()
{
    error = camera->RetrieveBuffer( &rawImage );
    if ( error == PGRERROR_OK ){ //Acquire image
        rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &colorImage );
        rowBytes = (double)colorImage.GetReceivedDataSize()/(double)colorImage.GetRows();
        (*frameBuffer) = Mat(colorImage.GetRows(), colorImage.GetCols(), CV_8UC3, colorImage.GetData(),rowBytes);
        (*gotNewFrame) = true;
    } else (*gotNewFrame) = false;
}

// Sets up Reading Thread
void BlackFlyDriver::doSetup(QTimer *thread,int requiredMS)
{
    parentTimer = thread;
    requiredTiming = requiredMS;
    connect(parentTimer,SIGNAL(timeout()),this,SLOT(doWork()));
}

// Worker thread that reads images from camera if possible
void BlackFlyDriver::doWork()
{
    int timing = 0;
    parentTimer->stop();
    QTime measure;
    measure.start();
    error = camera->RetrieveBuffer( &rawImage );
    if ( error == PGRERROR_OK ){ //Acquire image
        if(imgMutex->tryLock(1)){
            rawImage.Convert( FlyCapture2::PIXEL_FORMAT_BGR, &colorImage );
            rowBytes = (double)colorImage.GetReceivedDataSize()/(double)colorImage.GetRows();
            (*frameBuffer) = Mat(colorImage.GetRows(), colorImage.GetCols(), CV_8UC3, colorImage.GetData(),rowBytes);
            (*gotNewFrame) = true;
            imgMutex->unlock();
            timing = requiredTiming-measure.elapsed();
            if(timing<0) timing = requiredTiming;
            parentTimer->start(timing);
        }
    } else {
        (*gotNewFrame) = false;
        parentTimer->start(5);
    }
}


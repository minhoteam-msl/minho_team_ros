#include "blackflycamera.h"

// Constructor
BlackFlyCamera::BlackFlyCamera(bool enableDEBUG)
{
    newFrameAvailable = cameraReady = false; //Flags start as false
    printDebug = enableDEBUG;
    frameBuffer = Mat(480,480,CV_8UC3,Scalar(0,0,0));
}

//Starts camera connection and image capture thread, returns true if successful
bool BlackFlyCamera::startCamera()
{
    // Connect the camera
    error = camera.Connect(0);
    if ( error != PGRERROR_OK ) {
        if(printDebug) cout << "GigE Connection failed (1)."<<endl;
        cameraReady = false;
        return cameraReady;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK ) {
        if(printDebug) cout << "GigE Connection failed (2)."<<endl;
        cameraReady = false; return cameraReady;
    }

    cameraReady = true;
    if(printDebug) cout << "GigE Connection successful."<<endl;
    return cameraReady;
}

//Closes camera connection and image capture thread
void BlackFlyCamera::closeCamera()
{
    camera.StopCapture();
    camera.Disconnect();
    cameraReady = false;
    if(printDebug) cout << "GigE Disconnection successful."<<endl;
}

//Starts image capture thread, returns true if successful
bool BlackFlyCamera::startImaging()
{
    if(!cameraReady) return false;
    else {
        error = camera.StartCapture();
        if ( error != PGRERROR_OK ) {
            if(printDebug) cout << "GigE Imaging failed."<<endl;
            cameraReady = false; return cameraReady;
        }
    }

    cameraReady = true;
    if(printDebug) cout << "GigE Imaging successful."<<endl;

    /* Start imaging thread */
    driver = new BlackFlyDriver(&camera,&newFrameAvailable,&mymutex,&frameBuffer);
    int requiredMS = 33;
    thread = new QTimer();
    driver->doSetup(thread,requiredMS);
    thread->start(requiredMS);

    return cameraReady;
}

//Prints camera information to stdout
void BlackFlyCamera::printCameraInfo()
{
    if(!cameraReady) return;
    else {
        cout << "Camera ID: " << camInfo.modelName << " by " << camInfo.vendorName<<endl;
    }
}

// Returns image and flags success if image available
Mat *BlackFlyCamera::getImage(bool *success)
{
    if(!cameraReady) {
        (*success) = false;
        return NULL;
    }
    else {
        if(mymutex.tryLock(1)){
            if(newFrameAvailable){
                newFrameAvailable = false;
                buffer = new Mat(frameBuffer.rows,frameBuffer.cols,CV_8UC3,frameBuffer.data,frameBuffer.step);
                buffer->copyTo(holder);
                (*success) = true;
            } else (*success) = false;
            mymutex.unlock();
        } else (*success) = false;
        return &holder;
    }
}

// Returns true if camera was successfully initiated
bool BlackFlyCamera::isCameraReady() //Returns the sate of the camera
{
    return cameraReady;
}

// Returns true if a new frame is available;
bool BlackFlyCamera::frameAvailable() //Returns the existance of a new frame
{
    return newFrameAvailable;
}


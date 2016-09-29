#include "blackflycamera.h"

// Constructor
BlackFlyCamera::BlackFlyCamera()
{
    newFrameAvailable = cameraReady = false; //Flags start as false
    frameBuffer = Mat(480,480,CV_8UC3,Scalar(0,0,0));
}

//Starts camera connection and image capture thread, returns true if successful
bool BlackFlyCamera::startCamera()
{
    // Connect the camera
    error = camera.Connect(0);
    if ( error != PGRERROR_OK ) {
        ROS_ERROR("PGE GiGE Connection Failed (1).");
        cameraReady = false;
        return cameraReady;
    }

    // Get the camera info and print it out
    error = camera.GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK ) {
        ROS_ERROR("PGE GiGE Connection Failed (2).");
        cameraReady = false; 
        return cameraReady;
    }

    cameraReady = true;
    ROS_INFO("PGE GigE Connection successful.");
    return cameraReady;
}

//Closes camera connection and image capture thread
void BlackFlyCamera::closeCamera()
{
    camera.StopCapture();
    camera.Disconnect();
    cameraReady = false;
    ROS_INFO("PGE GigE Disconnection successful.");
}

//Starts image capture thread, returns true if successful
bool BlackFlyCamera::startImaging()
{
    if(!cameraReady) return false;
    else {
        error = camera.StartCapture();
        if ( error != PGRERROR_OK ) {
            ROS_ERROR("PGE GigE Imaging failed.");
            cameraReady = false; return cameraReady;
        }
    }

    cameraReady = true;
    ROS_INFO("PGE GigE Imaging successful.");

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
        //cout << "Camera ID: " << camInfo.modelName << " by " << camInfo.vendorName<<endl;
      ROS_INFO("Cam ID: %s | Vendor: %s", camInfo.modelName, camInfo.vendorName);
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


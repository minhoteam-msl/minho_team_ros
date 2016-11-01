#include "blackflycam.h"

BlackflyCam::BlackflyCam()
{
    camera = new Camera();
    imageready = false;
    newimage = false;
    frameBuffer = new Mat(480,480,CV_8UC3,Scalar(0,0,0));
    calibrate=true;
}

BlackflyCam::~BlackflyCam()
{
    destroyAllWindows();
    closeCamera();
    mutex.unlock();
}

///
/// \brief BlackflyCam::closeCamera
/// \return
///
bool BlackflyCam::closeCamera()
{
    camera->StopCapture();
    error=camera->Disconnect();
    if(error != PGRERROR_OK)return false;
    else return true;
}

///
/// \brief readframe
/// \param pImage
/// \param pCallbackData
///
void readFrame(Image* pImage, const void* pCallbackData )
{
    BlackflyCam *ptr = (BlackflyCam *)pCallbackData;
    ptr->setNewFrame(pImage);
}

///
/// \brief BlackflyCam::startCapture
/// \return
///
bool BlackflyCam::startCapture()
{
    if(!camera->IsConnected()) { ROS_ERROR("GigE Camera not connected!"); return false; }
    else{
        error=camera->StartCapture(readFrame,this);
        if(error!=PGRERROR_OK) { ROS_ERROR("Failed to start GigE Capture!"); return false; }
        timer.start();
        return true;
    }

}

///
/// \brief BlackflyCam::stopCapture
/// \return
///
bool BlackflyCam::stopCapture()
{
    error=camera->StopCapture();
    if(error != PGRERROR_OK)return false;
    else return true;
}

///
/// \brief BlackflyCam::printcamerainfo
///
void BlackflyCam::printCameraInfo()
{
    if(!(camera->IsConnected())) return;
    else {
        ROS_INFO("Camera: %s by %s",camInfo.modelName,camInfo.vendorName);
        ROS_INFO("Serial Nº: %x. Firmware Ver: %s",camInfo.serialNumber,camInfo.firmwareVersion);
    }
}

///
/// \brief BlackflyCam::setnewframe
/// \param pImage
///
void BlackflyCam::setNewFrame(Image *pImage)
{  
    static int count = 0;
    static int count_conf=0;

    if(count_conf>32 || calibrate)
    {
        count_conf=0;
        calibrate=camcalib->cameraCalibrate(*frameBuffer);
    }
    else count_conf++;
    
    fps = 1000/timer.elapsed();
    timer.start();

    pImage->Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rawimage );
    rowBytes = (double)rawimage.GetReceivedDataSize()/(double)rawimage.GetRows();
    {
        lock_guard<Mutex> locker(mutex);
        (*frameBuffer) = Mat(rawimage.GetRows(), rawimage.GetCols(), CV_8UC3, rawimage.GetData(),rowBytes);
    }
    newimage=true;
}

///
/// \brief BlackflyCam::frameAvailable
/// \return
///
bool BlackflyCam::frameAvailable()
{
    return newimage;
}

///
/// \brief BlackflyCam::isCameraReady
/// \return
///
bool BlackflyCam::isCameraReady()
{
    return camera->IsConnected();
}

///
/// \brief BlackflyCam::getFPS
/// \return
///
int BlackflyCam::getFPS()
{
    return fps;
}

///
/// \brief BlackflyCam::getLockingMutex
/// \return
///
Mutex *BlackflyCam::getLockingMutex()
{
    return &mutex;
}

///
/// \brief BlackflyCam::getImage
/// \return
///
Mat *BlackflyCam::getImage()
{
    buffer = NULL;
    if(newimage==true){
        newimage = false;
        {
            lock_guard<Mutex> locker(mutex);
            buffer = new Mat(frameBuffer->rows,frameBuffer->cols,CV_8UC3,frameBuffer->data,frameBuffer->step);
        }
    }
    
    return buffer;
}

///
/// \brief BlackflyCam::connect
/// \return
///
bool BlackflyCam::connect()
{

    error=camera->Connect(0);
    if(error!=PGRERROR_OK){
        ROS_ERROR("GigE Connection Failed.");
        return false;
    }

    error = camera->GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )return false;
    else{
        props = new Properties(camera);
        camcalib = new CameraCalib(props,*frameBuffer);
        cout << "GigE Connection successful."<<endl;
        return true;
    }
}


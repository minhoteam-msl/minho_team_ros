#include "blackflycam.h"

BlackflyCam::BlackflyCam()
{
    camera = new Camera();
    imageready = false;
    newimage = false;
    frameBuffer = new Mat(480,480,CV_8UC3,Scalar(0,0,0));
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
    Q_UNUSED(pImage);
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
/// \brief BlackflyCam::setBrigtness
/// \param value
///
void BlackflyCam::setBrigtness(float value)
{
    props[0].absValue=value;
}
float BlackflyCam::getBrigtness()
{
    return props[0].absValue;
}

bool BlackflyCam::getBrigtnessState()
{
    return props[0].absControl;
}

void BlackflyCam::setBrigtnessState(bool state)
{
    props[0].absControl=state;
}

///
/// \brief BlackflyCam::setShutter
/// \param value
///
void BlackflyCam::setShutter(float value)
{
       props[1].absValue=value;
}
float BlackflyCam::getShuttertime()
{
    return props[1].absValue;
}
void BlackflyCam::setShutterState(bool state)
{
    props[1].absControl=state;
}
bool BlackflyCam::getShutterState()
{
    return props[1].absControl;
}

///
/// \brief BlackflyCam::setGain
/// \param value
///
void BlackflyCam::setGain(float value)
{
    props[2].absValue=value;
}
float BlackflyCam::getGain()
{
    return props[2].absValue;
}

bool BlackflyCam::getGainState()
{
    return props[2].absControl;
}

void BlackflyCam::setGainState(bool state)
{
    props[2].absControl=state;
}

///
/// \brief BlackflyCam::setExposure
/// \param value
///
void BlackflyCam::setExposure(float value)
{
    props[3].absValue=value;
}

float BlackflyCam::getExposure()
{
    return props[3].absValue;
}

bool BlackflyCam::getExposureState()
{
    return props[3].absControl;
}

void BlackflyCam::setExposureState(bool state)
{
    props[3].absControl=state;
}

///
/// \brief BlackflyCam::setGamma
/// \param value
///
void BlackflyCam::setGamma(float value)
{
    props[4].absValue=value;
}

float BlackflyCam::getGamma()
{
    return props[4].absValue;
}

bool BlackflyCam::getGammaState()
{
    return props[4].absControl;
}

void BlackflyCam::setGammaState(bool state)
{
    props[4].absControl=state;
}

///
/// \brief BlackflyCam::setHUE
/// \param value
///
void BlackflyCam::setHUE(float value)
{
    props[5].absValue=value;
}

float BlackflyCam::getHUE()
{
    return props[5].absValue;
}

bool BlackflyCam::getHUEState()
{
    return props[5].absControl;
}

void BlackflyCam::setHUEState(bool state)
{
    props[5].absControl=state;
}

///
/// \brief BlackflyCam::setSaturation
/// \param value
///
void BlackflyCam::setSaturation(float value)
{
    props[6].absValue=value;
}

float BlackflyCam::getSaturation()
{
    return props[6].absValue;
}

bool BlackflyCam::getSaturationState()
{
    return props[6].absControl;
}

void BlackflyCam::setSaturationState(bool state)
{
    props[6].absControl=state;
}

///
/// \brief BlackflyCam::setWhite_Balance
/// \param value
/// \param value2
///
void BlackflyCam::setWhite_Balance(float value, float value2)
{
    props[7].valueA=value;
    props[7].valueB=value2;
}

float BlackflyCam::getWhite_Balance_valueA()
{
    return props[7].valueA;
}

float BlackflyCam::getWhite_Balance_valueB()
{
    return props[7].valueB;
}

bool BlackflyCam::getWhite_BalanceState()
{
    return props[7].absControl;
}

void BlackflyCam::setWhite_BalanceState(bool state)
{
    props[7].absControl=state;
}


void BlackflyCam::setProps(int num)
{
    error=camera->SetProperty(&props[num]);
    // if(error!=PGRERROR_OK)cout<<"Error Ocurred!! Impossible to set properties!!"<<endl;
}

///
/// \brief BlackflyCam::initProps
///
void BlackflyCam::initProps()
{

    //Define the property to adjust.
    props[0].type = BRIGHTNESS;
    //Ensure the property is set up to use absolute value control.
    props[0].absControl = true;
    //Set the absolute value of brightness to 0.5%.
    props[0].absValue = 0.5;

    //Define the property to adjust.
    props[1].type = SHUTTER;
    //Ensure the property is on.
    props[1].onOff = true;
    //Ensure auto-adjust mode is off.
    props[1].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[1].absControl = true;
    //Set the absolute value of shutter to 20 ms.
    props[1].absValue = 20;

    props[2].type = GAIN;
    //Ensure auto-adjust mode is off.
    props[2].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[2].absControl = true;
    //Set the absolute value of gain to 10.5 dB.
    props[2].absValue = 10.5;

    props[3].type = AUTO_EXPOSURE;
    //Ensure the property is on.
    props[3].onOff = true;
    //Ensure auto-adjust mode is off.
    props[3].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[3].absControl = true;
    //Set the absolute value of auto exposure to -3.5 EV.
    props[3].absValue = -3.5;

    //Define the property to adjust.
    props[4].type = GAMMA;
    //Ensure the property is on.
    props[4].onOff = true;
    //Ensure the property is set up to use absolute value control.
    props[4].absControl = true;
    //Set the absolute value of gamma to 1.5
    props[4].absValue = 1.5;

    //Define the property to adjust.
    props[5].type = HUE;
    //Ensure the property is on.
    props[5].onOff = true;
    //Ensure the property is set up to use absolute value control.
    props[5].absControl = true;
    //Set the absolute value of hue to -30 deg.
    props[5].absValue = 15.293;

    //Define the property to adjust.
    props[6].type = SATURATION;
    //Ensure the property is on.
    props[6].onOff = true;
    //Ensure auto-adjust mode is off.
    props[6].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[6].absControl = true;
    //Set the absolute value of saturation to 200%.
    props[6].absValue=200;

    //Define the property to adjust.
    props[7].type = WHITE_BALANCE;
    //Ensure the property is on.
    props[7].onOff = true;
    //Ensure auto-adjust mode is off.
    props[7].autoManualMode = false;
    //Set the white balance red channel to 500.
    props[7].valueA = 500;
    //Set the white balance blue channel to 850.
    props[7].valueB = 850;
}


///
/// \brief BlackflyCam::connect
/// \return
///
bool BlackflyCam::connect()
{

    error=camera->Connect(0);
    if(error!=PGRERROR_OK){
        return false;
    }

    error = camera->GetCameraInfo( &camInfo );
    if ( error != PGRERROR_OK )return false;
    else{
        initProps();
        return true;
    }
}


#include "blackflycam.h"

BlackflyCam::BlackflyCam(bool cal)
{
    calibrate=cal;
    camera = new Camera();
    imageready = false;
    newimage = false;
    frameBuffer = new Mat(480,480,CV_8UC3,Scalar(0,0,0));

    //Parameters for calibration
    roi_black = Rect(0,0,10,10);
    roi_white = Rect(0,0,10,10);
    minError=127*0.05;
    firsttime=true;
}

BlackflyCam::~BlackflyCam()
{
    destroyAllWindows();
    closeCamera();
    mutex.unlock();
}

///
/// \brief Disconnect camera from application
/// \return
///
bool BlackflyCam::closeCamera()
{
    camera->StopCapture();
    error=camera->Disconnect();
    mutex.unlock();
    if(error != PGRERROR_OK)return false;
    else return true;
}

///
/// \brief Defines wich function to call when new image was acquired from camera
/// \param pImage
/// \param pCallbackData
///
void readFrame(Image* pImage, const void* pCallbackData )
{
    BlackflyCam *ptr = (BlackflyCam *)pCallbackData;
    ptr->setNewFrame(pImage);
}

///
/// \brief Makes camera to start capturing images
/// \return
///
bool BlackflyCam::startCapture()
{
    if(!camera->IsConnected()) { ROS_ERROR("GigE Camera not connected!"); return false; }
    else{
        error=camera->StartCapture(readFrame,this);
        if(error!=PGRERROR_OK) { ROS_ERROR("Failed to start GigE Capture!"); return false; }
        return true;
    }

}

///
/// \brief Stop camera from capture images
/// \return
///
bool BlackflyCam::stopCapture()
{
    error=camera->StopCapture();
    if(error != PGRERROR_OK)return false;
    else return true;
}

///
/// \brief Returns Camera Info
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
/// \brief Gives information about fps, converts and update image received from camera,it executes
/// calibration on camera parameters with the intent to get a better image if ambient conditions turn to change
/// \param pImage
///
void BlackflyCam::setNewFrame(Image *pImage)
{  
    static int count_conf=0;
    bool do_calibration = false;
    
    /// Counter for auto calibration
    if(count_conf>32 && calibrate){
        count_conf=0; firsttime=false; do_calibration = true;
    } else count_conf++;
    
    /// Time measure
    clock_gettime(CLOCK_MONOTONIC,&present);
    float temp = 1000000000.0/((float)(present.tv_nsec-past.tv_nsec));
    if(temp>0.0 && temp<=31.0) {
        fps = temp;
    }
    past = present;

    pImage->Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rawimage );
    rowBytes = (double)rawimage.GetReceivedDataSize()/(double)rawimage.GetRows();
    {
        lock_guard<Mutex> locker(mutex);
        (*frameBuffer) = Mat(rawimage.GetRows(), rawimage.GetCols(), CV_8UC3, rawimage.GetData(),rowBytes);
        if(do_calibration)frameBuffer->copyTo(image);
    }
    newimage=true;
    
    if(do_calibration) cameraCalibrate();
}

///
/// \brief Returns true/false if a new image is available
/// \return
///
bool BlackflyCam::frameAvailable()
{
    return newimage;
}

///
/// \brief Returns true/false depending if camera is connected or not
/// \return
///
bool BlackflyCam::isCameraReady()
{
    return camera->IsConnected();
}

///
/// \brief Retunr LockingMutex
/// \return
///
Mutex *BlackflyCam::getLockingMutex()
{
    return &mutex;
}

///
/// \brief Returns the last image captured by the camera
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
/// \brief Connect camera to application and initialize properties
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
        //Inicializa vetor de Properties com valores da camara
        initProps();
        return true;
    }
}

///
/// \brief Return camera FPS
/// \return
///
float BlackflyCam::getFPS()
{
    return fps;
}

/*---------------- SET/GET CAMERA PROPERTIES -------------------------------*/
/*------------------------------------------------------------------------*/

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
    camera->SetProperty(&props[num]);
}

void BlackflyCam::initProps()
{

    //Define the property to adjust.
    props[0].type = BRIGHTNESS;
    camera->GetProperty(&props[0]);
    //Ensure the property is set up to use absolute value control.
    props[0].absControl = true;
    camera->SetProperty(&props[0]);

    //Define the property to adjust.
    props[1].type = SHUTTER;
    camera->GetProperty(&props[1]);
    //Ensure the property is on.
    props[1].onOff = true;
    //Ensure auto-adjust mode is off.
    props[1].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[1].absControl = true;
    camera->SetProperty(&props[1]);

    props[2].type = GAIN;
    camera->GetProperty(&props[2]);
    //Ensure auto-adjust mode is off.
    props[2].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[2].absControl = true;
    camera->SetProperty(&props[2]);

    props[3].type = AUTO_EXPOSURE;
    camera->GetProperty(&props[3]);
    //Ensure the property is on.
    props[3].onOff = true;
    //Ensure auto-adjust mode is off.
    props[3].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[3].absControl = true;
    camera->SetProperty(&props[3]);

    //Define the property to adjust.
    props[4].type = GAMMA;
    camera->GetProperty(&props[4]);
    //Ensure the property is on.
    props[4].onOff = true;
    //Ensure the property is set up to use absolute value control.
    props[4].absControl = true;
    camera->SetProperty(&props[4]);

    //Define the property to adjust.
    props[5].type = HUE;
    camera->GetProperty(&props[5]);
    //Ensure the property is on.
    props[5].onOff = true;
    //Ensure the property is set up to use absolute value control.
    props[5].absControl = true;
    camera->SetProperty(&props[5]);

    //Define the property to adjust.
    props[6].type = SATURATION;
    camera->GetProperty(&props[6]);
    //Ensure the property is on.
    props[6].onOff = true;
    //Ensure auto-adjust mode is off.
    props[6].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[6].absControl = true;
    camera->SetProperty(&props[6]);

    //Define the property to adjust.
    props[7].type = WHITE_BALANCE;
    camera->GetProperty(&props[7]);
    //Ensure the property is on.
    props[7].onOff = true;
    //Ensure auto-adjust mode is off.
    props[7].autoManualMode = false;
    camera->SetProperty(&props[7]);
}



/*----------------CALIBRATION CAMERA PROPERTIES FUNCTIONS -------------------------------*/
/*------------------------------------------------------------------------*/

///
/// \brief Calculates Luminosity histogram
///
void BlackflyCam::calcLumiHistogram()
{
    Mat gray;

    cvtColor(image,gray,CV_RGB2GRAY);

    histvalue.clear();
    histvalue.resize(256);

    for(int i=0;i<gray.cols*image.rows;i++)
      histvalue[(int)gray.ptr()[i]]++;
}

///
/// \brief Calculates Saturation histogram
///
void BlackflyCam::calcSatHistogram()
{
    Mat Hsv;

    cvtColor(image,Hsv,CV_RGB2HSV);

    histvalue.clear();
    histvalue.resize(256);

    for(int i=0;i<Hsv.cols*Hsv.rows;i++)
      histvalue[(int)Hsv.ptr()[i]]++;
}

///
/// \brief Given a region of interest calculates the mean values of RGB
/// \return
///
Scalar BlackflyCam::averageRGB()
{
    int meanR=0,meanG=0,meanB=0,count=0;

    for(int i=0;i<image_roi_black.cols;i++){
        for(int j=0;j<image_roi_black.rows;j++){

            meanR = meanR + image_roi_black.ptr()[(i*image_roi_black.cols + j)*3];
            meanG = meanG + image_roi_black.ptr()[(i*image_roi_black.cols + j)*3 + 1];
            meanB = meanB + image_roi_black.ptr()[(i*image_roi_black.cols + j)*3 + 2];

            count++;
        }
    }
    meanR=meanR/count;
    meanG=meanG/count;
    meanB=meanB/count;


    Scalar s=Scalar(meanR,meanG,meanB);

    return s;
}

///
/// \brief Calculates mean given a vector with histogram values
/// \return
///
float BlackflyCam::calcMean()
{
    float sum=0,temp=0;


    for(int i=0;i<256;i++)
    {
        sum=sum+i*histvalue[i];
        temp=temp+histvalue[i];
    }

    sum=sum/temp;
    return sum;
}

///
/// \brief Given a region of interest calculates the mean values of UV
/// \return
///
Scalar BlackflyCam::averageUV()
{
    int meanU=0,meanV=0,count=0;
    Mat temp;

    cvtColor(image_roi_white,temp,CV_RGB2YUV);

    for(int i=0;i<temp.cols;i++){
        for(int j=0;j<temp.rows;j++){

            meanU = meanU + temp.ptr()[(i*temp.cols + j)*3 + 1];
            meanV = meanV + temp.ptr()[(i*temp.cols + j)*3 + 2];

            count++;
        }
    }
    meanU=meanU/count;
    meanV=meanV/count;
    Scalar s= Scalar(meanU,meanV);

    return s;

}

///
/// \brief Defines regions of interest from our image
///
void BlackflyCam::setRegions(){

    image_roi_white = image(roi_white);
    image_roi_black = image(roi_black);
}

///
/// \brief this function is responsible for controling camera parameters when ambient conditions change
/// \return true/false to give information if some parameters was changed
///
bool BlackflyCam::cameraCalibrate()
{

    setRegions();
    bool changed=false;
    //*******************************PID values start here******************************//
    static float WbKp=0.1;
    static float WbKi=0.0;

    static float WrKp=0.1;
    static float WrKi=0;

    static float GammaKp=5.2;
    static float GammaKi=0.0;

    static float SatKp=5.2;
    static float SatKi=0.0;

    static float BrigKp=5.2;
    static float BrigKi=0.0;

    static float GainKp=5.2;
    static float GainKi=0.0;

    static float ExpKp=5.2;
    static float ExpKi=0.0;

    static float msvError=0.0;

    //*******************************PID values end here******************************//

    float wblue=getWhite_Balance_valueB();
    float wred=getWhite_Balance_valueA();
    float gain=getGain();
    float gamma=getGamma();
    float sat=getSaturation();
    float brig=getBrigtness();
    float exp=getExposure();


    //Initialization of PID's
    PID wBluePID(WbKp,WbKi,0,getWhite_BalanceMin(),getWhite_BalanceMax());
    PID wRedPID(WrKp,WrKi,0,getWhite_BalanceMin(),getWhite_BalanceMax());
    PID GainPID(GainKp,GainKi,0,getGainMin(),getGainMax());
    PID GammaPID(GammaKp,GammaKi,0,getGammaMin(),getGainMax());
    PID SatPID(SatKp,SatKi,0,getSaturationMin(),getSaturationMax());
    PID BrigPID(BrigKp,BrigKi,0,getBrigtnessMin(),getBrigtnessMax());
    PID ExpPID(ExpKp,ExpKi,0,getExposureMin(),getExposureMax());

    //Initialization of some variables
    Scalar rgbMean=averageRGB();
    Scalar uvMean=averageUV();
    calcLumiHistogram();
    int msv=calcMean();
    msvError=127-msv;


    if(firsttime){
        wBluePID.reset();
        wRedPID.reset();
        GainPID.reset();
        GammaPID.reset();
        SatPID.reset();
        BrigPID.reset();
        ExpPID.reset();
        firsttime=false;
    }



    if(msvError>minError || msvError<(-minError))
    {
        changed=true;
        if(gain==getGainMax() && exp==getExposureMax())
        {
            gamma=GammaPID.calc_pid(gamma,msvError);
            setGamma(gamma);
            setProps(GAM);
        }
        else
        {
            if(gain==getGainMax())
            {
                exp=ExpPID.calc_pid(exp,msvError);
                setExposure(exp);
                setProps(EXP);
            }
            else
            {
                gain=GainPID.calc_pid(gain,msvError);
                setGain(gain);
                setProps(GAI);
            }

        }
    }
    if(msv>101 && msv<152)
    {
        calcSatHistogram();
        msvError=127-calcMean();
        if(msvError>minError || msvError<minError)
        {
            changed=true;
            sat=SatPID.calc_pid(sat,msvError);
            setSaturation(sat);
            setProps(SAT);
        }

        if(uvMean[0]>minError || uvMean[0]<(-minError) || uvMean[1]>minError || uvMean[1]<(-minError))
        {
            changed=true;
            wblue=wBluePID.calc_pid(wblue,minError-uvMean[0]);
            wred=wRedPID.calc_pid(wred,minError-uvMean[1]);
            setWhite_Balance(wred,wblue);
            setProps(WB);
        }
        float temp=-100;
        if(rgbMean[0]>minError || rgbMean[1]>minError || rgbMean[2]>minError)
        {
            changed=true;
            for(int i=0;i<3;i++){
                if(temp<rgbMean[i])temp=rgbMean[i];
            }
            brig=BrigPID.calc_pid(brig,temp);
            setBrigtness(brig);
            setProps(BRI);
        }
    }

    return changed;
}




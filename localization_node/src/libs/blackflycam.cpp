#include "blackflycam.h"


// Constructor of the class
BlackflyCam::BlackflyCam(bool cal, int robot_id)
{
    calibrate = cal;
    rob_id = robot_id;

    // Initialize PID gains for Properties controler
    if(!initPidValues(rob_id)){
		ROS_ERROR("Error Reading Properties PID controler gains");
	} else ROS_INFO("Properties PID controler configurations Ready.");

    camera = new Camera();
    imageready = false;
    newimage = false;
    frameBuffer = new Mat(IMG_SIZE,IMG_SIZE,CV_8UC3,Scalar(0,0,0));

    //Parameters for calibration
    minError_Lumi=lumiTarget*0.08;
    minError_Sat=satTarget*0.08;
    minError_RGB=127*0.1;

    firsttime=true;
}

BlackflyCam::~BlackflyCam()
{
    //destroyAllWindows();
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
    static bool do_calibration = false;

    /// Counter for auto calibration
    if(count_conf>32 && calibrate){
        count_conf=0; do_calibration = true;
    } else count_conf++;

    pImage->Convert( FlyCapture2::PIXEL_FORMAT_BGR, &rawimage );
    rowBytes = (double)rawimage.GetReceivedDataSize()/(double)rawimage.GetRows();
    {
        lock_guard<Mutex> locker(mutex);
        (*frameBuffer) = Mat(rawimage.GetRows(), rawimage.GetCols(), CV_8UC3, rawimage.GetData(),rowBytes);
        if(do_calibration)frameBuffer->copyTo(image);
    }
    newimage=true;

    if(do_calibration) do_calibration = cameraCalibrate();
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
        initProps(rob_id);
        return true;
    }
}

///
/// \brief Return camera FPS
/// \return fps (float)
///
float BlackflyCam::getFPS()
{
    return fps;
}

/*---------------- SET/GET CAMERA PROPERTIES -------------------------------*/
/*------------------------------------------------------------------------*/

///
/// \brief Functions that set or return camera Properties
/// \return
///

void BlackflyCam::setBrigtness(float value)
{
    props[BRI].absValue=value;
}
float BlackflyCam::getBrigtness()
{
    return props[BRI].absValue;
}

bool BlackflyCam::getBrigtnessState()
{
    return props[BRI].absControl;
}

void BlackflyCam::setBrigtnessState(bool state)
{
    props[BRI].absControl=state;
}

void BlackflyCam::setShutter(float value)
{
       props[SHU].absValue=value;
}
float BlackflyCam::getShuttertime()
{
    return props[SHU].absValue;
}
void BlackflyCam::setShutterState(bool state)
{
    props[SHU].absControl=state;
}
bool BlackflyCam::getShutterState()
{
    return props[SHU].absControl;
}

void BlackflyCam::setGain(float value)
{
    props[GAI].absValue=value;
}
float BlackflyCam::getGain()
{
    return props[GAI].absValue;
}

bool BlackflyCam::getGainState()
{
    return props[GAI].absControl;
}

void BlackflyCam::setGainState(bool state)
{
    props[GAI].absControl=state;
}

void BlackflyCam::setExposure(float value)
{
    props[EXP].absValue=value;
}

float BlackflyCam::getExposure()
{
    return props[EXP].absValue;
}

bool BlackflyCam::getExposureState()
{
    return props[EXP].absControl;
}

void BlackflyCam::setExposureState(bool state)
{
    props[EXP].absControl=state;
}

void BlackflyCam::setGamma(float value)
{
    props[GAM].absValue=value;
}

float BlackflyCam::getGamma()
{
    return props[GAM].absValue;
}

bool BlackflyCam::getGammaState()
{
    return props[GAM].absControl;
}

void BlackflyCam::setGammaState(bool state)
{
    props[EXP].absControl=state;
}

void BlackflyCam::setSaturation(float value)
{
    props[SAT].absValue=value;
}

float BlackflyCam::getSaturation()
{
    return props[SAT].absValue;
}

bool BlackflyCam::getSaturationState()
{
    return props[SAT].absControl;
}

void BlackflyCam::setSaturationState(bool state)
{
    props[SAT].absControl=state;
}

void BlackflyCam::setWhite_Balance(float value,float value2)
{
    props[WB].valueA=value;
    props[WB].valueB=value2;
}

void BlackflyCam::setWhite_Balance_valueA(float value)
{
    props[WB].valueA=value;
}

void BlackflyCam::setWhite_Balance_valueB(float value)
{
	 props[WB].valueB=value;
}

float BlackflyCam::getWhite_Balance_valueA()
{
    camera->GetProperty(&props[WB]);
    return props[WB].valueA;
}

float BlackflyCam::getWhite_Balance_valueB()
{
    camera->GetProperty(&props[WB]);
    return props[WB].valueB;
}

bool BlackflyCam::getWhite_BalanceState()
{
    return props[WB].absControl;
}

void BlackflyCam::setWhite_BalanceState(bool state)
{
    props[WB].absControl=state;
}

void BlackflyCam::setWhite_BalanceAuto(bool state)
{
    props[WB].autoManualMode = state;
}


void BlackflyCam::setProps(int num)
{
    camera->SetProperty(&props[num]);
}

/*------------------------------------------------------------------------*/
/*------------------------------------------------------------------------*/


///
/// \brief Initialize properties values ensuring that the desire
/// configuration is set
///
bool BlackflyCam::initProps(int robot_id)
{

  QString home = QString::fromStdString(getenv("HOME"));
  QString commonDir = home+QString(COMMON_PATH);
  QString cfgDir = commonDir+QString(LOC_CFG_PATH);

  propertyPath = cfgDir+"/"+"Robot"+QString::number(robot_id)+"/"+(QString(PROPERTYFILENAME));

  QFile file(propertyPath);
  if(!file.open(QIODevice::ReadOnly)) {
    return false;
  }
  QTextStream in(&file);
  QString values = in.readLine();
  values = values.right(values.size()-values.indexOf('=')-1);
  QStringList propConf = values.split(",");

    //Define the property to adjust.
    props[BRI].type = BRIGHTNESS;
    camera->GetProperty(&props[BRI]);
    //Ensure the property is set up to use absolute value control.
    props[BRI].absControl = true;
    props[BRI].absValue = propConf.at(0).toDouble();
    camera->SetProperty(&props[BRI]);

    //Define the property to adjust.
    props[SHU].type = SHUTTER;
    camera->GetProperty(&props[SHU]);
    //Ensure the property is on.
    props[SHU].onOff = true;
    //Ensure auto-adjust mode is off.
    props[SHU].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[SHU].absControl = true;
    props[SHU].absValue = propConf.at(1).toDouble();
    camera->SetProperty(&props[SHU]);

    props[GAI].type = GAIN;
    camera->GetProperty(&props[GAI]);
    //Ensure auto-adjust mode is off.
    props[GAI].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[GAI].absControl = true;
    props[GAI].absValue = propConf.at(2).toDouble();
    camera->SetProperty(&props[GAI]);

    props[EXP].type = AUTO_EXPOSURE;
    camera->GetProperty(&props[EXP]);
    //Ensure the property is on.
    props[EXP].onOff = true;
    //Ensure auto-adjust mode is off.
    props[EXP].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[EXP].absControl = true;
    //props[EXP].absolute = propConf.at(3).toDouble();
    camera->SetProperty(&props[EXP]);

    //Define the property to adjust.
    props[GAM].type = GAMMA;
    camera->GetProperty(&props[GAM]);
    //Ensure the property is on.
    props[GAM].onOff = true;
    //Ensure the property is set up to use absolute value control.
    props[GAM].absControl = true;
    props[GAM].absValue = propConf.at(4).toDouble();
    camera->SetProperty(&props[GAM]);

    //Define the property to adjust.
    props[SAT].type = SATURATION;
    camera->GetProperty(&props[SAT]);
    //Ensure the property is on.
    props[SAT].onOff = true;
    //Ensure auto-adjust mode is off.
    props[SAT].autoManualMode = false;
    //Ensure the property is set up to use absolute value control.
    props[SAT].absControl = true;
    props[SAT].absValue = propConf.at(5).toDouble();
    camera->SetProperty(&props[SAT]);

    //Define the property to adjust.
    props[WB].type = WHITE_BALANCE;
    camera->GetProperty(&props[WB]);
    //Ensure the property is on.
    props[WB].onOff = true;
    //Ensure auto-adjust mode is off.
    props[WB].autoManualMode = false;
    //props[WB].valueA = propConf.at(6).toDouble(); // red
    props[WB].valueB = propConf.at(7).toDouble(); // blue
    camera->SetProperty(&props[WB]);

    file.close();
    return true;
}



/*----------------CALIBRATION CAMERA PROPERTIES FUNCTIONS -------------------------------*/
/*------------------------------------------------------------------------*/

///
/// \brief Calculates Luminosity histogram
///
void BlackflyCam::calcLumiHistogram()
{
  static bool first = true;
  Mat gray;

  cvtColor(image,gray,CV_RGB2GRAY);

  histvalue.clear();
  histvalue.resize(256);

  if(first==true){
      for(int i=0;i<gray.cols*image.rows;i++) histvalue[(int)gray.ptr()[i]]++;
      first = false;
  }

  else{
      for(int i=0;i<gray.cols*image.rows;i=i+10)
          histvalue[(int)gray.ptr()[i]]++;
  }
}

///
/// \brief Calculates Saturation histogram
///
void BlackflyCam::calcSatHistogram()
{
  bool static first = true;
  Mat Hsv;

  cvtColor(image,Hsv,CV_RGB2HSV);

  histvalue.clear();
  histvalue.resize(256);

  if(first){
      for(int i=0;i<Hsv.cols*Hsv.rows;i++) histvalue[(int)Hsv.ptr()[i]]++;
      first = false;
  }
  else{
      for(int i=0;i<Hsv.cols*Hsv.rows;i++)
          histvalue[(int)Hsv.ptr()[i]]++;
  }
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

    //image_roi_white = image(roi_white);
    image_roi_black = image(roi_black);
}

///
/// \brief this function is responsible for controling camera parameters when ambient conditions change
/// \return true/false to give information if some parameters was changed
///
bool BlackflyCam::cameraCalibrate()
{
    setRegions();
    changed=false;

    float gain=getGain();
    float sat=getSaturation();
    float brig=getBrigtness();
    float shu=getShuttertime();

    if(firsttime){
        GainPID->reset();
        SatPID->reset();
        BrigPID->reset();
        ShuPID->reset();
        firsttime=false;
    }

    //Initialization of some variables
    calcLumiHistogram();
    msv=calcMean();
    msvError=lumiTarget-msv;
    //std::cerr << msv << endl;
    if(msvError>minError_Lumi || msvError<-(minError_Lumi))
    {
        changed = true;
        if(gain>=(getGainMax()-(getGainMax()/4))){
            shu = ShuPID->calc_pid(shu,msvError);
            setShutter(SHU);
            setProps(GAI);
            gain = 0;
            setGain(GAI);
            setProps(GAI);
        }
        else{
            gain = GainPID->calc_pid(gain,msvError);
            setGain(gain);
            setProps(GAI);
        }
    }

    if(!changed){
      calcSatHistogram();
      msv = calcMean();
      msvError=satTarget-msv;

      if(msvError>minError_Sat || msvError<(-minError_Sat))
      {
        changed=true;
        sat=SatPID->calc_pid(sat,msvError);
        setSaturation(sat);
        setProps(SAT);
      }

      Scalar rgbMean=averageRGB();
      if((rgbMean[0]>minError_RGB || rgbMean[1]>minError_RGB || rgbMean[2]>minError_RGB) && brig<=(getBrigtnessMax()/2))
      {
        //std::cerr << "Entra sempre na calibração do brightness" << endl;
        changed = true;
        float rgbError = 0;
        for(int i = 0; i < 3; i++)rgbError += rgbMean[i];
        brig = BrigPID->calc_pid(brig,rgbError/3);
        if(brig>(getBrigtnessMax()-getBrigtnessMax()/4))brig = (getBrigtnessMax()/2);
        setBrigtness(brig);
        setProps(BRI);
      }
    }
  return changed;
}


///
/// \brief Function responsible for setting PID controler of Properties
/// to the respective property object
///
void BlackflyCam::setPropControlPID(int id, float p, float i, float d, bool blue)
{
	if(id<=5 && id>=0){
		if(id==0)BrigPID->setPIDValues(p,i,d);
		if(id==1)GainPID->setPIDValues(p,i,d);
		if(id==2)ShuPID->setPIDValues(p,i,d);
		if(id==3)GammaPID->setPIDValues(p,i,d);
		if(id==4)SatPID->setPIDValues(p,i,d);
		if(id==5 && blue==true)wBluePID->setPIDValues(p,i,d);
			else if(id==5)wRedPID->setPIDValues(p,i,d);
	firsttime=true;
	writePIDConfig();
	}
	else ROS_ERROR("Property ID Out Of Range");
}

///
/// \brief Initializes PID controler of Properties readed from a file
///
bool BlackflyCam::initPidValues(int robot_id)
{
    float pid_conf[21];

    QString home = QString::fromStdString(getenv("HOME"));
    QString commonDir = home+QString(COMMON_PATH);
    QString cfgDir = commonDir+QString(LOC_CFG_PATH);

    pidPath = cfgDir+"/"+"Robot"+QString::number(robot_id)+"/"+(QString(PIDFILENAME));

	  QFile file(pidPath);
    if(!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    int count_list=0;
    QString line;
    QTextStream in(&file);
    QStringList pid_list;

    for(int i=0;i<21;i=i+3){
		line = in.readLine();
		pid_list = line.right(line.size()-line.indexOf('=')-1).split(",");
		pid_conf[i]=pid_list[0].toFloat();
		pid_conf[i+1]=pid_list[1].toFloat();
		pid_conf[i+2]=pid_list[2].toFloat();
		if(pid_list.size()==0)count_list++;
	}

	//This is to do not read EXPOSSURE VALUES of file
	line = in.readLine();

	//LEITURA DOS ROIS
	for(int i=0;i<2;i++){
		line = in.readLine();
		pid_list = line.right(line.size()-line.indexOf('=')-1).split(",");
		if(i==1)roi_black = Rect(pid_list[0].toInt(),pid_list[1].toInt(),pid_list[2].toInt(),pid_list[2].toInt());
		else roi_white = Rect(pid_list[0].toInt(),pid_list[1].toInt(),pid_list[2].toInt(),pid_list[2].toInt());
		}

    line = in.readLine();
    pid_list = line.right(line.size()-line.indexOf('=')-1).split(",");
    lumiTarget = pid_list[0].toInt();
    satTarget = pid_list[1].toInt();

    std::cerr << "Targets: " << lumiTarget << "  " << satTarget << endl;

    file.close();

    if(count_list!=0){
		ROS_ERROR("Bad Configuration in %s",PIDFILENAME);
		return false;
	}

    // Initialization of PID's
    BrigPID = new PID(pid_conf[0],pid_conf[1],pid_conf[2],getBrigtnessMin(),getBrigtnessMax());
    GainPID = new PID(pid_conf[3],pid_conf[4],pid_conf[5],getGainMin(),getGainMax());
    ShuPID = new PID(pid_conf[6],pid_conf[7],pid_conf[8],getExposureMin(),getExposureMax());
    GammaPID = new PID(pid_conf[9],pid_conf[10],pid_conf[11],getGammaMin(),getGainMax());
    SatPID = new PID(pid_conf[12],pid_conf[13],pid_conf[14],getSaturationMin(),getSaturationMax());
    wBluePID = new PID(pid_conf[18],pid_conf[19],pid_conf[20],getWhite_BalanceMin(),getWhite_BalanceMax());
    wRedPID = new PID(pid_conf[15],pid_conf[16],pid_conf[17],getWhite_BalanceMin(),getWhite_BalanceMax());

    return true;
}

///
/// \brief Function responsible for writing file with PID controlers values
///
bool BlackflyCam::writePIDConfig()
{
	QFile file(pidPath);
        if(!file.open(QIODevice::WriteOnly)){
        ROS_ERROR("Error writing to %s.",PIDFILENAME);
        return false;
    }
    QTextStream in(&file);

	in<<"BRIGTNESS="<<BrigPID->getP()<<","<<BrigPID->getI()<<","<<BrigPID->getD()<<"\r\n";
	in<<"GAIN="<<GainPID->getP()<<","<<GainPID->getI()<<","<<GainPID->getD()<<"\r\n";
	in<<"SHUTTER="<<ShuPID->getP()<<","<<ShuPID->getI()<<","<<ShuPID->getD()<<"\r\n";
	in<<"GAMMA="<<GammaPID->getP()<<","<<GammaPID->getI()<<","<<GammaPID->getD()<<"\r\n";
	in<<"SATURATION="<<SatPID->getP()<<","<<SatPID->getI()<<","<<SatPID->getD()<<"\r\n";
	in<<"WB_red="<<wRedPID->getP()<<","<<wRedPID->getI()<<","<<wRedPID->getD()<<"\r\n";
	in<<"WB_blue="<<wBluePID->getP()<<","<<wBluePID->getI()<<","<<wBluePID->getD()<<"\r\n";
	in<<"EXPOSSURE="<<"0,0,0"<<"\r\n";
	in<<"WHITE="<<roi_white.x<<","<<roi_white.y<<","<<roi_white.height<<"\r\n";
	in<<"BLACK="<<roi_black.x<<","<<roi_black.y<<","<<roi_black.height<<"\r\n";
  in<<"TARGETS="<<lumiTarget<<","<<satTarget<<"\r\n";
	QString message = QString("#Property=Kp, Ki, Kd\r\n");
    in << message;
    message = QString("#ROI=X,Y,AREA\r\n#TARGETS=LUMI, SAT");
    in<<message;
    message = QString("#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
    in << message;
    file.close();
    return true;
}

void BlackflyCam::setWhiteROI(int x, int y, int h)
{
	roi_white.x = x;
	roi_white.y = y;
	roi_white.height = h;
  roi_white.width = h;
	writePIDConfig();
}

void BlackflyCam::setBlackROI(int x, int y, int h)
{
	roi_black.x=x;
	roi_black.y=y;
	roi_black.height=roi_white.width=h;
	writePIDConfig();
}

bool BlackflyCam::writePropConfig()
{
	QFile file(propertyPath);
  if(!file.open(QIODevice::WriteOnly)){
        ROS_ERROR("Error writing to %s.",PROPERTYFILENAME);
        return false;
    }

  QTextStream in(&file);

  in << getBrigtness()<< "," << getShuttertime() << "," << getGain() << "," << getExposure() << ","
  << getGamma() << "," << getSaturation() << "," << getWhite_Balance_valueA() << "," << getWhite_Balance_valueB()<<"\r\n";
  QString message = QString("#PROPERTYS=BRIGTNESS,SHUTTER,GAIN,EXPOSSURE,GAMMA,SATURATION,WB_red,WB_blue\r\n");
  in << message;
  message = QString("#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
  in << message;
  file.close();
  return true;
}


Point2d BlackflyCam::getError(int prop_in_use)
{
	Point2d point;
  point.x = 0.00;
  point.y = 0.00;
		if(prop_in_use<=6 && prop_in_use>=0 && calibrate){
			if(prop_in_use==0){point.x=BrigPID->getError(); point.y=getBrigtness();}
			if(prop_in_use==1){point.x=GainPID->getError(); point.y=getGain();}
			if(prop_in_use==2){point.x=GainPID->getError(); point.y=getShuttertime();}
			if(prop_in_use==3){point.x=GainPID->getError(); point.y=getGamma();}
			if(prop_in_use==4){point.x=SatPID->getError(); point.y=getSaturation();}
			if(prop_in_use==5){point.x=wRedPID->getError(); point.y=getWhite_Balance_valueA();}
			if(prop_in_use==6){point.x=wBluePID->getError(); point.y=getWhite_Balance_valueB();}
		return point;
	}
	else {
		//ROS_ERROR("Property ID Out Of Range %d",prop_in_use);point.x=0.00;
		return point;
	}
}

void BlackflyCam::setSatTarget(int val)
{
  satTarget = val;
  minError_Sat=satTarget*0.08;
  writePIDConfig();
}

void BlackflyCam::setLumiTarget(int val)
{
  lumiTarget = val;
  minError_Lumi=lumiTarget*0.08;
  writePIDConfig();
}


int BlackflyCam::getSatTarget()
{
  return satTarget;
}


int BlackflyCam::getLumiTarget()
{
  return lumiTarget;
}

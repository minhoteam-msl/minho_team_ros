#include "localization.h"

Localization::Localization(int rob_id, ros::NodeHandle *par , bool *init_success, bool use_camera,int side, QObject *parent) : QObject(parent)
{
   initVariables();
   camera = use_camera;
   fieldSide = side;
   if(!use_camera) is_hardware_ready = true; // for test purposes
   else is_hardware_ready = false;
   //#### Initialize major components ####
   //#####################################
   bool correct_initialization = true;
   processor = new ImageProcessor(rob_id,use_camera,&correct_initialization); // true -> use camera
   if(!correct_initialization) { (*init_success) = false; return; }
   confserver = new ConfigServer(par);

   correct_initialization = initWorldMap();
   if(!correct_initialization) { (*init_success) = false; return; }
   correct_initialization = initializeKalmanFilter();
   if(!correct_initialization) { (*init_success) = false; return; }


   confserver->setOmniVisionConf(processor->getMirrorConfAsMsg(),processor->getVisionConfAsMsg(),processor->getImageConfAsMsg(), processor->getBallConfAsMsg(), processor->getObsConfAsMsg(), processor->getRLEConfAsMsg());
   confserver->setProps(processor->getCamProperties());
   confserver->setPIDValues(processor->getPropControlerPID());
   confserver->setROIs(processor->getRois());
   confserver->setKalman(kalman.Q.x,kalman.Q.z,kalman.R.x,kalman.R.z);
   parentTimer = new QTimer();
   requiredTiming = 33; //ms
   time_interval = (float)requiredTiming/1000.0;
   connect(parentTimer,SIGNAL(timeout()),this,SLOT(discoverWorldModel()));
   //#####################################

   //##################### Setup connections to confserver ##################
   //#########################################################################
   connect(confserver,SIGNAL(stopImageAssigning()),this,SLOT(stopImageAssigning()));
   connect(confserver,SIGNAL(changedImageRequest(uint8_t)),this,SLOT(changeImageAssigning(uint8_t)));
   connect(confserver,SIGNAL(changedMirrorConfiguration(mirrorConfig::ConstPtr)),this,SLOT(changeMirrorConfiguration(mirrorConfig::ConstPtr)));
   connect(confserver,SIGNAL(changedLutConfiguration(visionHSVConfig::ConstPtr)),this,SLOT(changeLookUpTableConfiguration(visionHSVConfig::ConstPtr)));
   connect(confserver,SIGNAL(changedImageConfiguration(imageConfig::ConstPtr)),this,SLOT(changeImageConfiguration(imageConfig::ConstPtr)));
   connect(confserver,SIGNAL(changedCameraProperties(cameraProperty::ConstPtr)),this,SLOT(changeCameraProperties(cameraProperty::ConstPtr)));
   connect(confserver,SIGNAL(changedCamPID(PID::ConstPtr)),this,SLOT(changeCamPID(PID::ConstPtr)));
   connect(confserver,SIGNAL(changedROIConfiguration(ROI::ConstPtr)),this,SLOT(changeROI(ROI::ConstPtr)));
   connect(confserver,SIGNAL(changedWorldConfiguration(worldConfig::ConstPtr)),this,SLOT(changeWorldConfiguration(worldConfig::ConstPtr)));
   //#########################################################################

   //## Setup ROS Pubs and Subs ##
   //#############################
   //Initialize robotInfo publisher
   robot_info_pub = par->advertise<robotInfo>("robotInfo", 1);
	//Initialize hardwareInfo subscriber
	hardware_info_sub = par->subscribe("hardwareInfo",
                                       1,
                                       &Localization::hardwareCallback,
                                       this);
   reloc_service = par->advertiseService("requestReloc",
                                    &Localization::doReloc,
                                    this);
   ext_debug_service = par->advertiseService("requestExtendedDebug",
                                    &Localization::setExtDebug,
                                    this);
   //#############################

   reloc = true;
   parentTimer->start(33);
}

Localization::~Localization()
{

}

void Localization::discoverWorldModel() // Main Function
{
   bool have_image = false;
   int timing = 0;
   float meanTime = 0.0;
   parentTimer->stop();
   QTime measure, loctime;
   measure.start();
   // Acquire image using function pointer
   buffer = CALL_MEMBER_FN((*processor),processor->acquireImage)(&have_image);
   // Localization code
   is_hardware_ready=true;
   if(have_image && is_hardware_ready){
      current_hardware_state.imu_value = 0;
      if(reloc){
        last_state.robot_pose.z = current_hardware_state.imu_value;
        //ROS_INFO("Reloc Angle: %d",current_hardware_state.imu_value);
      }

      //loctime.start();
      // Detect points of interest like lines, obstacles and ball
      processor->detectInterestPoints(); //  máximo 10ms que demorou (em média)
      // Calculate Orientation with histograms
      calcHistOrientation();
      // Fuse odometry and vision orientation with kalmanFilter
      fuseOrientationEstimates();
      // Maps points to real World
      processor->mapPoints(current_state.robot_pose.z);
      //std::cerr << "Tempo de deteção dos pontos:  " << loctime.elapsed() <<endl;



      if(reloc){ // Global localization

         // Do global localization
         //loctime.start();
         reloc = computeGlobalLocalization(fieldSide);
         //std::cerr << "Tempo localização:  " << double(loctime.elapsed())<<endl;

      } else {  // Local localization

         //loctime.start();
         // Do local localization
         computeLocalLocalization();
         fuseEstimates();

         //std::cerr << "Tempo localização:  " << loctime.elapsed() <<endl;
      }

      processor->creatWorld(); // Creat World arround robot (obstacle and ball positions)
      computeVelocities(); // Calculates robot velocity
      decideBallPossession(); // Decides if robot posses ball
      sendWorldInfo(); // Load obstacls to current_state for further publish (only sends Blobs)
      generateDebugData(); // generate Debug Data
      memset(&odometry,0,sizeof(localizationEstimate)); // Clears odometry object to do nto use past values

	    robot_info_pub.publish(current_state); // Publish information
	    last_state = current_state; // Saves current_state to next iteration
   }

   if(!have_image) parentTimer->start(1);
   else {
      //Send image to confserver if requested
      if(assigning_images){
         if(assigning_type==0) {
			processed = buffer->clone();
			confserver->assignImage(&processed);
         } else if(assigning_type==1) {
            processed = buffer->clone();
            processor->getSegmentedImage(&processed);
            confserver->assignImage(&processed);
         } else if(assigning_type==3) {
            processed = buffer->clone();
            processor->drawInterestInfo(&processed);
            confserver->assignImage(&processed);
         } else if(assigning_type==4) {
            processed = buffer->clone();
            processor->drawWorldInfo(&processed);
            confserver->assignImage(&processed);
         } else if(assigning_type==5){
			      processed = buffer->clone();
			      processor->drawWorldPoints(&processed);
			      confserver->assignImage(&processed);
		 } else if(assigning_type==2){
			processed = buffer->clone();
			processor->drawScanlines(&processed);
			confserver->assignImage(&processed);
		 }
     if(confserver->getCalib())confserver->sendError(processor->getPropError(confserver->prop_used));
    }

      //Compensate time took in the function
      timing = requiredTiming-measure.elapsed();
      if(timing<0) timing = requiredTiming;

      //std::cerr << "Tempo localização:  " << measure.elapsed() <<endl;
      parentTimer->start(timing);
   }

}

void Localization::sendWorldInfo()
{
  current_state.obstacles.clear();
  obstacle p;
  for(unsigned i = 0; i < processor->obsBlob.UMblobs.size(); i++){
    p.x = processor->obsBlob.UMblobs[i].center.x+current_state.robot_pose.x;
    p.y = processor->obsBlob.UMblobs[i].center.y+current_state.robot_pose.y;
    current_state.obstacles.push_back(p);
  }
}

void Localization::initVariables()
{
   qRegisterMetaType<uint8_t>("uint8_t");
   qRegisterMetaType<mirrorConfig::ConstPtr>("mirrorConfig::ConstPtr");
   qRegisterMetaType<visionHSVConfig::ConstPtr>("visionHSVConfig::ConstPtr");
   qRegisterMetaType<imageConfig::ConstPtr>("imageConfig::ConstPtr");
   qRegisterMetaType<cameraProperty::ConstPtr>("cameraProperty::ConstPtr");
   qRegisterMetaType<PID::ConstPtr>("PID::ConstPtr");
   qRegisterMetaType<ROI::ConstPtr>("ROI::ConstPtr");
   qRegisterMetaType<worldConfig::ConstPtr>("worldConfig::ConstPtr");

   assigning_images = false;
   is_hardware_ready = false;
   generate_extended_debug = false;
   service_call_counter = 0;
}

bool Localization::initializeKalmanFilter()
{
  QString filename;
  filename = processor->getKalmanFileName();

  QFile file(filename);
  if(!file.open(QIODevice::ReadOnly)) {
      ROS_ERROR("Error reading %s.",filename.toStdString().c_str());
      return false;
  }

  QTextStream in(&file);

  QString text = in.readLine();
  text = text.right(text.size()-text.indexOf('=')-1);
  QStringList values = text.split(",");
  kalman.Q.x = kalman.Q.y = values.at(0).toInt();
  kalman.Q.z = values.at(1).toInt();

  text = in.readLine();
  text = text.right(text.size()-text.indexOf('=')-1);
  values = text.split(",");
  kalman.R.x = kalman.R.y = values.at(0).toInt();
  kalman.R.z = values.at(1).toInt();

  memset(&odometry,0,sizeof(localizationEstimate));
  memset(&vision,0,sizeof(localizationEstimate));
  kalman.lastCovariance.x = kalman.lastCovariance.y = kalman.lastCovariance.z = 0;
  kalman.covariance = kalman.predictedCovariance = kalman.lastCovariance;

  file.close();
  return true;
}

bool Localization::WriteKalmanFilter()
{
  QString filename;
  filename = processor->getKalmanFileName();
  QFile file(filename);

  if(!file.open(QIODevice::WriteOnly)){
  ROS_ERROR("Error writing to %s.",KALMANFILENAME);
  return false;
}
  QTextStream in(&file);


  in << "Q=" << kalman.Q.x << "," << kalman.Q.z << "\r\n";
  in << "R=" << kalman.R.x << "," << kalman.R.z << "\r\n";
  QString message = QString("#WEIGHT=(XY),Z\r\n#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
  in << message;

  file.close();
  return true;
}

void Localization::setKalman(worldConfig::ConstPtr msg)
{
  kalman.Q.x = kalman.Q.y = (int)msg->value_a;
  kalman.Q.z = msg->value_b;
  kalman.R.x = msg->value_c;
  kalman.R.z = msg->window;

  WriteKalmanFilter();
}

bool Localization::initWorldMap()
{
   QString filename;
   filename = processor->getWorldFileName();

   QFile file(filename);
   if(!file.open(QIODevice::ReadOnly)) {
       ROS_ERROR("Error reading %s.",filename.toStdString().c_str());
       return false;
   }
   QTextStream in(&file);

   QStringList mapConfigs = in.readLine().split(",");
   currentField.FIELD_LENGTH = mapConfigs.at(0).toDouble();
   currentField.FIELD_WIDTH = mapConfigs.at(1).toDouble();
   currentField.FIELD_POSITIONS = mapConfigs.at(2).toDouble();
   currentField.FIELD_NAME = mapConfigs.at(3);
   if(mapConfigs.at(4)=="m")currentField.SCALE = 0.1;
   else currentField.SCALE = 100;
   currentField.HALF_FIELD_LENGTH = currentField.FIELD_LENGTH/2.0;
   currentField.HALF_FIELD_WIDTH = currentField.FIELD_WIDTH/2.0;
   currentField.TERM1 = (currentField.FIELD_LENGTH*10)/2;
   currentField.TERM2 = (currentField.FIELD_WIDTH*10);
   currentField.TERM3 = (currentField.FIELD_WIDTH*10)/2;

   // Tamanho da matriz
   int x = float(currentField.FIELD_LENGTH/currentField.SCALE)+1;
   int y = float(currentField.FIELD_WIDTH/currentField.SCALE)+1;

   WorldMap = (struct nodo**)malloc(x * sizeof(struct nodo *));
   for(int i = 0; i < x ; i++){
     WorldMap[i] = (struct nodo*) malloc(y * sizeof(struct nodo));
   }

   QStringList values,positions;
   QString auxPos,auxVal;

   for(unsigned int i=0;i<currentField.FIELD_POSITIONS;i++){
       values = in.readLine().split(")");
       auxVal = values.at(1);
       auxVal.remove(0,1);
       auxPos = values.at(0);
       auxPos.remove(0,1);
       positions = auxPos.split(",");

       WorldMap[getFieldIndexX(positions.at(0).toDouble())][getFieldIndexY(positions.at(1).toDouble())].x = positions.at(0).toDouble();
       WorldMap[getFieldIndexX(positions.at(0).toDouble())][getFieldIndexY(positions.at(1).toDouble())].y = positions.at(1).toDouble();
       WorldMap[getFieldIndexX(positions.at(0).toDouble())][getFieldIndexY(positions.at(1).toDouble())].closestDistance = auxVal.toDouble();
   }

   file.close();
   return true;
}

/*************************************************************************************/
/*************************** ConfServer Functions needed *****************************/
void Localization::stopImageAssigning()
{
   assigning_images = false;
}

void Localization::changeImageAssigning(uint8_t type)
{
   assigning_type = type;
   assigning_images = true;
}

void Localization::changeLookUpTableConfiguration(visionHSVConfig::ConstPtr msg)
{
   ROS_WARN("New configuration of Look Up Table will be set.");
   parentTimer->stop();
   processor->updateLabelLutConf(FIELD,msg->field);
   processor->updateLabelLutConf(LINE,msg->line);
   processor->updateLabelLutConf(BALL,msg->ball);
   processor->updateLabelLutConf(OBSTACLE,msg->obstacle);
   processor->generateLookUpTable();
   if(processor->writeLookUpTable())ROS_INFO("New %s saved!",LUTFILENAME);
   parentTimer->start(requiredTiming);
}
void Localization::changeMirrorConfiguration(mirrorConfig::ConstPtr msg)
{
   ROS_WARN("New configuration of mirror will be set.");
   parentTimer->stop();
   processor->updateDists(msg->max_distance,msg->step,msg->pixel_distances, msg->lines_length);
   processor->generateMirrorConfiguration();
   if(processor->writeMirrorConfig())ROS_INFO("New %s saved!",MIRRORFILENAME);
   parentTimer->start(requiredTiming);
}

void Localization::changeImageConfiguration(imageConfig::ConstPtr msg)
{
   ROS_WARN("New configuration of image will be set.");
   parentTimer->stop();
   processor->setCenter(msg->center_x,msg->center_y,msg->tilt);
   processor->generateMirrorConfiguration();
   if(processor->writeImageConfig())ROS_INFO("New %s saved!",IMAGEFILENAME);
   parentTimer->start(requiredTiming);
}

void Localization::changeCameraProperties(cameraProperty::ConstPtr msg)
{
  if(camera){
     ROS_WARN("New property value set.");
     parentTimer->stop();
     if(msg->targets == false)processor->setCamPropreties(msg);
     else processor->setCalibrationTargets(msg);
     parentTimer->start(requiredTiming);
   }
   else ROS_WARN("Camera not Connected!.");
}

void Localization::changeCamPID(PID::ConstPtr msg)
{
  if(camera){
     parentTimer->stop();
     processor->setPropControlerPID(msg);
     parentTimer->start(requiredTiming);
   }
  else ROS_WARN("Camera not Connected!.");
}
void Localization::changeROI(ROI::ConstPtr msg)
{
  if(camera){
    ROS_WARN("New ROI set. ");
    parentTimer->stop();
    processor->setROIs(msg);
    parentTimer->start(requiredTiming);
  }
  else ROS_WARN("Camera not Connected!.");
}

void Localization::changeWorldConfiguration(worldConfig::ConstPtr msg)
{
  ROS_INFO("New World Configuration set. ");
  parentTimer->stop();
  if(msg->RLE == true)processor->changeRLEConfiguration(msg);
  else if(msg->RLE == false && msg->kalman == false)processor->changeBlobsConfiguration(msg);
       else if(msg->kalman == true){
         setKalman(msg);
       }
            else ROS_ERROR("ERROR SENDING WORLD CONFIGURATION. ");
  parentTimer->start(requiredTiming);
}
/*************************************************************************************/
/*************************************************************************************/

bool Localization::doReloc(requestReloc::Request &req,requestReloc::Response &res)
{
   current_state.robot_pose.x = 0;
   current_state.robot_pose.y = 0;
   current_state.robot_velocity.x = 0;
   current_state.robot_velocity.y = 0;
   current_state.robot_velocity.w = 0;
   memset(&odometry,0,sizeof(localizationEstimate));
   memset(&vision,0,sizeof(localizationEstimate));
   last_state.robot_pose = current_state.robot_pose;
   last_state.robot_velocity = current_state.robot_velocity;

   reloc = true;
   return true;
}


bool Localization::setExtDebug(requestExtendedDebug::Request &req,requestExtendedDebug::Response &res){
   generate_extended_debug = req.requested;
   res.success = true;
   service_call_counter++;
   return true;
}


void Localization::hardwareCallback(const hardwareInfo::ConstPtr &msg)
{
   // Process hardware information
   static int receivedFrames = 0;
   current_hardware_state = *msg;

   if(!is_hardware_ready){
		receivedFrames++;
		if(receivedFrames>10) is_hardware_ready = true;
		else {
		   current_state.robot_pose.z = msg->imu_value;
		   last_state.robot_pose.z = current_state.robot_pose.z;
		   vision.angle = msg->imu_value;
		}
		last_hardware_state = current_hardware_state;
   }

   // Perform Odometry Calculations here
   double v1, v2, v3, v, vn, w, halfTeta;
   int deltaenc1 = 0.0, deltaenc2 = 0.0, deltaenc3 = 0.0;

   // Normalize differences in encoder values due to wrap around (overflow)
   if(current_hardware_state.encoder_1*last_hardware_state.encoder_1<0 && abs(current_hardware_state.encoder_1)>16384){
       if(current_hardware_state.encoder_1<0){
            deltaenc1 = 32768+current_hardware_state.encoder_1+32768-last_hardware_state.encoder_1;
       } else {
           deltaenc1 = 32768-current_hardware_state.encoder_1+32768+last_hardware_state.encoder_1;
       }
   } else deltaenc1 = current_hardware_state.encoder_1-last_hardware_state.encoder_1;

   if(current_hardware_state.encoder_2*last_hardware_state.encoder_2<0 && abs(current_hardware_state.encoder_2)>16384){
       if(current_hardware_state.encoder_2<0){
            deltaenc2 = 32768+current_hardware_state.encoder_2+32768-last_hardware_state.encoder_2;
       } else {
           deltaenc2 = 32768-current_hardware_state.encoder_2+32768+last_hardware_state.encoder_2;
       }
   } else deltaenc2 = current_hardware_state.encoder_2-last_hardware_state.encoder_2;

   if(current_hardware_state.encoder_3*last_hardware_state.encoder_3<0 && abs(current_hardware_state.encoder_3)>16384){
       if(current_hardware_state.encoder_3<0){
            deltaenc3 = 32768+current_hardware_state.encoder_3+32768-last_hardware_state.encoder_3;
       } else {
           deltaenc3 = 32768-current_hardware_state.encoder_3+32768+last_hardware_state.encoder_3;
       }
   } else deltaenc3 = current_hardware_state.encoder_3-last_hardware_state.encoder_3;

   // KWheels converts encoder ticks to m/s
   v1 = (deltaenc1)*KWHEELS; v2 = (deltaenc2)*KWHEELS; v3 = (deltaenc3)*KWHEELS;
   // Compute velocity, normal velocity and angular velocity
   v  = (-v1+v3)/(2*SINT);
   vn = (v1-2*v2+v3)/(2*(COST+1));
   w  = (v1+2*COST*v2+v3)/(2*DROB*(COST+1));
   halfTeta = normalizeAngleRad(w*DELTA_T_2)-msg->imu_value*DEGTORAD;

   // Acumulate odometry estimate while not used by other functions
   odometry.x += (sin(halfTeta)*v+cos(halfTeta)*vn)*DELTA_T;
   odometry.y += (cos(halfTeta)*v-sin(halfTeta)*vn)*DELTA_T;
   odometry.angle -= normalizeAngleRad(w*DELTA_T)*RADTODEG;

   last_hardware_state = current_hardware_state;
}

// Math Utilities
float Localization::normalizeAngleRad(float angle)
{
   while (angle >  M_PI) angle -= M_PI;
   while (angle < -M_PI) angle += M_PI;
   return angle;
}

float Localization::normalizeAngleDeg(float angle)
{
   while (angle >  360.0) angle -= 360.0;
   while (angle < 0.0) angle += 360.0;
   return angle;
}

void Localization::fuseOrientationEstimates()
{
  // PREDICTION PHASE (Using physical data -> Odometry)
  // X' = A*X + B*u (where B*u is the data from the odometry)
  kalman.predictedState.z = last_state.robot_pose.z + odometry.angle;
  kalman.predictedState.z = normalizeAngleDeg(kalman.predictedState.z);
  if(vision.angle < 90.0 && kalman.predictedState.z > 270.0) kalman.predictedState.z -= 360.0;
  if(vision.angle > 270.0 && kalman.predictedState.z < 90.0) kalman.predictedState.z += 360.0;

  // P' = A*Pant*A' + Q
  kalman.predictedCovariance.z = kalman.lastCovariance.z + kalman.Q.z;

  // CORRECTION PHASE
  //K = P'/(P'+R)
  kalman.K.z = kalman.predictedCovariance.z/(kalman.predictedCovariance.z+kalman.R.z);

  //K = P'/(P'+R)
  // óptima (ŷ) = Previsão + (Ganho Kalman) * (Medição - Previsão)
  current_state.robot_pose.z = kalman.predictedState.z + kalman.K.z*(vision.angle - kalman.predictedState.z);
  current_state.robot_pose.z = normalizeAngleDeg(current_state.robot_pose.z);

  //P = (1-K)*P'
  // Variância da estimativa = Variância da previsão * (1 – Ganho do Kalman)
  kalman.covariance.z = (1-kalman.K.z)*kalman.predictedCovariance.z;
}

// Kalman filter
void Localization::fuseEstimates()
{
   //Fuses Vision(local) with last_state.robot_pose(local)+odometry
   vision.angle = current_hardware_state.imu_value;

   // PREDICTION PHASE (Using physical data -> Odometry)
   // X' = A*X + B*u (where B*u is the data from the odometry)
   kalman.predictedState.x = last_state.robot_pose.x + odometry.x;
   kalman.predictedState.y = last_state.robot_pose.y + odometry.y;
   /*kalman.predictedState.z = last_state.robot_pose.z + odometry.angle;
   kalman.predictedState.z = normalizeAngleDeg(kalman.predictedState.z);
   if(vision.angle < 90.0 && kalman.predictedState.z > 270.0) kalman.predictedState.z -= 360.0;
   if(vision.angle > 270.0 && kalman.predictedState.z < 90.0) kalman.predictedState.z += 360.0;*/

   // P' = A*Pant*A' + Q
   kalman.predictedCovariance.x = kalman.lastCovariance.x + kalman.Q.x;
   kalman.predictedCovariance.y = kalman.lastCovariance.y + kalman.Q.y;
   //kalman.predictedCovariance.z = kalman.lastCovariance.z + kalman.Q.z;

   // CORRECTION PHASE
   //K = P'/(P'+R)
   kalman.K.x = kalman.predictedCovariance.x/(kalman.predictedCovariance.x+kalman.R.x);
   kalman.K.y = kalman.predictedCovariance.y/(kalman.predictedCovariance.y+kalman.R.y);
   //kalman.K.z = kalman.predictedCovariance.z/(kalman.predictedCovariance.z+kalman.R.z);

   //K = P'/(P'+R)
   // óptima (ŷ) = Previsão + (Ganho Kalman) * (Medição - Previsão)
   current_state.robot_pose.x = kalman.predictedState.x + kalman.K.x*(vision.x - kalman.predictedState.x);
   current_state.robot_pose.y = kalman.predictedState.y + kalman.K.y*(vision.y - kalman.predictedState.y);
   /*current_state.robot_pose.z = kalman.predictedState.z + kalman.K.z*(vision.angle - kalman.predictedState.z);
   current_state.robot_pose.z = normalizeAngleDeg(current_state.robot_pose.z);*/

   //P = (1-K)*P'
   // Variância da estimativa = Variância da previsão * (1 – Ganho do Kalman)
   kalman.covariance.x = (1-kalman.K.x)*kalman.predictedCovariance.x;
   kalman.covariance.y = (1-kalman.K.y)*kalman.predictedCovariance.y;
   //kalman.covariance.z = (1-kalman.K.z)*kalman.predictedCovariance.z;
}

void Localization::computeVelocities()
{
   int it_limit = 4;
   static int iteration = it_limit;

   if(iteration<(it_limit-1)) iteration++;
   else {
      iteration = 0;
      float lpf_weight = 0.9;
      float lpf_minor = 1-lpf_weight;
      // First, compute the robot velocities based on final localization estimate
      // ########################################################################
      // Time interval between estimates is requiredTiming = 33ms/30Hz
      current_state.robot_velocity.x = lpf_weight*((current_state.robot_pose.x-last_vel_state.robot_pose.x)/(time_interval*(float)it_limit))+lpf_minor*last_vel_state.robot_velocity.x;
      current_state.robot_velocity.y = lpf_weight*((current_state.robot_pose.y-last_vel_state.robot_pose.y)/(time_interval*(float)it_limit))+lpf_minor*last_vel_state.robot_velocity.y;
      current_state.robot_velocity.w = lpf_weight*((current_state.robot_pose.z-last_vel_state.robot_pose.z)/(time_interval*(float)it_limit))+lpf_minor*last_vel_state.robot_velocity.z;
      // ########################################################################
      if(current_state.robot_velocity.x>2.5) current_state.robot_velocity.x = 0;
      if(current_state.robot_velocity.y>2.5) current_state.robot_velocity.y = 0;
      last_vel_state = current_state;
   }

}


void Localization::decideBallPossession()
{
   // Use both hardware information and vision information to know
   // if the robot possesses the ball or not
   // ########################################################################
   current_state.has_ball = current_hardware_state.ball_sensor;
}

void Localization::generateDebugData()
{
   static int info_counter = 0;
   static unsigned int last_counter;
   Point2d robot_world_pos = Point2d(current_state.robot_pose.x,
                                     current_state.robot_pose.y);
   if(info_counter<33 && generate_extended_debug){
      float max_distance = processor->getMaxDistance();
      current_state.interest_points.clear();
      interestPoint temp;
      Point2d point,mirror_map;
      temp.type = 0;


      for(unsigned int i = 0; i<processor->mappedLinePoints.size();i++){
         temp.pos.x = processor->mappedLinePoints[i].x+robot_world_pos.x;
         temp.pos.y = processor->mappedLinePoints[i].y+robot_world_pos.y;
         current_state.interest_points.push_back(temp);
      }

      temp.type = 1;
      for(unsigned int i = 0; i<processor->mappedBallPoints.size();i++){
          temp.pos.x = point.x+robot_world_pos.x;
          temp.pos.y = point.y+robot_world_pos.y;
          current_state.interest_points.push_back(temp);
      }

      temp.type = 2;
      for(unsigned int i = 0; i<processor->mappedObstaclePoints.size();i++){
            temp.pos.x = processor->mappedObstaclePoints[i].x+robot_world_pos.x;
            temp.pos.y = processor->mappedObstaclePoints[i].y+robot_world_pos.y;
            current_state.interest_points.push_back(temp);
      }
      info_counter++;
   }

   if(info_counter>=33 && generate_extended_debug){
      info_counter = 0;
      if(service_call_counter<=last_counter){
         generate_extended_debug = false;
      }
      last_counter = service_call_counter;
   }

   if(!generate_extended_debug) current_state.interest_points.clear();
}

void Localization::calcHistOrientation()
{
  int horHist[HIST_ANG+1][(int)HIST_SIZE+1] = {0};
  int verHist[HIST_ANG+1][(int)HIST_SIZE+1] = {0};
  int horizMax = 0, vertMax = 0, sumMax = 0;
  int indice=0, sum = 0;
  int px = 0, py = 0, index = 0;
  int startAngle = last_state.robot_pose.z, histEstimate = 0;
  for(int i = -(HIST_ANG/2); i <= (HIST_ANG/2); i++){
    index = i+(HIST_ANG/2);
    rotatePoints((int)i+startAngle);
    horizMax = vertMax = 0;
    for(unsigned int j = 0; j < rotatedLinePoints.size(); j++){
      //ROS_INFO("X: %.2f  Y: %.2f",rotatedLinePoints[j].x, rotatedLinePoints[j].y);
      px = (HIST_SIZE/2) + ((HIST_SIZE/2)/(processor->getMaxDistance()))*rotatedLinePoints[j].x;
      py = (HIST_SIZE/2) + ((HIST_SIZE/2)/(processor->getMaxDistance()))*rotatedLinePoints[j].y;
      verHist[index][py]++;
      horHist[index][px]++;
      if(verHist[index][py]>vertMax) vertMax=verHist[index][py];
      if(horHist[index][px]>horizMax) horizMax=horHist[index][py];
    }
    sum = horizMax + vertMax;
    if(sum>sumMax){
      sumMax = sum;
      indice = i;
    }
  }
  histEstimate = indice + startAngle;
  //ROS_INFO("angulo: %d",histEstimate);
  if(current_hardware_state.imu_value < 90.0 && histEstimate > 270.0) histEstimate -= 360.0;
  if(current_hardware_state.imu_value > 270.0 && histEstimate < 90.0) histEstimate += 360.0;
  current_hardware_state.imu_value = 0.0 * current_hardware_state.imu_value + 1.0 * histEstimate;

  /*delete[] horHist;
  delete[] verHist;*/
}

void Localization::rotatePoints(int angle)
{
  rotatedLinePoints.clear();
  float max_distance = processor->getMaxDistance();
  double pointRelX, pointRelY, ang;
  Point2d lut_mapped;
  for(unsigned int i=0;i<processor->linePoints.size();i++){
     lut_mapped = processor->worldMapping(Point(processor->linePoints[i].x,processor->linePoints[i].y));
     if(lut_mapped.x>RROBOT && lut_mapped.x<=max_distance){
        // Always mapped in relation to (0,0)
        pointRelX = lut_mapped.x*cos((lut_mapped.y)*DEGTORAD);
        pointRelY = lut_mapped.x*sin((lut_mapped.y)*DEGTORAD);
        ang = angle*DEGTORAD;
        rotatedLinePoints.push_back(Point2d(cos(ang)*pointRelX-sin(ang)*pointRelY,sin(ang)*pointRelX+cos(ang)*pointRelY));
    }
  }
}

bool Localization::computeGlobalLocalization(int side) //compute initial localization globally
{
   //First, get the detected line points and map them using the whole field positions.
   //Then, compute the error for every position and find the least error position

   unsigned int inboundPoints = 0;
   double positionError = 0.0, leastError = 100000.0;
   double xCor = 0.0, yCor = 0.0;
   Point2d wp,leastPos, point;
   int Lenght, Width;
   unsigned int i, j;

   // Tamanho da matriz
   if(side == 0){
     i = float(currentField.FIELD_LENGTH/currentField.SCALE)+1;
     i = i/2;
     j = float(currentField.FIELD_WIDTH/currentField.SCALE)+1;
     j = j/2;
     Lenght = float(currentField.FIELD_LENGTH/currentField.SCALE)+1;
     Width = float(currentField.FIELD_WIDTH/currentField.SCALE)+1;
   }
   else {
     i = 0; j = 0;
     Lenght = float(currentField.FIELD_LENGTH/currentField.SCALE)+1;
     Lenght = Lenght/2;
     Width = float(currentField.FIELD_WIDTH/currentField.SCALE)+1;
     Width = Width/2;
   }


   for(i = 0; i < Lenght; i++){
     for(j = 0; j < Width; j++){
       positionError = 0.0;
       inboundPoints = 0;

       // coordenadas em metros
       wp.x = WorldMap[i][j].x;
       wp.y = WorldMap[i][j].y;

       for(unsigned int p = 0; p < processor->mappedLinePoints.size(); p++){
         point.x = processor->mappedLinePoints[p].x;
         point.y = processor->mappedLinePoints[p].y;
         if(isInbounds(point,wp)){
           inboundPoints++;
           xCor = ceil(((point.x+wp.x)*10)-0.49)/10; // calcula aproximação mais rigorasa ao do ponto ao mapa
           yCor = ceil(((point.y+wp.y)*10)-0.49)/10;
           positionError += errorFunction(WorldMap[getFieldIndexX(xCor)][getFieldIndexY(yCor)].closestDistance, processor->mappedLinePoints[p].z);
         }
       }
       if((positionError>0)&&(inboundPoints>=processor->mappedLinePoints.size()*0.9)){ //if all linePoints are inbounds, enquire the error
          if(positionError<leastError){
             leastError = positionError;
             leastPos.x = WorldMap[i][j].x;
             leastPos.y = WorldMap[i][j].y;
          }
       }
     }
   }

   xCor = 0;
   yCor = 0;
   float localGap = 0.5;
   for(float x=leastPos.x-localGap; x<=leastPos.x+localGap; x+=0.1){
      for(float y=leastPos.y-localGap; y<=leastPos.y+localGap; y+=0.1){
            positionError = 0.0;
            wp.x = x;
            wp.y = y;
            inboundPoints = 0;
            for(unsigned int p=0;p<processor->mappedLinePoints.size();p++){ //Counts inbound points and compute error
              point.x = processor->mappedLinePoints[p].x;
              point.y = processor->mappedLinePoints[p].y;
               if(isInbounds(point,wp)){
                  inboundPoints++;
                  xCor = ceil(((point.x+wp.x)*10)-0.49)/10; // calcula aproximação mais rigorasa ao do ponto ao mapa
                  yCor = ceil(((point.y+wp.y)*10)-0.49)/10;
                  positionError += errorFunction(WorldMap[getFieldIndexX(xCor)][getFieldIndexY(yCor)].closestDistance, processor->mappedLinePoints[p].z);
               }
             }
            if((positionError>0) && (inboundPoints>=processor->mappedLinePoints.size()*0.9)){
              if(positionError<leastError){
                leastError = positionError;
                leastPos.x = x;
                leastPos.y = y;
          }
        }
      }
    }
    vision.x = leastPos.x;
    vision.y = leastPos.y;
    vision.angle = current_state.robot_pose.z;
    current_state.robot_pose.x = leastPos.x;
    current_state.robot_pose.y = leastPos.y;
    return false;
}

void Localization::computeLocalLocalization() //compute next localization locally
{
   unsigned int inboundPoints = 0;
   double positionError = 0.0, leastError = 100000.0;
   double xCor = 0.0, yCor = 0.0;
   float localGap = 0.4;
   Point2d wp, leastPos, point;

   for(float x=last_state.robot_pose.x-localGap; x<=last_state.robot_pose.x+localGap; x+=0.1)
      for(float y=last_state.robot_pose.y-localGap; y<=last_state.robot_pose.y+localGap; y+=0.1){
         positionError = 0.0;
         inboundPoints = 0;
         wp.x = x;
         wp.y = y;
         for(unsigned int p=0;p<processor->mappedLinePoints.size();p++){ //Counts inbound points and compute error
           point.x = processor->mappedLinePoints[p].x;
           point.y = processor->mappedLinePoints[p].y;
            if(isInbounds(point,wp)){
               inboundPoints++;
               xCor = ceil(((point.x+wp.x)*10)-0.49)/10; // calcula aproximação mais rigorasa ao do ponto ao mapa
               yCor = ceil(((point.y+wp.y)*10)-0.49)/10;
               positionError += errorFunction(WorldMap[getFieldIndexX(xCor)][getFieldIndexY(yCor)].closestDistance,processor->mappedLinePoints[p].z );
            }
         }
      if((positionError>0)&&(inboundPoints>=processor->mappedLinePoints.size()*0.9)){ //if all linePoints are inbounds, enquire the error
         if(positionError<leastError){
            leastError = positionError;
            leastPos.x = x;
            leastPos.y = y;
         }
      }
   }
   vision.x = leastPos.x;
   vision.y = leastPos.y;
   vision.angle = vision.angle;
}



int Localization::getLine(float x, float y) //Returns matching map position
{
   int line = 0;
   line = (currentField.TERM1+x*10)*(currentField.TERM2+1)-1+(currentField.TERM3+y*10)+1;
   return line;
}

float Localization::errorFunction(float error, double weight)
{
  double err = 1-((0.5*0.5)/((0.5*0.5)+(error*error)));
  return weight * err;
}

bool Localization::isInbounds(Point2d point,Point2d center)
{
   double x = point.x+center.x;
   double y = point.y+center.y;

   if(((x>=-(currentField.HALF_FIELD_LENGTH))&&(x<=(currentField.HALF_FIELD_LENGTH)))
           && ((y>=-(currentField.HALF_FIELD_WIDTH))&&(y<=(currentField.HALF_FIELD_WIDTH)))) return true;
   else return false;
}

int Localization::getFieldIndexX(float value)
{
   int tempX = float((value+currentField.HALF_FIELD_LENGTH)/currentField.SCALE);
   return tempX;
}

int Localization::getFieldIndexY(float value)
{
   int tempX = float((value+currentField.HALF_FIELD_WIDTH)/currentField.SCALE);
   return tempX;
}

/*
Point2d Localization::mapPointToRobot(double orientation, Point2d dist_lut)
{
   // Always mapped in relation to (0,0)
   double pointRelX = dist_lut.x*cos((dist_lut.y)*DEGTORAD);
   double pointRelY = dist_lut.x*sin((dist_lut.y)*DEGTORAD);
   double ang = orientation*DEGTORAD;

   return Point2d(cos(ang)*pointRelX-sin(ang)*pointRelY,
                  sin(ang)*pointRelX+cos(ang)*pointRelY);
}

void Localization::mapLinePoints(int robot_heading)
{
   rotatedLinePoints.clear();
   float max_distance = processor->getMaxDistance();
   Point2d lut_mapped;
   for(unsigned int i=0;i<processor->linePoints.size();i++){
      lut_mapped = processor->worldMapping(Point(processor->linePoints[i].x,processor->linePoints[i].y));
      if(lut_mapped.x>0 && lut_mapped.x<=max_distance)
         rotatedLinePoints.push_back(mapPointToRobot(robot_heading,lut_mapped));//
   }
}
*/

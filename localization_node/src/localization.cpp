#include "localization.h"

Localization::Localization(ros::NodeHandle *par , bool *init_success, bool use_camera, QObject *parent) : QObject(parent)
{
   initVariables();
   
   //#### Initialize major components ####
   //##################################### 
   bool correct_initialization = true;
   processor = new ImageProcessor(use_camera,&correct_initialization); // true -> use camera
   if(!correct_initialization) { (*init_success) = false; return; }
   confserver = new ConfigServer(par); 
   
   confserver->setOmniVisionConf(processor->getMirrorConfAsMsg(),processor->getVisionConfAsMsg(),processor->getImageConfAsMsg());
   parentTimer = new QTimer();
   requiredTiming = 33; //ms
   time_interval = (float)requiredTiming/1000.0;
   connect(parentTimer,SIGNAL(timeout()),this,SLOT(discoverWorldModel()));
   //##################################### 
   
   //## Setup connections to confserver ##
   //##################################### 
   connect(confserver,SIGNAL(stopImageAssigning()),this,SLOT(stopImageAssigning()));
   connect(confserver,SIGNAL(changedImageRequest(uint8_t)),this,SLOT(changeImageAssigning(uint8_t)));
   connect(confserver,SIGNAL(changedMirrorConfiguration(mirrorConfig::ConstPtr)),this,SLOT(changeMirrorConfiguration(mirrorConfig::ConstPtr)));
   connect(confserver,SIGNAL(changedLutConfiguration(visionHSVConfig::ConstPtr)),this,SLOT(changeLookUpTableConfiguration(visionHSVConfig::ConstPtr)));
   connect(confserver,SIGNAL(changedImageConfiguration(imageConfig::ConstPtr)),this,SLOT(changeImageConfiguration(imageConfig::ConstPtr)));
   //#####################################
   
   //## Setup ROS Pubs and Subs ##
   //############################# 
   //Initialize robotInfo publisher
   robot_info_pub = par->advertise<robotInfo>("robotInfo", 1);
	//Initialize hardwareInfo subscriber
	ros::Subscriber hardware_info_sub = par->subscribe("hardwareInfo", 
                                                   1, 
                                                   &Localization::hardwareCallback,
                                                   this);
   reloc_service = par->advertiseService("requestReloc",
                                    &Localization::doReloc,
                                    this);
   //############################# 
   //TEST
   buffer = imread(QString(imgFolderPath+"temp.png").toStdString());
   parentTimer->start(33);
}

Localization::~Localization()
{

}

void Localization::discoverWorldModel() // Main Function
{
   bool have_image = true;
   int timing = 0;
   parentTimer->stop();
   QTime measure;
   measure.start();
   
   // Localization code
   if(have_image){
      //Publish information   
      fuseEstimates();
      computeVelocities();
      decideBallPossession();
      memset(&odometry,0,sizeof(localizationEstimate));
		robot_info_pub.publish(current_state);
		last_state = current_state;
   }  
   
   if(!have_image) parentTimer->start(1);
   else {
      //Send image to confserver if requested
      if(assigning_images){
         if(assigning_type==IMG_RAW) {
            confserver->assignImage(&buffer);  
         } else {
            processed = buffer.clone();
            processor->getSegmentedImage(&processed);  
            confserver->assignImage(&processed);     
         }
      }
   
      //Compensate time took in the function
      timing = requiredTiming-measure.elapsed();
      if(timing<0) timing = requiredTiming;
      parentTimer->start(timing);
   }
   
}
   
void Localization::initVariables()
{
   qRegisterMetaType<uint8_t>("uint8_t");
   qRegisterMetaType<mirrorConfig::ConstPtr>("mirrorConfig::ConstPtr");
   qRegisterMetaType<visionHSVConfig::ConstPtr>("visionHSVConfig::ConstPtr");
   qRegisterMetaType<imageConfig::ConstPtr>("imageConfig::ConstPtr");
   QString home = QString::fromStdString(getenv("HOME"));
   QString cfgDir = home+QString(configFolderPath);
   imgFolderPath = cfgDir+QString(imageFolderPath);
   
   assigning_images = false;  
   is_hardware_ready = false; 
   initializeKalmanFilter();
}

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
   //TODO:Stop processing mechanism
   processor->updateLabelLutConf(FIELD,msg->field);
   processor->updateLabelLutConf(LINE,msg->line);
   processor->updateLabelLutConf(BALL,msg->ball);
   processor->updateLabelLutConf(OBSTACLE,msg->obstacle);
   processor->generateLookUpTable();
   if(processor->writeLookUpTable())ROS_INFO("New %s saved!",lutFileName);
   //TODO:Start processing mechanism
}
void Localization::changeMirrorConfiguration(mirrorConfig::ConstPtr msg)
{
   ROS_WARN("New configuration of mirror will be set.");
   //TODO:Stop processing mechanism
   processor->updateDists(msg->max_distance,msg->step,msg->pixel_distances);
   processor->generateMirrorConfiguration();
   if(processor->writeMirrorConfig())ROS_INFO("New %s saved!",mirrorFileName);
   //TODO:Start processing mechanism
}

void Localization::changeImageConfiguration(imageConfig::ConstPtr msg)
{
   ROS_WARN("New configuration of image will be set.");
   //TODO:Stop processing mechanism
   processor->setCenter(msg->center_x,msg->center_y,msg->tilt);
   processor->generateMirrorConfiguration();
   if(processor->writeImageConfig())ROS_INFO("New %s saved!",imageFileName);
   //TODO:Start processing mechanism
}

bool Localization::doReloc(requestReloc::Request &req,requestReloc::Response &res)
{
   //############################# 
   current_state.robot_pose.x = 0;
   current_state.robot_pose.y = 0;
   current_state.robot_velocity.x = 0;
   current_state.robot_velocity.y = 0;
   current_state.robot_velocity.w = 0;
   memset(&odometry,0,sizeof(localizationEstimate));
   memset(&vision,0,sizeof(localizationEstimate));
   last_state.robot_pose = current_state.robot_pose;
   last_state.robot_velocity = current_state.robot_velocity;
   //############################# 
   // TODO: Add reloc/global localization flag
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
   v1 = (deltaenc1)*KWheels; v2 = (deltaenc2)*KWheels; v3 = (deltaenc3)*KWheels;
   // Compute velocity, normal velocity and angular velocity
   v  = (-v1+v3)/(2*sint);
   vn = (v1-2*v2+v3)/(2*(cost+1));
   w  = (v1+2*cost*v2+v3)/(2*dRob*(cost+1));
   halfTeta = normalizeAngleRad(w*deltaT_2)-msg->imu_value*degToRad;
   
   // Acumulate odometry estimate while not used by other functions
   odometry.x += (sin(halfTeta)*v+cos(halfTeta)*vn)*deltaT;
   odometry.y += (cos(halfTeta)*v-sin(halfTeta)*vn)*deltaT;
   odometry.angle -= normalizeAngleRad(w*deltaT)*radToDeg;
   
   last_hardware_state = current_hardware_state;
}

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

void Localization::fuseEstimates()
{
   //Fuses Vision(local) with last_state.robot_pose(local)+odometry
  
   // TODO: Temporary, to enable hardware-only localization
   // ###############################################
   vision.x += odometry.x; vision.y += odometry.y;
   // ###############################################
   
   vision.angle = current_hardware_state.imu_value;
   // PREDICTION PHASE (Using physical data -> Odometry)
   // X' = A*X + B*u (where B*u is the data from the odometry)
   kalman.predictedState.x = last_state.robot_pose.x + odometry.x;
   kalman.predictedState.y = last_state.robot_pose.y + odometry.y;
   kalman.predictedState.z = last_state.robot_pose.z + odometry.angle;
   kalman.predictedState.z = normalizeAngleDeg(kalman.predictedState.z);
   if(vision.angle < 90.0 && kalman.predictedState.z > 270.0) kalman.predictedState.z -= 360.0; 
   if(vision.angle > 270.0 && kalman.predictedState.z < 90.0) kalman.predictedState.z += 360.0; 
	
   // P' = A*Pant*A' + Q
   kalman.predictedCovariance.x = kalman.lastCovariance.x + kalman.Q.x;
   kalman.predictedCovariance.y = kalman.lastCovariance.y + kalman.Q.y;
   kalman.predictedCovariance.z = kalman.lastCovariance.z + kalman.Q.z;

   // CORRECTION PHASE
   //K = P'/(P'+R)  
   kalman.K.x = kalman.predictedCovariance.x/(kalman.predictedCovariance.x+kalman.R.x);
   kalman.K.y = kalman.predictedCovariance.y/(kalman.predictedCovariance.y+kalman.R.y);
   kalman.K.z = kalman.predictedCovariance.z/(kalman.predictedCovariance.z+kalman.R.z);

   //K = P'/(P'+R)
   current_state.robot_pose.x = kalman.predictedState.x + kalman.K.x*(vision.x - kalman.predictedState.x);
   current_state.robot_pose.y = kalman.predictedState.y + kalman.K.y*(vision.y - kalman.predictedState.y);
   current_state.robot_pose.z = kalman.predictedState.z + kalman.K.z*(vision.angle - kalman.predictedState.z);
   current_state.robot_pose.z = normalizeAngleDeg(current_state.robot_pose.z);

   //P = (1-K)*P'
   kalman.covariance.x = (1-kalman.K.x)*kalman.predictedCovariance.x;
   kalman.covariance.y = (1-kalman.K.y)*kalman.predictedCovariance.y;
   kalman.covariance.z = (1-kalman.K.z)*kalman.predictedCovariance.z;
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

void Localization::initializeKalmanFilter()
{
   memset(&odometry,0,sizeof(localizationEstimate));
   memset(&vision,0,sizeof(localizationEstimate));
   kalman.Q.x = kalman.Q.y = 1; kalman.Q.z = 1;
   kalman.R.x = kalman.R.y = 20; kalman.R.z = 30;
   kalman.lastCovariance.x = kalman.lastCovariance.y = kalman.lastCovariance.z = 0;
   kalman.covariance = kalman.predictedCovariance = kalman.lastCovariance;
}



#include "localization.h"

Localization::Localization(ros::NodeHandle *par , bool *init_success, bool use_camera, QObject *parent) : QObject(parent)
{
   initVariables();
   if(!use_camera) is_hardware_ready = true; // for test purposes
   //#### Initialize major components ####
   //##################################### 
   bool correct_initialization = true;
   processor = new ImageProcessor(use_camera,&correct_initialization); // true -> use camera
   if(!correct_initialization) { (*init_success) = false; return; }
   confserver = new ConfigServer(par); 
   
   correct_initialization = initWorldMap();
   if(!correct_initialization) { (*init_success) = false; return; }
   
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
   parentTimer->stop();
   QTime measure;
   measure.start();
   // Acquire image using function pointer
   buffer = CALL_MEMBER_FN((*processor),processor->acquireImage)(&have_image);
   current_hardware_state.imu_value = 8;
   // Localization code
   if(have_image && is_hardware_ready){
      processor->detectInterestPoints();   
      
      if(reloc){ // Global localization
         // Assign hardware angle to vision estimate angle
         current_state.robot_pose.z = current_hardware_state.imu_value;
         reloc = false;
         // Do global localization
         computeGlobalLocalization(0);
      } else { // Local localization
         // Do local localization   
         computeLocalLocalization ();
         fuseEstimates();
      }
      //Publish information 
      computeVelocities();
      decideBallPossession();
      generateDebugData();
      memset(&odometry,0,sizeof(localizationEstimate));
		robot_info_pub.publish(current_state);
		last_state = current_state;
   }  
   
   if(!have_image) parentTimer->start(1);
   else {
      //Send image to confserver if requested
      if(assigning_images){
         if(assigning_type&IMG_RAW) {
            confserver->assignImage(buffer);  
         } else if(assigning_type&IMG_SEG) {
            processed = buffer->clone();
            processor->getSegmentedImage(&processed);  
            confserver->assignImage(&processed);     
         } else if(assigning_type&IMG_INT) {
            processed = buffer->clone();
            processor->drawInterestInfo(&processed);  
            confserver->assignImage(&processed); 
         } else if(assigning_type&IMG_WRL) {
            processed = buffer->clone();
            processor->drawWorldInfo(&processed);  
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
   QString cfgDir = home+QString(CONFIGFOLDERPATH);
   
   assigning_images = false;  
   is_hardware_ready = false; 
   generate_extended_debug = false;
   service_call_counter = 0;
   initializeKalmanFilter();
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
   currentField.HALF_FIELD_LENGTH = currentField.FIELD_LENGTH/2.0;
   currentField.HALF_FIELD_WIDTH = currentField.FIELD_WIDTH/2.0;
   currentField.TERM1 = (currentField.FIELD_LENGTH*10)/2;
   currentField.TERM2 = (currentField.FIELD_WIDTH*10);
   currentField.TERM3 = (currentField.FIELD_WIDTH*10)/2;

   worldMap = (struct nodo*)calloc(currentField.FIELD_POSITIONS,sizeof(struct nodo));
   memset(worldMap,0,sizeof(nodo)*currentField.FIELD_POSITIONS);
   QStringList values,positions;
   QString auxPos,auxVal;

   for(unsigned int i=0;i<currentField.FIELD_POSITIONS;i++){
       values = in.readLine().split(")");
       auxVal = values.at(1);
       auxVal.replace(",",".");
       auxPos = values.at(0);
       auxPos.remove(0,1);
       auxPos.replace(",",".");
       positions = auxPos.split(";");
       worldMap[i].x = positions.at(0).toDouble();
       worldMap[i].y = positions.at(1).toDouble();
       worldMap[i].closestDistance = auxVal.toDouble();
   }
   file.close();
   return true;
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
   processor->updateDists(msg->max_distance,msg->step,msg->pixel_distances);
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
      for(unsigned int i = 0; i<mappedLinePoints.size();i++){   
         temp.pos.x = mappedLinePoints[i].x+robot_world_pos.x;
         temp.pos.y = mappedLinePoints[i].y+robot_world_pos.y; 
         current_state.interest_points.push_back(temp);                  
      }
      
      temp.type = 1;
      for(unsigned int i = 0; i<processor->ballPoints.size();i++){
         mirror_map = processor->worldMapping(processor->ballPoints[i]);
         if(mirror_map.x<=max_distance){
            point = mapPointToRobot(current_state.robot_pose.z,mirror_map);
            temp.pos.x = point.x+robot_world_pos.x;
            temp.pos.y = point.y+robot_world_pos.y; 
            current_state.interest_points.push_back(temp);   
         }                     
      }
      
      temp.type = 2;
      for(unsigned int i = 0; i<processor->obstaclePoints.size();i++){
         mirror_map = processor->worldMapping(processor->obstaclePoints[i]);
         if(mirror_map.x<=max_distance){
            point = mapPointToRobot(current_state.robot_pose.z,mirror_map);
            temp.pos.x = point.x+robot_world_pos.x;
            temp.pos.y = point.y+robot_world_pos.y; 
            current_state.interest_points.push_back(temp);   
         }                       
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

Point2d Localization::mapPointToRobot(double orientation, Point2d dist_lut)
{
   // Always mapped in relation to (0,0)
   double pointRelX = dist_lut.x*cos((dist_lut.y)*DEGTORAD);
   double pointRelY = dist_lut.x*sin((dist_lut.y)*DEGTORAD);
   double ang = orientation*DEGTORAD;
   
   return Point2d(cos(ang)*pointRelX-sin(ang)*pointRelY,
                  sin(ang)*pointRelX+cos(ang)*pointRelY);
}

void Localization::computeGlobalLocalization(int side) //compute initial localization globally
{
   //First, get the detected line points and map them using the whole field positions.
   //Then, compute the error for every position and find the least error position

   unsigned int inboundPoints = 0, leastIndex = 0;
   double positionError = 0.0, leastError = 100000.0;
   double xCor = 0.0, yCor = 0.0;
   mapLinePoints(current_state.robot_pose.z); // Maps line points to the current position
    for(unsigned int i=0;i<currentField.FIELD_POSITIONS;i++){
      positionError = 0.0;
      inboundPoints = 0;
      Point2d wp; wp.x = worldMap[i].x; wp.y = worldMap[i].y;
     for(unsigned int p=0;p<mappedLinePoints.size();p++){ //Counts inbound points and compute error
         if(isInbounds(mappedLinePoints[p],wp)){
            inboundPoints++;
            xCor = ceil(((mappedLinePoints[p].x+worldMap[i].x)*10)-0.49)/10;
            yCor = ceil(((mappedLinePoints[p].y+worldMap[i].y)*10)-0.49)/10;
            positionError += errorFunction(worldMap[getLine(xCor,yCor)].closestDistance);
         }
      }

      if((positionError>0)&&(inboundPoints>=mappedLinePoints.size()*0.9)){ //if all linePoints are inbounds, enquire the error
         if(positionError<leastError){
            leastError = positionError;
            leastIndex = i;
         }
      }
   }

   //Assign Initial Guesses of the Kalman Filter
   vision.x = worldMap[leastIndex].x;
   vision.y = worldMap[leastIndex].y;
   vision.angle = current_state.robot_pose.z;
   current_state.robot_pose.x = worldMap[leastIndex].x;
   current_state.robot_pose.y = worldMap[leastIndex].y;
}

void Localization::computeLocalLocalization() //compute next localization locally
{
   unsigned int inboundPoints = 0; Point2f leastPos;
   double positionError = 0.0, leastError = 100000.0;
   double xCor = 0.0, yCor = 0.0;
   float localGap = 0.5;
   mapLinePoints(current_hardware_state.imu_value);// Maps line points to the current positiongap
   for(float x=last_state.robot_pose.x-localGap; x<=last_state.robot_pose.x+localGap; x+=0.1)
      for(float y=last_state.robot_pose.y-localGap; y<=last_state.robot_pose.y+localGap; y+=0.1){
         positionError = 0.0;
         inboundPoints = 0;
         Point2d wp; wp.x = x; wp.y = y;
         for(unsigned int p=0;p<mappedLinePoints.size();p++){ //Counts inbound points and compute error
            if(isInbounds(mappedLinePoints[p],wp)){
               inboundPoints++;
               xCor = ceil(((mappedLinePoints[p].x+wp.x)*10)-0.49)/10;
               yCor = ceil(((mappedLinePoints[p].y+wp.y)*10)-0.49)/10;
               positionError += errorFunction(worldMap[getLine(xCor,yCor)].closestDistance);
            }
         }

      if((positionError>0)&&(inboundPoints>=mappedLinePoints.size()*0.9)){ //if all linePoints are inbounds, enquire the error
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

void Localization::mapLinePoints(int robot_heading)
{
   mappedLinePoints.clear();
   float max_distance = processor->getMaxDistance();
   Point2d lut_mapped;
   for(unsigned int i=0;i<processor->linePoints.size();i++){   
      lut_mapped = processor->worldMapping(Point(processor->linePoints[i].x,processor->linePoints[i].y));
      if(lut_mapped.x>0 && lut_mapped.x<=max_distance)
         mappedLinePoints.push_back(mapPointToRobot(current_state.robot_pose.z,lut_mapped));
   }
}

int Localization::getLine(float x, float y) //Returns matching map position
{
   int line = 0;
   line = (currentField.TERM1+x*10)*(currentField.TERM2+1)-1+(currentField.TERM3+y*10)+1;
   return line;
}

float Localization::errorFunction(float error)
{
   return 1-((0.3*0.3)/((0.3*0.3)+error));
}

bool Localization::isInbounds(Point2d point,Point2d center)
{
   double x = point.x+center.x;
   double y = point.y+center.y;

   if(((x>=-currentField.HALF_FIELD_LENGTH)&&(x<=currentField.HALF_FIELD_LENGTH))
           && ((y>=-currentField.HALF_FIELD_WIDTH)&&(y<=currentField.HALF_FIELD_WIDTH))) return true;
   else return false;
}

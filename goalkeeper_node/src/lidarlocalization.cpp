#include "lidarlocalization.h"

// CONSTRUCTOR
lidarLocalization::lidarLocalization()
{
    if(!configFilePaths()){
        ROS_ERROR("Failed to read world map.");
    }
    worldPoints.clear();
    readyHardware = false;
    memset(&odometry,0,sizeof(struct localizationEstimate));
    memset(&lidar,0,sizeof(struct localizationEstimate));
    lastPose = pose = Point3d(5.1,0.0,90);
    initField(fieldPath);
    readMapConfFile(mapPath);
    initKalmanFilter();
    
    // Mudar para ficheiro
    halfIsRight = true;
    tipPositionRight = Point2d(3.55,-2.2);
    tipPositionLeft = Point2d(-3.55,+2.2);
    lastPoseM = velocities = Point3d(0,0,0);
}
// **********************************
// INITIALIZATION FUNCTIONS
void lidarLocalization::initField(QString file_)
{
    // Create view of current Field
    QFile file(file_);
    if(!file.open(QIODevice::ReadOnly)) {
        ROS_ERROR("Error reading %s",file_.toStdString().c_str());
        exit(6);
    }
    QTextStream in(&file);

    QString value; 
    int counter = 0;
    while(!in.atEnd()){
       value = in.readLine();
       fieldAnatomy.dimensions[counter] = value.right(value.size()-value.indexOf('=')-1).toInt();
       counter++;
    }
}

void lidarLocalization::readMapConfFile(QString file_)
{
    QFile file(file_);
    if(!file.open(QIODevice::ReadOnly)) {
        ROS_ERROR("Error reading %s",file_.toStdString().c_str());
       	exit(10);
    } QTextStream in(&file);

    int xCounter = 0;
    map.clear();
    vector<struct nodo> temp; temp.clear();
    struct nodo dummy;
    QString res = in.readLine();
    outputResolution = res.toInt();
    outputResolution_m = outputResolution/1000.0;
    ceilaux = outputResolution_m/2;
    while(!in.atEnd()){
        QString values = in.readLine();
        dummy = parseString(values);
        if(xCounter>0&&dummy.x==0){
            map.push_back(temp);
            temp.clear();
            xCounter = 0;
        }
        temp.push_back(dummy);
        xCounter++;
    } map.push_back(temp);

    xMax = (double)map[0][map[0].size()-1].x;
    yMax = (double)map[map.size()-1][0].y;
    xTransform = (map[0].size()-1)/xMax;
    yTransform = (map.size()-1)/yMax;
    file.close();
}

bool lidarLocalization::configFilePaths()
{
    QString home = QString::fromStdString(getenv("HOME"));
    QString commonDir = home+QString(COMMON_PATH);
    QString cfgDir = commonDir+QString(KIN_CFG_PATH);
    QString fieldsDir = commonDir+QString(FIELDS_PATH);
    QString mainFile = commonDir+QString(MAINFILENAME);

    QFile file(mainFile);
    if(!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QTextStream in(&file);

    QString field_ = in.readLine();

    fieldPath = fieldsDir+field_+".view";
    mapPath = fieldsDir+QString("G")+field_+".map";
    file.close();
    return true;
}

nodo lidarLocalization::parseString(QString str)
{
    struct nodo dummy;
    memset(&dummy,0,sizeof(struct nodo));
    QString left = str.left(str.indexOf(":")-1);
    left = left.right(left.size()-1);
    QStringList coordinates = left.split(",");
    QString right = str.right(str.length()-str.indexOf(":")-1);
    dummy.x = coordinates[0].toDouble();
    dummy.y = coordinates[1].toDouble();
    dummy.closestDistance = right.toDouble();
    return dummy;
}
// LOCALIZATION FUNCTIONS
void lidarLocalization::updateOdometryEstimate(const hardwareInfo::ConstPtr &msg)
{
   // Process hardware information
   static int receivedFrames = 0;
   current_hardware_state = *msg;

   if(!readyHardware){
   		receivedFrames++;
   		if(receivedFrames>10) readyHardware = true;
   		else { 
   			pose.z = msg->imu_value;
   			poseM.z = msg->imu_value;
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
   halfTeta = NormalizeAngle(w*DELTA_T_2)-msg->imu_value*DEGTORAD;

   // Update for Kalman Filtering
   double deltaX = (sin(halfTeta)*v+cos(halfTeta)*vn)*DELTA_T;
   double deltaY = (cos(halfTeta)*v-sin(halfTeta)*vn)*DELTA_T;
   double deltaTheta = NormalizeAngle(w*DELTA_T)*(180.0/M_PI);

   odometry.y += deltaY; odometry.x += deltaX; odometry.angle -= deltaTheta;

   // Mean Time Update for interpolation
   lidar.angle = msg->imu_value;
   poseM.x += deltaX; poseM.y += deltaY; poseM.z -= deltaTheta;
   
   last_hardware_state = current_hardware_state;
}

void lidarLocalization::updateLidarEstimate(vector<float> *distances)
{
	static float LastLidarAng = 0;
	static bool doGlobalLocalization = true;

	if(readyHardware){
		//map detected points to world position (0,0,angle)
		mapDetectedPoints(distances);
		if(doGlobalLocalization){
			//Global Localization
			doGlobalLocalization = false;
			calculateGlobalOptimum();
		} else {
			//Local Localization
			if(worldPoints.size()>100){//if was less than 100 point from the lidar, doen't do the kalman filter
				if(int(lidar.angle)==297){
					if(int(lidar.angle)==int(LastLidarAng)){
						calculateBestLocalPose(0.25);
						fuseEstimates();  
					}
				}
				else{				
					calculateBestLocalPose(0.25);
					fuseEstimates();
				}	
			}
		}
		
		if(worldPoints.size()>100){
			memset(&odometry,0,sizeof(struct localizationEstimate));
			assertPoseToGlobalPosition();  		
		} else{
			pose.x +=odometry.x;
			pose.y +=odometry.y;
			pose.z +=odometry.angle;
			lastPose = pose;
			memset(&odometry,0,sizeof(struct localizationEstimate));	
		}
	    	computeVelocities();
	}
	
	LastLidarAng = lidar.angle;
}

void lidarLocalization::fuseEstimates()
{
	//Fuses lidar(local) with lastPose(local)+odometry
	//Simple assignment
	
	// PREDICTION PHASE
    // X' = A*X + B*u (where B*u is the data from the odometry)
    kalman.predictedState.x = lastPose.x + odometry.x;
    kalman.predictedState.y = lastPose.y + odometry.y;
    kalman.predictedState.z = lastPose.z + odometry.angle;
    kalman.predictedState.z = normalizeAngle(kalman.predictedState.z);

    if (lidar.angle < 90.0 && kalman.predictedState.z > 270.0) kalman.predictedState.z = kalman.predictedState.z - 360.0;
    if (lidar.angle > 270.0 && kalman.predictedState.z < 90.0) kalman.predictedState.z = kalman.predictedState.z + 360.0;
    
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
    pose.x = kalman.predictedState.x + kalman.K.x*(lidar.x - kalman.predictedState.x);
    pose.y = kalman.predictedState.y + kalman.K.y*(lidar.y - kalman.predictedState.y);
    pose.z = kalman.predictedState.z + kalman.K.z*(lidar.angle - kalman.predictedState.z);
    pose.z = normalizeAngle(pose.z);
   
	//P = (1-K)*P'
	kalman.covariance.x = (1-kalman.K.x)*kalman.predictedCovariance.x;
    kalman.covariance.y = (1-kalman.K.y)*kalman.predictedCovariance.y;
    kalman.covariance.z = (1-kalman.K.z)*kalman.predictedCovariance.z;
	lastPose = pose;

}
    
double lidarLocalization::getDistance(double x, double y) // X AND Y HAS TO BE DOUBLE, OTHERWISE ERRORS WILL HAPPEN
{
    double correctedX = round(ceil(x/outputResolution_m))*outputResolution_m;
    double correctedY = round(ceil(y/outputResolution_m))*outputResolution_m;
    int idx = correctedX*xTransform;int idy = correctedY*yTransform;
    return map[idy][idx].closestDistance;
}

void lidarLocalization::calculateGlobalOptimum()
{
    double leastError = 10000000.0,error = 0.0;
    for(double y=0.0;y<yMax;y+=0.05){
        for(double x=0.0;x<=xMax;x+=0.05){
            error = positionError(x,y,pose.z);
            if(error<leastError){
                leastError = error;
                lidar.x = x;
                lidar.y = y;
            }
        }
    }

    meanError = leastError/worldPoints.size();
    pose.x = lidar.x;
    pose.y = lidar.y;
    pose.z = lidar.angle;
    lastPose = pose;
}

void lidarLocalization::calculateBestLocalPose(double radius)
{
    double leastError = 10000000.0,error = 0.0;
    for(double y=lastPose.y-radius;y<=lastPose.y+radius;y+=0.05){
        for(double x=lastPose.x-radius;x<=lastPose.x+radius;x+=0.05){
            error = positionError(x,y,pose.z);
            if(error<leastError){
                leastError = error;
                lidar.x = x;
                lidar.y = y;
            }
        }
    }

    meanError = leastError/worldPoints.size();
}

double lidarLocalization::positionError(double rx, double ry, double angle)
{
    double error = 0;
    for(unsigned int i=0;i<worldPoints.size();i++){
        double px = worldPoints[i].x + rx;
        double py = worldPoints[i].y + ry;
        if(px>=0&&px<=xMax&&py>=0&&py<=yMax)
            error += errorFunction(getDistance(px,py));
        else error += errorFunction(5);
    }

    return error;
}

double lidarLocalization::errorFunction(double error)
{
    return 1-((0.25*0.25)/((0.25*0.25)+error));
}
// **********************************
// GETTER FUNCTIONS
Point3d lidarLocalization::getPose()
{
    return poseM;
}

Point3d lidarLocalization::getVelocities()
{
	return velocities;
}
    
vector<Point2f> lidarLocalization::getWorldPoints()
{
	return detectedPoints; 
}
// **********************************
// MATHEMATIC OPERATIONS
Point lidarLocalization::world2WorldModel(Point2d pos)
{
    Point converted = Point(0,0);
    double totalLength = fieldAnatomy.fieldDims.TOTAL_LENGTH*fieldAnatomy.fieldDims.FACTOR/100.0;
    double totalWidth = fieldAnatomy.fieldDims.TOTAL_WIDTH*fieldAnatomy.fieldDims.FACTOR/100.0;

    converted.x = pos.x*(fieldAnatomy.fieldDims.TOTAL_LENGTH/totalLength)+fieldAnatomy.fieldDims.TOTAL_LENGTH/2+
            fieldAnatomy.fieldDims.LINE_WIDTH/2;
    converted.y = pos.y*(fieldAnatomy.fieldDims.TOTAL_WIDTH/totalWidth)+fieldAnatomy.fieldDims.TOTAL_WIDTH/2+
            fieldAnatomy.fieldDims.LINE_WIDTH/2;
    return converted;
}

Point2d lidarLocalization::mapPointToWorld(double rx, double ry, double angle, float dist, double theta, double offset)
{
    // theta must be in radians
    // rx,ry,dist,offset must be in meters
    // angle must be in degrees
    double pointRelX = -dist*cos(theta);
    double pointRelY = dist*sin(theta);
    double ang = angle*(M_PI/180.0);
    double angOffset = ang;
    return Point2d(rx-cos(ang)*pointRelX-sin(ang)*pointRelY+(offset*sin((angOffset))),
                   ry-sin(ang)*pointRelX+cos(ang)*pointRelY-(offset*cos((angOffset))));
}

void lidarLocalization::mapDetectedPoints(vector<float> *distances)
{
//LAST ADDED
//if(normalizeAngle(pose.z)<216 || normalizeAngle(pose.z)>324)//não faz o ciclo se virado de costas para a baliza
{
    worldPoints.clear();
    float maxDistance = 2.35;
    double offset = 0.20;//distance between the center of the robot and the center of the lidar
    double theta = -M_PI-(M_PI/4.0);
    double res = 0.36*(M_PI/180.0);
    float cutleftangle=0,cutrightangle=0;
    //filter and map points
	float mylittlepos = ((normalizeAngle(pose.z)-90));//diversion from the front of the robot (goal in the back)
	if(mylittlepos>180){ 
	    mylittlepos -=  360;
	}
	
	if (mylittlepos<0){
	    mylittlepos = abs(mylittlepos);
	    mylittlepos = mylittlepos/0.36;
	     cutleftangle = 125 + mylittlepos;
	     cutrightangle = 125 - mylittlepos;
    }else{
	    mylittlepos /=0.36;
	     cutleftangle = 125 - mylittlepos; 
	    cutrightangle = 125 + mylittlepos;
    }//125 corresponds to 45º in the lidar, since the lidar has 270º of view, 
    //if 45º is subtracted to each side, the end is 180º


	if(cutleftangle<97){//97 is equivalent to 10 degrees in the left side of the robot  and is the maximum that can be read without the interferency of the robot structure
		cutleftangle = 97;
		if(cutrightangle>551)//sabe as the left side but for 190 degrees (the 0 is the left side of the goal if the robot is on top of his line, and the right side is the 180º)
			cutrightangle = 653;
}
	if(cutrightangle<97){//sabe as the left side but for 190 degrees (the 0 is the left side of the goal if the robot is on top of his line, and the right side is the 180º)
		cutrightangle = 97;
		if(cutleftangle>551)//97 is equivalent to 10 degrees in the left side of the robot and is the maximum that can be read without the interferency of the robot structure
			cutleftangle = 653;
}

    for(unsigned int i = cutleftangle; i<distances->size()-cutrightangle;i++){
        if(distances->at(i)<=maxDistance && distances->at(i)>=0.25){//inside the 0.25cm radius from the lidar, all points are ignored
            worldPoints.push_back(mapPointToWorld(0,0,poseM.z,distances->at(i),theta+i*res,offset));
        }
    }

	/*printf("**%f **%f **%f  **%f Es %f Dr %f\n", cutleftangle,cutrightangle,mylittlepos,normalizeAngle(pose.z), cutleftangle*0.36, cutrightangle*0.36);*/
}
}

float lidarLocalization::NormalizeAngle(float angulo)
{
    float angle = angulo;
    while (angulo >  M_PI) angulo -= M_PI;
    while (angulo < -M_PI) angulo += M_PI;
    angle = angulo;
    return angle;
}

void lidarLocalization::assertPoseToGlobalPosition()
{
	poseM = pose;
	if(halfIsRight){
		poseM.x = tipPositionRight.x + pose.x;
		poseM.y = tipPositionRight.y + pose.y;
	} else {
		poseM.x = tipPositionRight.x - pose.x;
		poseM.y = tipPositionRight.y + pose.y;
	}
}

// Initialization of the Kalman Filter
void lidarLocalization::initKalmanFilter()
{
    /*kalman.Q.x = kalman.Q.y = 1; kalman.Q.z = 10;
    kalman.R.x = kalman.R.y = 25; kalman.R.z = 0.5;*/

    kalman.Q.x = kalman.Q.y = 15; kalman.Q.z = 1;
    kalman.R.x = kalman.R.y = 1; kalman.R.z = 25;
    kalman.lastCovariance.x = kalman.lastCovariance.y = kalman.lastCovariance.z = 0 ;
    kalman.covariance = kalman.predictedCovariance = kalman.lastCovariance;
}

double lidarLocalization::normalizeAngle(double angulo)
{
    double angle = angulo;
    while (angulo >  360.0) angulo -= 360.0;
    while (angulo < 0.0) angulo += 360.0;
    return angulo;
    angle = angulo;
    return angle;
}

int lidarLocalization::getQuadrant(double angle)
{	
	if(angle>=0.0 && angle<90.0) return 1;
	if(angle>=90.0 && angle<180.0) return 2;
	if(angle>=180.0 && angle<270.0) return 3;
	if(angle>=270.0 && angle<=360.0) return 4;
}
    
void lidarLocalization::computeVelocities()
{
	velocities.x = (poseM.x-lastPoseM.x)/0.1;
	velocities.y = (poseM.y-lastPoseM.y)/0.1;
	if((getQuadrant(lastPoseM.z)==1&&getQuadrant(poseM.z)==4)){
		velocities.z = -(((0-lastPoseM.z)+(poseM.z-360))*(M_PI/180.0))/0.1;		
	} else if((getQuadrant(lastPoseM.z)==4&&getQuadrant(poseM.z)==1)){
		velocities.z = (((360-lastPoseM.z)+poseM.z)*(M_PI/180.0))/0.1;
	} else {
		velocities.z = ((poseM.z-lastPoseM.z)*(M_PI/180.0))/0.1;
	}
	lastPoseM = poseM;
}
// **********************************


#include "kinectvision.h"

//if(temp.h==0) temp.h = 180;

bool goodCandidateFound = false; // variable that tell us when we found the ball
int save_mind=0;   // variable that tell us the point where the ball is
int finalcounter=0;   // count how many time passed since we lose da ball  ( from 11 frames to 20 frames) 
   
kinectVision::kinectVision(ros::NodeHandle *par)
{
    parent = par;
	configFilePaths();
	initVariables();
	localization = new lidarLocalization();
	if(!readParameters() || !localization->readParameters()) exit(0);
    generateLookUpTable(false);
	_index = 0;
}

double kinectVision::calculate_margin(int RunningMargin)
{

    double margin=0;
    margin= 640- refinedCandidates[save_mind][0] - RunningMargin;

    if (margin < (refinedCandidates[save_mind][0] - RunningMargin)){
        margin = refinedCandidates[save_mind][0] - RunningMargin ;
    }

    if (margin < (refinedCandidates[save_mind][1]-RunningMargin)){
        margin = refinedCandidates[save_mind][1]-RunningMargin;
    }
    
    if (margin < (480 - refinedCandidates[save_mind][1]- RunningMargin)){
        margin = 480 - refinedCandidates[save_mind][1];
    }
    margin /= 10;

    return margin;
}

double kinectVision::checkLimits( int mode , double RunningMargin){  // function to make sure that 0<nrows<480   and 0<ncols<640

    switch (mode) {
    case 1:
        if ((refinedCandidates[save_mind][1] - RunningMargin)<0){
            return 0; 
        }else{
            return (refinedCandidates[save_mind][1] - RunningMargin);
        }
        break;
    case 2:
        if ((refinedCandidates[save_mind][1] + RunningMargin)>480){
            return 480;
        }else {
            return (refinedCandidates[save_mind][1] + RunningMargin);
        }
        break;
    case 3:
        if ((refinedCandidates[save_mind][0] - RunningMargin)<0){
            return 0;
        } else {
            return (refinedCandidates[save_mind][0] - RunningMargin);
        }
        break;
    case 4:
        if ((refinedCandidates[save_mind][0] + RunningMargin)>640){
            return 640;
        } else {
            return (refinedCandidates[save_mind][0] + RunningMargin);
        }
        break;
    default: break;
    }

}


double kinectVision::updateLimits (int mode=1, int minId=0){//sabendo que a bola tem tamanho real x então podemos utilizar isso para definir quantas vezes maior será o pedaço a seguir, por exemplo:
//a bola mede m metros e p pixeis, a uma velocidade v ela anda k*x metros em 30 milisegundos, logo anda k*p pixeis da camara,
//logo para seguir a bola a margem deverá ser de 1,5*k*p pixeis, sendo que k*x neste caso representa a v maxima que a bola poderá atingir (deve incluir uma pequena margem)

/*

ObjSizePix - tamanho da bola em pixeis
RunningMargin - margem em pixeis calculada para uma velocidade x a multiplicar por uma margem de y, vai ser o tamanho da janela que se deve pesquisar em x
*/	//static bool found_the_ball=false;
	static int lastballPositionY=0, counter=0,RunningMargin=640;
	double ObjSizePix=0;
	static double Margin = 480 ;
	
	if (!mode) {
		save_mind=minId;
		//found_the_ball=ball_found;
		return 0;
	} else{
		
        if(!goodCandidateFound) {
            if (counter == 20){
                counter=20;
            } else {
                counter++;
            }
            if (counter == 11 ) Margin=calculate_margin(RunningMargin);
        } else{
            counter=0;
            //refinedCandidates[minID][2]; /raio em pixeis da bola que em cm é aproximadamente 11 cm 

            if(ballPosition3D.y != lastballPositionY){

                ObjSizePix = 2 * refinedCandidates[save_mind][2];//Get the size of the object in pixels,by multiplying the radius by two
                RunningMargin = 0.35*ObjSizePix/0.222;//By using the ball diameter (22.2cm) and the max velocity margin (40cm for 48km/h(13,3 m/s) with a margin) we get the velocity margin in pixels


            }	
        }
        
        lastballPositionY=ballPosition3D.y;

        finalcounter=counter-10; // after 10 frames of 30 ms the margin 
        if (finalcounter<0) 
        finalcounter=0; 
        return RunningMargin+(Margin*finalcounter);	// valor em pixeis
	}
	
	return 640;

}

void kinectVision::setRobotPose(Point3d pose)
{
	robotPose = pose;
}

Point2d kinectVision::mapPointToWorld(double rx, double ry, double angle, float dist, double theta, double offset)
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

void kinectVision::detectGameBall()
{
	static int fpsmeasure = 0,fpscounter = 0;
	QTime measure;
	retrieveTimer->stop();
	measure.start();
	int timing = 0;
	if(getImages(depthImage,rgbImage)){
		// Process Image
		filterByColor(true);
		filterByAnatomy(true);
		chooseBestCandidate(true);
		
		//Display Information
		fpsmeasure+= fpsReader.elapsed();
		fpscounter++;
		if(fpscounter==200) {
			ROS_INFO("Average FPS : %.1fFPS",(1000.0*fpscounter)/fpsmeasure);
			fpsmeasure=fpscounter=0;
		}
		fpsReader.start();
		timing = definedRatems-measure.elapsed();
        if(timing<=0) timing = 1;
        retrieveTimer->start(timing);
	} else {
		//ROS_INFO("Failure Getting Images");
		retrieveTimer->start(5);
	}
}


void kinectVision::filterByColor(bool show)
{
    candidates.clear();
    clear.copyTo(binary);
    Vec3b* pixel;
    uchar *pixelB;
    long int index = 0;
    double RunningMargin=640,teste=0;	
    static bool start = true;	
    	
	if(start){
		
		for(int i = 0; i < nRows; ++i){
        		pixel = rgbImage.ptr<Vec3b>(i);
        		pixelB = binary.ptr<uchar>(i);
        		for (int j = 0; j<nCols; ++j){
           			 index = (pixel[j][2]<<16) + (pixel[j][1]<<8) + (pixel[j][0]);
           			 if ( YUVLookUpTable[index] == UAV_ORANGE_BIT )  pixelB[j] = 255;//se é bola
        			 if(goodCandidateFound)		
					start= false;		     
						     }
    					      }
			
		}
	
	else{
		RunningMargin = updateLimits(1);


		for(int i = (checkLimits(1,RunningMargin)) ; i < (checkLimits( 2,RunningMargin)) ; ++i){
			pixel = rgbImage.ptr<Vec3b>(i);
        		pixelB = binary.ptr<uchar>(i);
			for (int j = (checkLimits(3,RunningMargin)); j< (checkLimits( 4,RunningMargin)) ; ++j){
				index = (pixel[j][2]<<16) + (pixel[j][1]<<8) + (pixel[j][0]);
				if ( YUVLookUpTable[index] == UAV_ORANGE_BIT )  pixelB[j] = 255;//se é bola
        		}
    		}
		if(show) rectangle(rgbImage, Point (checkLimits( 4,RunningMargin),checkLimits( 2,RunningMargin)), Point (checkLimits(3,RunningMargin),checkLimits(1,RunningMargin)), Scalar( 255, 255, 255 ));
		// copy rgbImage to toSendRgbImage if requested
		
		
	}	

    morphologyEx(binary,binary,MORPH_DILATE, elementdil);
    morphologyEx(binary,binary,MORPH_ERODE, elementero);
    
    if(show)imshow("ColorSegmentation",binary);
    // copy binary to toSendBinary if requested
    
    vector<Mat> contours; Moments moment;
    Point2f center; float radius;
    findContours(binary, contours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    for (unsigned int i = 0; i < contours.size(); i++){
        if (contourArea(contours[i])>300  && contourArea(contours[i])<30000 ){ // tweak contour area
            moment = moments(contours[i], true);
            double area = moment.m00;
            double x = moment.m10 / area; // calculate the center of mass
            double y = moment.m01 / area;

            Rect rBound = boundingRect(contours[i]); // calculate min enclosing circle and rectangle
            minEnclosingCircle((Mat)contours[i], center, radius);

            double wh = fabs(1 - ((double)rBound.width / rBound.height)); // calculate circle ratios
            double pi = fabs(1 - (fabs(area)) / (CV_PI * pow(radius, 2)));
            if (wh<=0.91 - (0.001 * finalcounter) && pi<=0.81 - (0.01 * finalcounter) ){   // errrrrrrrooooo
                candidates.push_back(Point3i(x,y,radius));
            }
            
         }
    }
}

void kinectVision::filterByAnatomy(bool show)
{
	vector<Vec4i> filteredCandidates;
    filteredCandidates.clear();	
    int x = 0, y = 0, radius = 0; double z = 0.0;
    uchar *pixelB;	
    
    for(unsigned int i=0;i<candidates.size();i++){
		x = candidates[i].x; y = candidates[i].y; radius = candidates[i].z;
		pixelB = depthImage.ptr<uchar>(y);
		z = pixelB[x];
		if((z>0 && z<245)) {
		    filteredCandidates.push_back(Vec4i(x,y,radius,z));
		}
    }
    
    uchar *candidate; refinedCandidates.clear();
    Mat roi; int dim = 0; int range = 0;
    for(unsigned int c=0;c<filteredCandidates.size();c++){
        x = filteredCandidates[c][0]; y = filteredCandidates[c][1]; dim = filteredCandidates[c][2]+range;
        z = filteredCandidates[c][3];
        pixelB = depthImage.ptr<uchar>(y);
        z = pixelB[x];
        roi = Mat(dim*2,dim*2,CV_8UC1,Scalar(0));

        for(int i=-dim;i<=dim/1.2;i++){
            pixelB = depthImage.ptr<uchar>(i+y);
            candidate = roi.ptr<uchar>(i+dim);
            for(int j=-dim;j<=dim;j++){
                if(pixelB[j+x] != 0 && pixelB[j+x] != 255){
                    if(isInRange(z,pixelB[j+x])){
                        candidate[j+dim] = 255;
                    }
                }
            }
        }

        //Verify if object
        morphologyEx(roi,roi,MORPH_DILATE, elementdil);
        morphologyEx(roi,roi,MORPH_ERODE, elementero);
        //imshow("ROI",roi);
        vector<Mat> contours; Moments moment; Point2f center; float radius;
        findContours(roi, contours,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

        for (unsigned int i = 0; i < contours.size(); i++){
            if (contourArea(contours[i])>300   && contourArea(contours[i])<30000){ // tweak contour area
                moment = moments(contours[i], true);
                double area = moment.m00;
                double _x = moment.m10 / area; // calculate the center of mass
                double _y = moment.m01 / area;

                Rect rBound = boundingRect(contours[i]); // calculate min enclosing circle and rectangle
                minEnclosingCircle((Mat)contours[i], center, radius);

                double wh = fabs(1 - ((double)rBound.width / rBound.height)); // calculate circle ratios
                double pi = fabs(1 - (fabs(area)) / (CV_PI * pow(radius, 2)));
                if (wh<=0.7 - (0.02*finalcounter) && pi<=0.7- (0.02*finalcounter)  ){   ///// erro proporcional to the size of the image
                    refinedCandidates.push_back(Vec6d((x-dim)+_x,(y-dim)+_y,radius,z,area,0.0));
                }
             }
        }

    }
    
    if(show) { // remove later
    	/*for(unsigned int i=0;i<filteredCandidates.size();i++){
			//circle(rgbImage,Point(filteredCandidates[i][0],filteredCandidates[i][1]),filteredCandidates[i][2],
			Scalar(255,0,0),2);
		}
		for(unsigned int i=0;i<refinedCandidates.size();i++){
			//circle(rgbImage,Point(refinedCandidates[i][0],refinedCandidates[i][1]),refinedCandidates[i][2],
			//Scalar(0,255,0),3);
		}*/
		imshow("Candidate",rgbImage); 
    }
    
}
	
void kinectVision::chooseBestCandidate(bool show)
{
    double devArea = 0.0, devRads = 0.0; //Deviations
    double mz = 0; double minSum = 10.0; int minID = 0; double minZ = 0.0;
    double tilt = 1.5; double xCand = 0.0, yCand = 0.0; double proximityRatio = 0.0;
    //bool goodCandidateFound = false;
	goodCandidateFound =false;
    //get Z, morphological z operation, choose best
    for(unsigned int cand=0;cand<refinedCandidates.size();cand++){
        mz = getZ(refinedCandidates[cand][3]); // Real Z in meters
        devArea = abs(1-(refinedCandidates[cand][4]/getStdArea(mz)));
        devRads = abs(1-(refinedCandidates[cand][2]/getStdRads(mz)));
		
		if(ballPosition3D.z>=0){
			xCand = mz+0.125+0.1;		
			yCand = tan(((refinedCandidates[cand][0]-320)*convHorizontal)*(M_PI/180.0))*mz;
			proximityRatio = sqrt((xCand-ballPosition3D.x)*(xCand-ballPosition3D.x)+(yCand-ballPosition3D.y)*(yCand-ballPosition3D.y))/1.0;
		} else proximityRatio = 0.0;

        if((devRads+devArea) < minSum && (devRads+devArea)<= 0.7 - (0.01 * finalcounter) && proximityRatio<=1.7 - (0.02 * finalcounter)) { 
// se estiver entre a area e o raio da bola goodCandidateFound ;
        	goodCandidateFound = true;

            minSum = (devRads+devArea);
            minID = cand;
            minZ = mz;
	    updateLimits(0,minID);  ///////// update inID
	   	
		
        }
    }
	/* AQUIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII o candidatoooooo*/ 
	if(refinedCandidates.size()>0 && goodCandidateFound){
		ballPosition3D.x = minZ+0.125+0.1;
		ballPosition3D.y = tan(((refinedCandidates[minID][0]-320)*convHorizontal)*(M_PI/180.0))*minZ;
		ballPosition3D.z = 0.52+minZ*tan((M_PI/180.0)*(6-(refinedCandidates[minID][1]-240)*convVertical))+0.11;	
		
		// map to world
		Point2d aux = mapPointToWorld(robotPose.x,robotPose.y,robotPose.z,sqrt((ballPosition3D.x*ballPosition3D.x)+
		(ballPosition3D.y*ballPosition3D.y)),atan2(ballPosition3D.y,ballPosition3D.x)+M_PI_2,0);
		
		ballPosWorld.x = aux.x; ballPosWorld.y = aux.y; ballPosWorld.z = ballPosition3D.z;
		// Calculate elapsed distance and elapsed time
		double elapsedDist = sqrt((lastBallPosition3D.x-ballPosition3D.x)*(lastBallPosition3D.x-ballPosition3D.x)
			+(lastBallPosition3D.y-ballPosition3D.y)*(lastBallPosition3D.y-ballPosition3D.y));
		double deltaTime = timerVel.elapsed()/1000.0;
		
		if((elapsedDist/deltaTime)<=7.0){ // 7m/s top speed
			//Assign initial velocities
			ballVelocities = Point3d((ballPosWorld.x-lastBallPosWorld.x)/deltaTime,(ballPosWorld.y-lastBallPosWorld.y)/deltaTime
				,(ballPosWorld.z-lastBallPosWorld.z)/deltaTime);
			//Predict impact zone
			predictBallPosition();
		} else {
			ballVelocities = Point3d(0,0,0);
			ballImpactZone = Point3d(0,0,-1);	
		}

		//Assign to last
		lastBallPosition3D = ballPosition3D;
		lastBallPosWorld = ballPosWorld;
		timerVel.start();
				
	} else ballPosition3D = Point3d(0,0,-1);
	
	/// draw on toSendRgbImage
	if(refinedCandidates.size()>0 && goodCandidateFound && show){///////////////////////// valoooooooooreeessss xyeraio
		circle(rgbImage,Point(refinedCandidates[minID][0],refinedCandidates[minID][1]),
		refinedCandidates[minID][2],Scalar(255,0,255),2);
	}
 	if(show) imshow("Candidate",rgbImage);   
}

bool kinectVision::getImages(Mat &depth_,Mat&rgb_)
{
	void *depth,*rgb;
	unsigned int _timestamp;
	if(freenect_sync_get_depth(&depth,&_timestamp,_index,FREENECT_DEPTH_REGISTERED)==0
	 && freenect_sync_get_video(&rgb,&_timestamp,_index,FREENECT_VIDEO_RGB)==0){
	 	//Retrieve video Image
	 	Mat rgbMat(Size(640,480),CV_8UC3,Scalar(0));
        uint8_t* rgb_data = static_cast<uint8_t*>(rgb);
        rgbMat.data = rgb_data;
        cvtColor(rgbMat, rgb_, CV_RGB2BGR);
	 	//Retrieve depth Image
	 	Mat depthMat(Size(640,480),CV_16UC1);
	 	uint16_t* depth_data = static_cast<uint16_t*>(depth);
	 	depthMat.data = (uchar *)depth_data;
	 	depthMat.convertTo(depth_, CV_8UC1, 255.0/4096.0);
	 }	
}

void kinectVision::configFilePaths()
{
    QString home = getenv("HOME");
    configFolderPath = home+QString(COMMON_PATH);
    configFolderPath += QString(KIN_CFG_PATH);
	lutFile = configFolderPath+QString(KINLUTFILENAME);
	paramFile = configFolderPath+QString(KINPARAMSFILENAME);
}

void kinectVision::generateLookUpTable()
{
 	if(start){
 		QFile file_(lutFile);
		if(!file_.open(QIODevice::WriteOnly)) {
		    ROS_ERROR("Error writing to %s",KINLUTFILENAME);
		    return;
		} QTextStream in_(&file_);
		in_ << QString("HRANGES="+QString::number(LUTRanges[0])+","+QString::number(LUTRanges[1])+"\n");
		in_ << QString("SRANGES="+QString::number(LUTRanges[2])+","+QString::number(LUTRanges[3])+"\n");
		in_ << QString("VRANGES="+QString::number(LUTRanges[4])+","+QString::number(LUTRanges[5])+"\n");
		file_.close();
	}
    //Generates in memory look up table
	int y,u,v;
    rgb pix; hsv pix2;
    unsigned int index;
    for (int r=0; r<256; r++) // classify every RGB color into our LUT
        for (int g=0; g<256; g++)
            for (int b=0; b<256; b++)
            {
                pix.r = r; pix.g = g; pix.b = b;
                pix2 = rgbtohsv(pix);
                y = pix2.h;
                u = pix2.s;
                v = pix2.v;
                index = (r<<16)+(g<<8)+b;

                //-- initialize on update --
                YUVLookUpTable[index] = UAV_NOCOLORS_BIT;
                //-- Reference Colour range -UAV_GREEN_BIT-
                if (((y>=LUTRanges[0]) && (y<=LUTRanges[1])) && ((u>=LUTRanges[2]) && (u<=LUTRanges[3])) &&
                          ((v>=LUTRanges[4]) && (v<=LUTRanges[5]))){
                    YUVLookUpTable[index] = UAV_ORANGE_BIT;
                }
            }

}

void kinectVision::initVariables()
{
	double morph_size_dil = 0.5;
	double morph_size_erode = 2.0;
	
	Mat rgbImage(Size(640,480),CV_8UC3,Scalar(0));
	nRows = 480; nCols = 640;
	
	definedRatems = 33;
    
    convHorizontal = (57/2.0)/320.0; 
    convVertical = (43/2.0)/240.0;
    
    elementdil = getStructuringElement(2, Size( 2*morph_size_dil + 1, 2*morph_size_dil+1 ), Point( morph_size_dil, morph_size_dil ) );
    elementero = getStructuringElement(2, Size( 2*morph_size_erode + 1, 2*morph_size_erode+1 ), Point( morph_size_erode, morph_size_erode ) );
    
    clear = Mat (480,640,CV_8UC1,Scalar(0));
    
	ballImpactZone = Point3d(0,0,-1);
    ballVelocities = Point3d(0,0,0);
    ballPosition3D = Point3d(0,0,-1);
}
   
bool kinectVision::isInRange(int z, int val)
{
    int range = 7;
    if(abs(val-z)<range) {
        return true;
    }
    else return false;
}
 
bool kinectVision::readParameters()
{
    heightFromGround = 0.7;
    QFile file(paramFile);
    if(!file.open(QIODevice::ReadOnly)) {
        ROS_ERROR("Error reading %s",KINPARAMFILENAME);
        return false;
    } QTextStream in(&file);
	
	//Init height from ground
    QString height = in.readLine();
    heightFromGround = height.right(height.size()-height.indexOf('=')-1).toDouble();
	
	//Init FOV
    QString fov = in.readLine();
    fov =  fov.right(fov.size()-fov.indexOf('=')-1);
    QStringList fovs = fov.split(",");
    hFov = fovs[0].toDouble();
    vFov = fovs[1].toDouble();
    file.close();
    
    QFile file_(lutFile);
    if(!file_.open(QIODevice::ReadOnly)) {
        ROS_ERROR("Error reading %s",KINLUTFILENAME);
        return false;
    } QTextStream in_(&file_);
    
    memset(LUTRanges,0,6*sizeof(int));
    QString h = in_.readLine();
    QStringList hValues = h.right(h.size()-h.indexOf('=')-1).split(",");
    if(hValues.size()!=2 || h=="") { ROS_ERROR("Error in H %s file",KINLUTFILENAME); exit(0); }
    LUTRanges[0] = hValues[0].toInt(); LUTRanges[1] = hValues[1].toInt();
    QString s = in_.readLine();
    QStringList sValues = s.right(s.size()-s.indexOf('=')-1).split(",");
    if(sValues.size()!=2 || s=="") { ROS_ERROR("Error in S %s file",KINLUTFILENAME); exit(1); }
    LUTRanges[2] = sValues[0].toInt(); LUTRanges[3] = sValues[1].toInt();
    QString v = in_.readLine();
    QStringList vValues = v.right(v.size()-v.indexOf('=')-1).split(",");
    if(vValues.size()!=2 || v=="") { ROS_ERROR("Error in V %s file",KINLUTFILENAME); exit(2); }
    LUTRanges[4] = vValues[0].toInt(); LUTRanges[5] = vValues[1].toInt();
    
    file_.close();
    
    return true;
}

double kinectVision::getZ(int pixVal)
{
    if(pixVal<=28) return 0.5;
    else if(pixVal>=255) return 4.0;
    else {
        return (-0.000000263321223161165*pow(pixVal,3))
                +(0.00009535747359655*pow(pixVal,2))
                +(0.0076242311*pixVal)
                +0.2354778415+0.02;
    }
}

double kinectVision::getStdArea(double mVal)
{
    if(mVal==0.5) return 43000;
    else if(mVal==4) return 650;
    else {
        return (9871.3954736085*pow(mVal,-2.0397879024));
    }
}

double kinectVision::getStdRads(double mVal)
{
    if(mVal==0.5) return 125;
    else if(mVal==4) return 16;
    else {
        return (61.2541340469*pow(mVal,-0.9949180346));
    }
}

// Prediction
bool kinectVision::intrestingEventHappened(double x, double y, double xi)
{
    if(y>=3.0 || y<=-3.0) return true; // ball goes through lateral lines
    else if(x-xi<=-0.3) return true; // 30cm to avoid errors, ball goes towards the other goalkeeper
    else if(x>=4.9 && (y>=0.9 || y<=-0.9)) return true; // shot wide
    else if(x<=5.4 && x>=4.7 && y<=0.9 && y>=-0.9) return true;
    else return false;
}

bool kinectVision::crossGoalLine(double x)
{
    if(x<=5.5 && x>=4.7) return true;
    else return false;
}

void kinectVision::predictBallPosition()
{
    double Xf, Yf, Zf, huehue=0, maxHeight=0, XfAux, YfAux;
    bool potentialGoal=false, lowShot=false;
	double g = 9.8;
	double Xi = ballPosWorld.x, Yi = ballPosWorld.y, Zi = ballPosWorld.z;
	double Vz = ballVelocities.z;
	double e = 0.88;
	int N = 10;
 if(ballVelocities.x<0.9) { ballImpactZone = Point3d(0,0,-1); return;} 
 for(int i=0; i<N; i++)//while
    {
        double t=(-Vz-sqrt((Vz)*(Vz) + 4*ballPosWorld.z*0.5*g))/(2*-0.5*g);

        for(double j=0; j<=t; j+=0.01){
            Xf=Xi + ballVelocities.x*j; //Computes new predicted point
            Yf=Yi + ballVelocities.y*j;
            Zf=Zi+ Vz*j - 0.5*g*(j*j);

            if(maxHeight<Zf){ maxHeight=Zf; XfAux=Xf; YfAux=Yf; } // Assign max height
            if(intrestingEventHappened(Xf,Yf,Xi)){ potentialGoal=true; break;} // Detect events
        }

        if(maxHeight<=0.45) { lowShot=true; break; } // If it is a low shot, predict continuous movement
        maxHeight=0;

        if(!crossGoalLine(Xf) && potentialGoal) { // No solution on prediction and end of pred.
			ballImpactZone = Point3d(0,0,-1);
            break;
        } else if(crossGoalLine(Xf) && potentialGoal) { // Solution on prediction
            ballImpactZone = Point3d(Xf,Yf,Zf);
            break;
        } else { // Continue prediction
            Xi=Xf;
            Yi=Yf;
            Zi=Zf;
            Vz=Vz*e;
        }
     }

    if(lowShot){   
        potentialGoal=false;
        double m=(Yf-YfAux)/(Xf-XfAux);
        double b=Yf-m*Xf;
	double new_y=m*4.7+b;
        if(new_y<=0.9&&new_y>=-0.9){
		 ballImpactZone = Point3d(4.7,new_y,0);
        } else ballImpactZone = Point3d(0,0,-1);
    }
}


// Localization stuff
void kinectVision::updateLidar(vector<float> *distances)
{
	localization->updateLidarEstimate(distances);
}

void kinectVision::updateOdometry(float angle,int enc1,int enc2,int enc3)
{
	localization->updateOdometryEstimate(angle,enc1,enc2,enc3);
}

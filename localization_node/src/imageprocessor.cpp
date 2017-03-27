#include "imageprocessor.h"

//Constructor of the class
ImageProcessor::ImageProcessor(int rob_id, bool use_camera, bool *init_success)
{
    // Initialize GigE Camera Driver
    if(use_camera) {
      camera=true;
      omniCamera = new BlackflyCam(false, rob_id); //OmniVisionCamera Handler
      ROS_INFO("Using GigE Camera for image acquisition.");
      acquireImage = &ImageProcessor::getImage;
    } else {
      camera=false;
      acquireImage = &ImageProcessor::getStaticImage;
      ROS_INFO("Using static image for testing.");
    }
    // Initialize basic robot and field definitions
    if(!initializeBasics(rob_id)){
      ROS_ERROR("Error Reading %s.",MAINFILENAME);
      (*init_success) = false; return;
    } else ROS_INFO("Reading information for %s.",agent.toStdString().c_str());

    // Assign mask image
    mask = imread(maskPath.toStdString());
    if(mask.empty()){
        ROS_ERROR("Error Reading %s.",MASKFILENAME);
        (*init_success) = false; return;
    }else ROS_INFO("%s Ready.", MASKFILENAME);

    // Read color segmentation Look Up Table
    if(!readLookUpTable()){ // Read and Initialize Look Up Table
        memset(&YUVLookUpTable,UAV_NOCOLORS_BIT,LUT_SIZE);
        ROS_ERROR("Error Reading %s.",LUTFILENAME);
        (*init_success) = false; return;
    } else ROS_INFO("%s Ready.",LUTFILENAME);

    // Initialize world mapping parameters
    if(!initWorldMapping()){ // Initialize World Mapping
        ROS_ERROR("Error Reading World mapping configurations");
        (*init_success) = false; return;
    } else ROS_INFO("World mapping configurations Ready.");

    if(!getWorldConfiguration()){
      ROS_ERROR("Error Reading %s.",WORLDFILENAME);
      (*init_success) = false; return;
    }else ROS_INFO("World parameters configuration Ready.");

    variablesInitialization(); // Initialize common Variables
    rleModInitialization(); // Initialize RLE mod data
    // Initialize GigE Camera Image Feed
    if(use_camera){
        if(!startImaging()){
            ROS_ERROR("Error Starting GigE Camera.");
            (*init_success) = false;
            return;
        } else printCameraInfo();
    } else { // Load static image
      QString path = imgFolderPath+QString("2,50.png");
      static_image = imread(path.toStdString().c_str());
      if(static_image.empty()){
         ROS_ERROR("Failed to read static image");
         (*init_success) = false;
         return;
      }
    }
}

// Miscellaneous variables Initialization
// TODO :: Use file for variables initialization
void ImageProcessor::variablesInitialization()
{
    robotHeight = 0.74;
    double morph_size = 1.5;
    element = getStructuringElement(2, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    buffer = new Mat(IMG_SIZE,IMG_SIZE,CV_8UC3,Scalar(0,0,0));
    sizeRelThreshold = 0.65;
    piThreshold = 0.6;
}

// World mapping configuration initialization, returns true if successfully initialized
bool ImageProcessor::initWorldMapping()
{
    // Read mirror parameters
    QFile file(mirrorParamsPath);
    if(!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QTextStream in(&file);

    QString max_distance = in.readLine();
    QString step = in.readLine();
    QString pixel_distances = in.readLine();
    QString line_length = in.readLine();
    max_distance = max_distance.right(max_distance.size()-max_distance.indexOf('=')-1);
    step = step.right(step.size()-step.indexOf('=')-1);
    pixel_distances = pixel_distances.right(pixel_distances.size()-pixel_distances.indexOf('=')-1);
    line_length = line_length.right(line_length.size()-line_length.indexOf('=')-1);


    mirrorConf.max_distance = max_distance.toFloat();
    mirrorConf.step = step.toFloat();
    int expected_args = (int)(mirrorConf.max_distance/mirrorConf.step);
    QStringList mappedDists = pixel_distances.split(",");
    QStringList lineLeng = line_length.split(","); // está a ler apenas,falta fazer o resto

    if(expected_args!=mappedDists.size() || expected_args<=0 || mappedDists.size()<=0 || expected_args!=lineLeng.size() || lineLeng.size()<=0 || mappedDists.size()!=lineLeng.size()) {
       ROS_ERROR("Bad Configuration in %s",MIRRORFILENAME);
       return false;
    }

    mirrorConf.pixel_distances.clear();
    for(float dist=mirrorConf.step;dist<=mirrorConf.max_distance;dist+=mirrorConf.step) distReal.push_back(dist);
    for(int i=0;i<mappedDists.size();i++) {
      distPix.push_back(mappedDists[i].toDouble());
      distPixVal.push_back(lineLeng[i].toInt());
      mirrorConf.pixel_distances.push_back(mappedDists[i].toInt());
      mirrorConf.lines_length.push_back(lineLeng[i].toInt());
    }
    file.close();
    //

    QFile file2(imageParamsPath);
    if(!file2.open(QIODevice::ReadOnly)) {
        return false;
    }
    QTextStream in2(&file2);

    QString image_center = in2.readLine();
    QString tilt = in2.readLine();
    image_center = image_center.right(image_center.size()-image_center.indexOf('=')-1);
    tilt = tilt.right(tilt.size()-tilt.indexOf('=')-1);
    QStringList coords = image_center.split(",");

    if(coords.size()!=2) {
       ROS_ERROR("Bad Configuration (1) in %s",IMAGEFILENAME);
       return false;
    }

    imageConf.center_x = coords.at(0).toInt();
    imageConf.center_y = coords.at(1).toInt();
    imageConf.tilt = tilt.toInt();

    file2.close();

    generateMirrorConfiguration();
    return true;
}

bool ImageProcessor::initializeBasics(int rob_id)
{
    QString home = QString::fromStdString(getenv("HOME"));
    QString commonDir = home+QString(COMMON_PATH);
    QString cfgDir = commonDir+QString(LOC_CFG_PATH);
    QString fieldsDir = commonDir+QString(FIELDS_PATH);
    QString mainFile = commonDir+QString(MAINFILENAME);

    QFile file(mainFile);
    if(!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QTextStream in(&file);

    agent = "Robot"+QString::number(rob_id);
    field = in.readLine();

    imgFolderPath = commonDir+QString(IMAGES_PATH);
    mirrorParamsPath = cfgDir+agent+"/"+QString(MIRRORFILENAME);
    imageParamsPath = cfgDir+agent+"/"+QString(IMAGEFILENAME);
    lutPath = cfgDir+agent+"/"+QString(LUTFILENAME);
    maskPath = cfgDir+agent+"/"+QString(MASKFILENAME);
    fieldMapPath = fieldsDir+field+".map";
    pidPath = cfgDir+agent+"/"+QString(PIDFILENAME);
    worldPath = cfgDir+agent+"/"+QString(WORLDFILENAME);
    kalmanPath = cfgDir+agent+"/"+QString(KALMANFILENAME);

    ROS_INFO("System Configuration : %s in %s Field",agent.toStdString().c_str(),
    field.toStdString().c_str());
    ROS_INFO("Looking for config files in %s",cfgDir.toStdString().c_str());

    file.close();

    QFile file2(fieldMapPath);
    if(!file2.open(QIODevice::ReadOnly)) {
        ROS_ERROR("Error reading %s",fieldMapPath.toStdString().c_str());
        return false;
    }
    QTextStream in2(&file2);

    QStringList mapConfigs = in2.readLine().split(",");

    ROS_INFO("WorldMap for %s field has been initialized.",field.toStdString().c_str());
    return true;
}

QString ImageProcessor::getField()
{
    return field;
}

// Initializes data to perform RLE analysis
void ImageProcessor::rleModInitialization()
{
    // Create Scan Lines used by RLE Algorithm
    idxImage = Mat(IMG_SIZE,IMG_SIZE,CV_8UC1,Scalar(0));
    linesRad = ScanLines(idxImage,UAV_RADIAL, Point(imageConf.center_x,imageConf.center_y), 120, 75, 235,2,1);
    linesCir = ScanLines(idxImage,UAV_CIRCULAR,Point(imageConf.center_x,imageConf.center_y), 20, 75, 235,0,0);
}

// Preprocessed current image, preparing it for RLE scan
void ImageProcessor::preProcessIndexedImage()
{
    //int n_pixeis = 0;
    vector<int> s;
    Point temp;

    // Pre process radial sensors
    // TODO :: Find which is faster -> set or copy
    idxImage.setTo(0);
    for (unsigned k = 0 ; k < linesRad.scanlines.size() ; k++){
        s = linesRad.getLine(k);
        for(unsigned i = 0; i < s.size(); i++)
        {
            temp = linesRad.getPointXYFromInteger(s[i]);
            idxImage.ptr()[s[i]] = getClassifier(temp.x,temp.y);
        }
    }
    // Pre process circular sensors
    for (unsigned k = 0 ; k < linesCir.scanlines.size() ; k++){
        s = linesCir.getLine(k);
        //n_pixeis = n_pixeis + s.size();
        for(unsigned i = 0; i < s.size(); i++)
        {
            temp = linesCir.getPointXYFromInteger(s[i]);
            idxImage.ptr()[s[i]] = getClassifier(temp.x,temp.y);
        }
    }
    /*std::cerr << n_pixeis << endl;
    n_pixeis = 0;*/
}

void ImageProcessor::drawInterestInfo(Mat *buffer)
{
   rleBallRad.drawInterestPoints(Scalar(255,0,0),buffer,UAV_ORANGE_BIT);
   rleObs.drawInterestPoints(Scalar(255,0,255),buffer,UAV_BLACK_BIT);
   rleObs_2.drawInterestPoints(Scalar(255,0,255),buffer,UAV_BLACK_BIT);
   //rleLinesRad_2.drawInterestPoints(Scalar(255,0,255),buffer,UAV_BLACK_BIT);
   rleLinesRad.drawInterestPoints(Scalar(0,0,255),buffer,UAV_WHITE_BIT);
   rleLinesCir.drawInterestPoints(Scalar(0,255,255),buffer,UAV_WHITE_BIT);

}

void ImageProcessor::drawScanlines(Mat *buffer)
{
  Mat img(IMG_SIZE,IMG_SIZE, CV_8UC3, Scalar(0,0,0));
  linesRad.draw(img,Scalar(0,255,0));
  //rleLinesRad_2.draw(Scalar(255,255,255), Scalar(0,0,0), Scalar(255,255,255), buffer);
  rleLinesRad.draw(Scalar(255,255,255), Scalar(255,255,255), Scalar(255,255,255), &img);
  linesCir.draw(img,Scalar(0,255,0));
  rleLinesCir.draw(Scalar(255,255,255), Scalar(255,255,255), Scalar(255,255,255), &img);
  rleObs.draw(Scalar(255,255,255), Scalar(255,0,255), Scalar(255,255,255), &img);
  rleObs_2.draw(Scalar(255,255,255), Scalar(255,0,255), Scalar(255,255,255), &img);
  rleBallRad.draw(Scalar(255,255,255), Scalar(0,255,255), Scalar(255,255,255), &img);

  //rleLinesRad_2.draw(Scalar(255,255,255), Scalar(255,0,255), Scalar(255,255,255), buffer);

  *buffer=img;
}
void ImageProcessor::drawWorldInfo(Mat *buffer)
{
   for(int i=0;i<linePoints.size();i++){
       cv::circle(*buffer, linePoints[i], 2, Scalar(0,0,255), 2);
   }
   for(int i=0;i<obstaclePoints.size();i++){
       cv::circle(*buffer, obstaclePoints[i], 2, Scalar(255,0,255), 2);
   }
   for(int i=0;i<ballPoints.size();i++){
       cv::circle(*buffer, ballPoints[i], 2, Scalar(255,0,0), 4);
   }
}


// Draws points to real world
void ImageProcessor::drawWorldPoints(Mat *buffer)
{
	Mat img(IMG_SIZE,IMG_SIZE, CV_8UC3, Scalar(0,255,0));

	double conversion = (IMG_SIZE/2)/mirrorConf.max_distance;
  circle(img,getCenter(),RROBOT*conversion,Scalar(255,0,0),CV_FILLED,8,0);

	for(unsigned int i=0;i<mappedLinePoints.size();i++){
    circle(img,Point((IMG_SIZE/2)-conversion*mappedLinePoints[i].x,(IMG_SIZE/2)+conversion*mappedLinePoints[i].y),3,Scalar(255,255,255),CV_FILLED,8,0);
	}

  *buffer=img;

  obsBlob.draw(Scalar(150,150,150), buffer, getMaxDistance());
  ballBlob.draw(Scalar(0,0,255), buffer, getMaxDistance());

  for(unsigned int j=0;j<mappedObstaclePoints.size();j++){
    circle(*buffer,Point(imageConf.center_x-conversion*mappedObstaclePoints[j].x,imageConf.center_y+conversion*mappedObstaclePoints[j].y),2,Scalar(0,0,255),CV_FILLED,8,0);
  }

  for(unsigned int j=0;j<mappedBallPoints.size();j++){
    circle(*buffer,Point(imageConf.center_x-conversion*mappedBallPoints[j].x,imageConf.center_y+conversion*mappedBallPoints[j].y),2,Scalar(211,0,148),CV_FILLED,8,0);
  }
}

// Detects interest points, as line points, ball points and obstacle points using RLE
void ImageProcessor::detectInterestPoints()
{
    // Preprocess camera image, indexing it with predefined labels
    preProcessIndexedImage();

    // Run Different RLE's

    // RLE Lines
    rleLinesRad = RLE(linesRad, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 4, 2, 4, 30);
    rleLinesCir = RLE(linesCir, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 4, 2, 4, 20);
    //rleLinesRad_2 = RLE(linesRad, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 2, 1, 2, 30);

    // RLE Ball
    rleBallRad = RLE(linesRad, UAV_GREEN_BIT, UAV_ORANGE_BIT, UAV_GREEN_BIT, int(ballRLE.value_a), ballRLE.value_b, ballRLE.value_c, ballRLE.window);
    //rleBallRad = RLE(linesRad, UAV_NOCOLORS_BIT, UAV_ORANGE_BIT, UAV_NOCOLORS_BIT, 0, ballRLE.value_b, 0, 40);

    // RLE Obstacles
    rleObs = RLE(linesRad, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 4, 2, 0, 30);
    rleObs_2 = RLE(linesRad, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 0, 30, 0, 5);
    //rleLinesRad_2 = RLE(linesCir, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 4, 1, 4, 30);

    //Analyze and parse RLE's
    linePoints.clear(); obstaclePoints.clear(); ballPoints.clear();


    // Lines RLE
    rleLinesRad.LinespushData(linePoints, idxImage, distPix, distPixVal, Point(imageConf.center_x,imageConf.center_y));
    //rleLinesRad_2.LinespushData(linePoints, linePointsLength, idxImage);
    rleLinesCir.LinespushDataC(linePoints, idxImage);

    // Obstacles RLE
    rleObs.pushData(obstaclePoints, idxImage);
    rleObs_2.pushData(obstaclePoints, idxImage);
    //rleLinesRad_2.pushData(obstaclePoints, idxImage);

    // Ball RLE
    rleBallRad.pushData(ballPoints, idxImage);

}

void ImageProcessor::creatWorld()
{
  // Obsrtacle blobs initialization
  obsBlob = Blob(); ballBlob = Blob();

  // Create Blobs due to obstacle points detected
  obsBlob.createBlobs(mappedObstaclePoints, obsParameters.value_a, obsParameters.value_b, getCenter(), OBS_BLOB);
  ballBlob.createBlobs(mappedBallPoints, ballParameters.value_a, ballParameters.value_b, getCenter(), BALL_BLOB);

}

// Maps poins to real world
void ImageProcessor::mapPoints(int robot_heading)
{
  int count = 0;
   mappedLinePoints.clear(); mappedObstaclePoints.clear(); mappedBallPoints.clear();

   Point2d point;
   Point3d point_v2;
   double sqDist = 0.00;

   // Map line points
   for(unsigned int i = 0; i < linePoints.size(); i++){
       point_v2 = worldMappingP3(linePoints[i]);
       if(point_v2.x<=(mirrorConf.max_distance-0.5) && point_v2.x>RROBOT) {

          point.x = point_v2.x;
          point.y = point_v2.y;

          point = mapPointToRobot(robot_heading, point);

          point_v2.x = point.x;
          point_v2.y = point.y;

          mappedLinePoints.push_back(point_v2);

        }
   }
   // Map obstacle points
   for(unsigned int j = 0; j < obstaclePoints.size(); j++){
     point = worldMapping(obstaclePoints[j]);
     if(point.x<=(mirrorConf.max_distance-0.5) && point.x>RROBOT) mappedObstaclePoints.push_back(mapPointToRobot(robot_heading, point));
   }

   for(unsigned int j = 0; j < ballPoints.size(); j++){
     point = worldMapping(ballPoints[j]);
     if(point.x<=(mirrorConf.max_distance-0.5) && point.x>RROBOT) mappedBallPoints.push_back(mapPointToRobot(robot_heading, point));
   }
}


Point2d ImageProcessor::mapPointToRobot(double orientation, Point2d dist_lut)
{
   // Always mapped in relation to (0,0)
   double pointRelX = dist_lut.x*cos((dist_lut.y)*DEGTORAD);
   double pointRelY = dist_lut.x*sin((dist_lut.y)*DEGTORAD);
   double ang = orientation*DEGTORAD;

   return Point2d(cos(ang)*pointRelX-sin(ang)*pointRelY,
                  sin(ang)*pointRelX+cos(ang)*pointRelY);
}

// Returns world distance (in meters) given a pixel distance
double ImageProcessor::d2pWorld(int pixeis)
{
    unsigned int index = 0;
    while(pixeis>distPix[index] && index<(distPix.size()-1))index++;
    if(index<=0)return 0;
    if(pixeis>distPix[distPix.size()-1]){
        return 100;
    }
    else return distReal[index-1]+(((pixeis-distPix[index-1])*(distReal[index]-distReal[index-1]))/(distPix[index]-distPix[index-1]));
}

// Returns the distance between two points
int ImageProcessor::d2p(Point p1, Point p2)
{
    return sqrt(pow(p2.x-p1.x,2)+pow(p2.y-p1.y,2));
}

// Maps a point (pixel) to world values, returning world distance and angle
Point2d ImageProcessor::worldMapping(Point p)
{
    if(p.x<0 || p.x>=IMG_SIZE || p.y<0 || p.y>=IMG_SIZE) return Point2d(100.0,0.00);
    return Point2d(distLookUpTable[p.x][p.y].x,distLookUpTable[p.x][p.y].y);
}

// Maps a point (pixel) to world values, returning world distance and angle
Point3d ImageProcessor::worldMappingP3(Point p)
{
    if(p.x<0 || p.x>=IMG_SIZE || p.y<0 || p.y>=IMG_SIZE) return Point3d(100.0,0.00,0.00);
    return distLookUpTable[p.x][p.y];
}

void ImageProcessor::generateMirrorConfiguration()
{
   double weight = 0.00;
   double sqDist = 0.00;
   // Initialize Distance look up table
   distLookUpTable = vector<vector<Point3d> >(IMG_SIZE*IMG_SIZE,vector<Point3d>(0));

   for(int i=0; i<=IMG_SIZE; i++){ // columns
     for(int j=0; j<=IMG_SIZE; j++){ // rows
         double dist = d2pWorld(d2p(Point(imageConf.center_x,imageConf.center_y),Point(j,i)));
         double angulo = (atan2((j-imageConf.center_x),(i-imageConf.center_y))*(180.0/M_PI))-imageConf.tilt;
         while(angulo<0.0)angulo+=360.0;
         while(angulo>360.0)angulo-=360.0;
         sqDist = sqrt(pow(dist,2));
         weight = (REFERENCE + DISTANCE)/ (DISTANCE + sqDist);
         distLookUpTable[j].push_back(Point3d(dist,angulo,weight));
     }
   }
}

void ImageProcessor::updateDists(double max, double step, vector<short unsigned int>pix_dists, vector<short unsigned int>line_length)
{
   distReal.clear(); distPix.clear();
   for(float dist=step;dist<=max;dist+=step) distReal.push_back(dist);
   for(int i=0;i<pix_dists.size();i++) distPix.push_back((double)pix_dists[i]);
   for(int i=0;i<line_length.size();i++) distPixVal.push_back((int)line_length[i]);

   mirrorConf.pixel_distances = pix_dists;
   mirrorConf.max_distance = max;
   mirrorConf.lines_length = line_length;
   mirrorConf.step = step;
}

// Paints a pixel accordingly to its classifier
void ImageProcessor::paintPixel(int x, int y, int classifier,Mat *buf)
{
    switch(classifier){
        case UAV_NOCOLORS_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(127,127,127));
            break;
        }
        case UAV_WHITE_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(255,255,255));
            break;
        }
        case UAV_GREEN_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(0,255,0));
            break;
        }
        case UAV_ORANGE_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(0,0,255));
            break;
        }
        case UAV_BLACK_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(255,0,0));
            break;
        }
    }
}

// Converts rgb struct into hsv values, returns hsv struct
// TODO :: Rewrite this function
hsv ImageProcessor::rgbtohsv(rgb in)
{
    hsv temp;
    int min = 0, max = 0, delta = 0;
    if(in.r<in.g)min=in.r; else min=in.g;
    if(in.b<min)min=in.b;

    if(in.r>in.g)max=in.r; else max=in.g;
    if(in.b>max)max=in.b;

    temp.v = max;                // v, 0..255
    delta = max - min;                      // 0..255, < v

    if(max != 0)
        temp.s = (int)(delta)*255/max;        // s, 0..255
    else {
        // r = g = b = 0        // s = 0, v is undefined
        temp.s = 0;
        temp.h = 0;
        return temp;
    }
    if(delta==0) temp.h = 0;
    else {
        if( in.r == max )
            temp.h = (in.g - in.b)*30/delta;        // between yellow & magenta
        else if( in.g == max )
            temp.h = 60 + (in.b - in.r)*30/delta;    // between cyan & yellow
        else
            temp.h = 120 + (in.r - in.g)*30/delta;    // between magenta & cyan

        while( temp.h < 0 ) temp.h += 180;
    }

    if(temp.h>160){
        temp.h = (int)(-0.11111*temp.h)+20;
    }
    return temp;
}

// Returns the classifier of a pixel based on the Look Up Table
int ImageProcessor::getClassifier(int x, int y)
{
    if(x<0 || x>=IMG_SIZE || y<0 || y>=IMG_SIZE) return UAV_NOCOLORS_BIT;
    Vec3b *mask_color = mask.ptr<Vec3b>(y);
    if(mask_color[x][2]==255 && mask_color[x][0]==0) return UAV_NOCOLORS_BIT;
    Vec3b *color = buffer->ptr<Vec3b>(y);
    long int index = (color[x][2]<<16) + (color[x][1]<<8) + (color[x][0]);
    return YUVLookUpTable[index];
}

// Sets the center of the image (preferably, 240x240)
void ImageProcessor::setCenter(int x, int y,int tilt)
{
    imageConf.center_x = x;
    imageConf.center_y = y;
    imageConf.tilt = tilt;
}

// Returns the current center of the image
Point ImageProcessor::getCenter()
{
    return Point(imageConf.center_x,imageConf.center_y);
}

// Sets the image buffer
void ImageProcessor::setBuffer(Mat *buf)
{
    buffer = buf;
}

// Return current robotHeight
double ImageProcessor::getRobotHeight()
{
    return robotHeight;
}

// Returns the availability of new camera frames
bool ImageProcessor::frameAvailable()
{
    return omniCamera->frameAvailable();
}

// Returns image from camera, *success = true if new image is available
Mat *ImageProcessor::getImage(bool *success)
{
    Mat *cambuf = omniCamera->getImage();
    if(cambuf!=NULL) { (*success) = true; cambuf->copyTo(*buffer); }
    else (*success) = false;
    return buffer;
}

// Returns image from static image, *success = true if new image is available
Mat *ImageProcessor::getStaticImage(bool *success)
{
    buffer = &static_image;
    (*success) = true;
    return &static_image;
}

// Returns binary image of the buffer, given YUV(or HSV) ranges
void ImageProcessor::getBinary(Mat *in, minho_team_ros::label labelconf)
{
    //Returns binary representation of a certain range
    Vec3b *pixel; // iterator to run through captured image
    rgb pix; hsv pix2;

    for(int i = 0; i < IMG_SIZE; ++i){
        pixel = in->ptr<Vec3b>(i);
        for (int j = 0; j<IMG_SIZE; ++j){
            pix.r = pixel[j][2]; pix.g = pixel[j][1]; pix.b = pixel[j][0];
            pix2 = rgbtohsv(pix);

            if((pix2.h>=labelconf.H.min)&&(pix2.h<=labelconf.H.max) &&
            (pix2.s>=labelconf.S.min)&&(pix2.s<=labelconf.S.max) &&
            (pix2.v>=labelconf.V.min)&&(pix2.v<=labelconf.V.max))
            {
                pixel[j][2] = 255;
                pixel[j][1] = 255;
                pixel[j][0] = 255;
            }else {
                pixel[j][2] = 0;
                pixel[j][1] = 0;
                pixel[j][0] = 0;
            }
        }
    }
}

// Returns segmented image of buffer, based on current LUT configuration
void ImageProcessor::getSegmentedImage(Mat *buffer)
{
    int nRows = IMG_SIZE; int nCols = IMG_SIZE;
    if (buffer->isContinuous()){ nCols *= nRows; nRows = 1;}
    Vec3b* pixel,*mPixel;
    long int index = 0;

    for(int i = 0; i < nRows; ++i){
        pixel = buffer->ptr<Vec3b>(i);
        mPixel = mask.ptr<Vec3b>(i);
        for (int j = 0; j<nCols; ++j){
            if(mPixel[j][2]==255 && mPixel[j][0]==0){// se é mascara
                pixel[j][2] = 127;
                pixel[j][1] = 127;
                pixel[j][0] = 127;
            }else {
                index = (pixel[j][2]<<16) + (pixel[j][1]<<8) + (pixel[j][0]);
                if ( YUVLookUpTable[index] == UAV_GREEN_BIT )//se é campo
                {
                    pixel[j][2] = 0;
                    pixel[j][1] = 255;
                    pixel[j][0] = 0;
                } else if ( YUVLookUpTable[index] == UAV_WHITE_BIT )//se é campo
                {
                    pixel[j][2] = 255;
                    pixel[j][1] = 255;
                    pixel[j][0] = 255;
                }else if ( YUVLookUpTable[index] == UAV_ORANGE_BIT )//se é bola
                {
                    pixel[j][2] = 255;
                    pixel[j][1] = 0;
                    pixel[j][0] = 0;
                }else if ( YUVLookUpTable[index] == UAV_BLACK_BIT)//se é obstaculo
                {
                    pixel[j][2] = 0;
                    pixel[j][1] = 0;
                    pixel[j][0] = 0;
                }else if ( YUVLookUpTable[index] == UAV_NOCOLORS_BIT)//se é mascara ou desconhecido
                {
                    pixel[j][2] = 127;
                    pixel[j][1] = 127;
                    pixel[j][0] = 127;
                }
            }
        }
    }

}

// Returns true if camera was successfully initialized
bool ImageProcessor::isReady()
{
    return omniCamera->isCameraReady();
}

// Prints to stdout camera parameters
void ImageProcessor::printCameraInfo()
{
    omniCamera->printCameraInfo();
}

// Returns true if imaging procedure was successfuly started
bool ImageProcessor::startImaging()
{
    return (omniCamera->connect()&omniCamera->startCapture());
}

// Closes camera feed
void ImageProcessor::closeCamera()
{
    omniCamera->closeCamera();
}

// Updates Look up Table values of the selected pixel, with a radious of rad around it, for the given label
void ImageProcessor::updateLookUpTable(Mat *buffer, int x, int y, int label, int rad)
{
    for(int i=x-rad;i<=x+rad;i++){
        for(int j=y-rad;j<=y+rad;j++){
            if(i<0 || i>IMG_SIZE) return;
            if(j<0 || j>IMG_SIZE) return;

            Vec3b *color = buffer->ptr<Vec3b>(j);
            long int index = (color[i][2]<<16) + (color[i][1]<<8) + (color[i][0]);
            YUVLookUpTable[index] = label;
        }
    }
}

// Reads look up table configuration
bool ImageProcessor::readLookUpTable()
{
    QFile file(lutPath);
    if(!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QTextStream in(&file);
    memset(&YUVLookUpTable,UAV_NOCOLORS_BIT,LUT_SIZE);

    QString line;
    while(!in.atEnd()){
      line = in.readLine();
      if(line[0]=='#') continue;
      else {
         if(line[0]=='['){ // Process single color configuration
            ;
         } else { // Process range color configuration
            QString label_name = line.left(line.indexOf("["));
            QString temp = line.left(line.indexOf("]"));
            int classifier = temp.right(temp.size()-temp.indexOf('[')-1).toInt();
            QStringList values = line.right(line.size()-line.indexOf('=')-1).split(",");

            if(values.size()!=6) {
               ROS_ERROR("Bad Configuration (1) in %s",LUTFILENAME); return false;
            }

            LABEL_t label;
            minho_team_ros::label *lb;
            if(label_name=="FIELD"){
               lb = &lutconfig.field;
            } else if(label_name=="LINE"){
               lb = &lutconfig.line;
            } else if(label_name=="BALL"){
               lb = &lutconfig.ball;
            } else if(label_name=="OBSTACLE"){
               lb = &lutconfig.obstacle;
            } else { ROS_ERROR("Bad Configuration (2) in %s",LUTFILENAME); return false; }

            lb->H.min = values[0].toInt(); lb->H.max = values[1].toInt();
            lb->S.min = values[2].toInt(); lb->S.max = values[3].toInt();
            lb->V.min = values[4].toInt(); lb->V.max = values[5].toInt();
         }
      }
    }

    generateLookUpTable();
    return true;
}

// Writes new look up table configuration to vision.cfg file
bool ImageProcessor::writeLookUpTable()
{
    QFile file(lutPath);
    if(!file.open(QIODevice::WriteOnly)){
        ROS_ERROR("Error writing to %s.",LUTFILENAME);
        return false;
    }
    QTextStream in(&file);

    in << "FIELD[1]="
    << lutconfig.field.H.min << "," << lutconfig.field.H.max << ","
    << lutconfig.field.S.min << "," << lutconfig.field.S.max << ","
    << lutconfig.field.V.min << "," << lutconfig.field.V.max << "\n";
    in << "LINE[2]="
    << lutconfig.line.H.min << "," << lutconfig.line.H.max << ","
    << lutconfig.line.S.min << "," << lutconfig.line.S.max << ","
    << lutconfig.line.V.min << "," << lutconfig.line.V.max << "\n";
    in << "BALL[4]="
    << lutconfig.ball.H.min << "," << lutconfig.ball.H.max << ","
    << lutconfig.ball.S.min << "," << lutconfig.ball.S.max << ","
    << lutconfig.ball.V.min << "," << lutconfig.ball.V.max << "\n";
    in << "OBSTACLE[8]="
    << lutconfig.obstacle.H.min << "," << lutconfig.obstacle.H.max << ","
    << lutconfig.obstacle.S.min << "," << lutconfig.obstacle.S.max << ","
    << lutconfig.obstacle.V.min << "," << lutconfig.obstacle.V.max << "\n";

    QString message = QString("#NAME[CLASSIFIER]=HMIN,HMAX,SMIN,SMAX,VMIN,VMAX\r\n")+
                      QString("#[R,G,B]=CLASSIFIER\r\n")+
                      QString("#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
    in << message;
    file.close();
    return true;
}

bool ImageProcessor::writeMirrorConfig()
{
   QFile file(mirrorParamsPath);
    if(!file.open(QIODevice::WriteOnly)){
        ROS_ERROR("Error writing to %s.",MIRRORFILENAME);
        return false;
    }
    QTextStream in(&file);

    in<<"MAX_DISTANCE="<<mirrorConf.max_distance<<"\r\n";
    in<<"STEP="<<mirrorConf.step<<"\r\n";
    QString dists = "", length = "";
    if(mirrorConf.pixel_distances.size() == mirrorConf.lines_length.size()){
      for(unsigned int i=0;i<mirrorConf.pixel_distances.size();i++){
        dists+=QString::number(mirrorConf.pixel_distances[i])+QString(",");
        length+=QString::number(mirrorConf.lines_length[i])+QString(",");
      }
      dists = dists.left(dists.size()-1);
      length = length.left(length.size()-1);
      in<<"PIXEL_DISTANCES="<<dists<<"\r\n";
      in<<"LINES_LENGTH="<<length<<"\r\n";
    }
    else ROS_ERROR("Error!! Sizes not equal on %s!!",MIRRORFILENAME);


    QString message = QString("#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
    in << message;
    file.close();
    return true;
}

bool ImageProcessor::writeImageConfig()
{
   QFile file(imageParamsPath);
    if(!file.open(QIODevice::WriteOnly)){
        ROS_ERROR("Error writing to %s.",IMAGEFILENAME);
        return false;
    }
    QTextStream in(&file);

    in<<"IMG_CENTER="<<imageConf.center_x<<","<<imageConf.center_y<<"\r\n";
    in<<"TILT="<<imageConf.tilt<<"\r\n";

    QString message = QString("#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
    in << message;
    file.close();
    return true;
}

// Resets the look up table (puts everything to zero)
void ImageProcessor::resetLookUpTable()
{
    memset(&YUVLookUpTable,UAV_NOCOLORS_BIT,LUT_SIZE);
}

// Generates new look up table given the ranges in values
void ImageProcessor::generateLookUpTable()
{
    rgb pix; hsv pix2;
    unsigned int index;
    for (int r=0; r<256; r++) // classify every RGB color into our LUT
        for (int g=0; g<256; g++)
            for (int b=0; b<256; b++)
            {
                 pix.r = r; pix.g = g; pix.b =b;
                 pix2 = rgbtohsv(pix);
                 index = (r<<16)+(g<<8)+b;

                //-- initialize on update --
                YUVLookUpTable[index] = UAV_NOCOLORS_BIT;
                //-- Reference Colour range --
                if (((pix2.h>=lutconfig.line.H.min) && (pix2.h<=lutconfig.line.H.max))
                   && ((pix2.s>=lutconfig.line.S.min) && (pix2.s<=lutconfig.line.S.max))
                   && ((pix2.v>=lutconfig.line.V.min) && (pix2.v<=lutconfig.line.V.max))){
                    YUVLookUpTable[index] = UAV_WHITE_BIT;
                }else if (((pix2.h>=lutconfig.field.H.min) && (pix2.h<=lutconfig.field.H.max))
                   && ((pix2.s>=lutconfig.field.S.min) && (pix2.s<=lutconfig.field.S.max))
                   && ((pix2.v>=lutconfig.field.V.min) && (pix2.v<=lutconfig.field.V.max))){
                    YUVLookUpTable[index] = UAV_GREEN_BIT;
                } else if (((pix2.h>=lutconfig.obstacle.H.min) && (pix2.h<=lutconfig.obstacle.H.max))
                   && ((pix2.s>=lutconfig.obstacle.S.min) && (pix2.s<=lutconfig.obstacle.S.max))
                   && ((pix2.v>=lutconfig.obstacle.V.min) && (pix2.v<=lutconfig.obstacle.V.max))){
                    YUVLookUpTable[index] = UAV_BLACK_BIT;
                }else if (((pix2.h>=lutconfig.ball.H.min) && (pix2.h<=lutconfig.ball.H.max))
                   && ((pix2.s>=lutconfig.ball.S.min) && (pix2.s<=lutconfig.ball.S.max))
                   && ((pix2.v>=lutconfig.ball.V.min) && (pix2.v<=lutconfig.ball.V.max))){
                    YUVLookUpTable[index] = UAV_ORANGE_BIT;
                }
            }
}

void ImageProcessor::updateLabelLutConf(LABEL_t label,minho_team_ros::label lb_conf)
{
   minho_team_ros::label *lb;
   if(label==FIELD) lb = &lutconfig.field;
   else if(label==LINE) lb = &lutconfig.line;
   else if(label==BALL) lb = &lutconfig.ball;
   else if(label==OBSTACLE) lb = &lutconfig.obstacle;
   else return;
   *lb = lb_conf;
}

mirrorConfig ImageProcessor::getMirrorConfAsMsg()
{
   return mirrorConf;

}

visionHSVConfig ImageProcessor::getVisionConfAsMsg()
{
   return lutconfig;
}

imageConfig ImageProcessor::getImageConfAsMsg()
{
   return imageConf;
}

// Set Camera Properties given the values contained in the message
void ImageProcessor::setCamPropreties(cameraProperty::ConstPtr msg)
{
  	if(msg->reset!=true){
  		if(msg->property_id==0) {
  			omniCamera->setBrigtness(msg->value_a);
  			omniCamera->setProps(BRI);
  			}
  		else if(msg->property_id==1) {
  			omniCamera->setGain(msg->value_a);
  			omniCamera->setProps(GAI);
  			}
  		else if(msg->property_id==2) {
  			omniCamera->setShutter(msg->value_a);
  			omniCamera->setProps(SHU);
  			}
  		else if(msg->property_id==3) {
  			omniCamera->setGamma(msg->value_a);
  			omniCamera->setProps(GAM);
  			}
  		else if(msg->property_id==4) {
  			omniCamera->setSaturation(msg->value_a);
  			omniCamera->setProps(SAT);
  			}
  		else if(msg->property_id==5) {

  				if(msg->blue==false){
  					omniCamera->setWhite_Balance_valueA(msg->value_a);
  					omniCamera->setProps(WB);
  				}
  				else {
  					omniCamera->setWhite_Balance_valueB(msg->value_a);
  					omniCamera->setProps(WB);
  				}
  			}
        omniCamera->writePropConfig();
  		} else if(msg->reset == true)omniCamera->toggleFirsttime();
}

// Returns vector of Camera Properties values
vector<float> ImageProcessor::getCamProperties()
{
  	vector<float> props(7);
  	if(camera){
  	props[0] = omniCamera->getBrigtness();
  	props[1] = omniCamera->getGain();
  	props[2] = omniCamera->getShuttertime();
  	props[3] = omniCamera->getGamma();
  	props[4] = omniCamera->getSaturation();
  	props[5] = omniCamera->getWhite_Balance_valueA();
  	props[6] = omniCamera->getWhite_Balance_valueB();
  	}
  	return props;
}

// Set PID controler given the values in the message
void ImageProcessor::setPropControlerPID(PID::ConstPtr msg)
{
  if(msg->calibrate!=true) {
    omniCamera->setPropControlPID(msg->property_id,msg->p,msg->i,msg->d,msg->blue);
    ROS_WARN("New PID value set.");
  }
	else {
    omniCamera->toggleCalibrate();
    ROS_WARN("Auto Calibration activated.");
  }
}

// Read from file PID controler of Properties
vector<float> ImageProcessor::getPropControlerPID()
{
    vector<float> pid_conf(23);

	 QFile file(pidPath);
	 if(!file.open(QIODevice::ReadOnly)) {
        ROS_ERROR("ERROR READING pid.cfg FILE");
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

	//This line is to do not read EXPOSSURE VALUES of file
	line = in.readLine();
	ROI Roi;

	//LEITURA DOS ROIS
	for(int i=0;i<2;i++){
		line = in.readLine();
		pid_list = line.right(line.size()-line.indexOf('=')-1).split(",");
		Roi.x = pid_list[0].toInt();
		Roi.y = pid_list[1].toInt();
		Roi.d = pid_list[2].toInt();
		if(i==0)whiteRoi=Roi;
		else blackRoi=Roi;
		}
    line = in.readLine();
    pid_list = line.right(line.size()-line.indexOf('=')-1).split(",");
    pid_conf[22] = pid_list[0].toDouble();
    pid_conf[23] = pid_list[1].toDouble();
    file.close();

    if(count_list!=0){
		ROS_ERROR("Bad Configuration in %s",PIDFILENAME);
	}

    return pid_conf;
}

 // Returns vector of type ROI containig white and black ROI's
vector<ROI> ImageProcessor::getRois()
{
	vector<ROI> roi(2);
	roi[0]=whiteRoi;
	roi[1]=blackRoi;

	return roi;
}

// Sets ROI's on camera
void ImageProcessor::setROIs(ROI::ConstPtr msg)
{
  	if(msg->white==true){
      omniCamera->setWhiteROI(msg->x,msg->y,msg->d);
    }
  	else {
      omniCamera->setBlackROI(msg->x,msg->y,msg->d);
    }
}

void ImageProcessor::changeBlobsConfiguration(worldConfig::ConstPtr msg)
{
  if(msg->ball_obs==true){
    ballParameters = *msg;
  }
  else obsParameters = *msg;

  WriteWorldConfiguration();
}

void ImageProcessor::changeRLEConfiguration(worldConfig::ConstPtr msg)
{
  ballRLE = *msg;
  WriteWorldConfiguration();
}

bool ImageProcessor::getWorldConfiguration()
{
  QFile file(worldPath);
  if(!file.open(QIODevice::ReadOnly)) {
    return false;
  }
  QTextStream in(&file);

  QString blob_obs = in.readLine();
  QString blob_ball = in.readLine();
  QString rle_ball = in.readLine();
  blob_obs = blob_obs.right(blob_obs.size()-blob_obs.indexOf('=')-1);
  blob_ball = blob_ball.right(blob_ball.size()-blob_ball.indexOf('=')-1);
  rle_ball = rle_ball.right(rle_ball.size()-rle_ball.indexOf('=')-1);
  QStringList values = blob_obs.split(",");

  if(values.size()!=2) {
     ROS_ERROR("Bad Configuration (1) in %s",WORLDFILENAME);
     return false;
  }

  obsParameters.value_a = values.at(0).toDouble();
  obsParameters.value_b = values.at(1).toDouble();
  obsParameters.ball_obs = false;
  obsParameters.RLE = false;

  values = blob_ball.split(",");
  if(values.size()!=2) {
     ROS_ERROR("Bad Configuration (1) in %s",WORLDFILENAME);
     return false;
  }
  ballParameters.value_a = values.at(0).toDouble();
  ballParameters.value_b = values.at(1).toDouble();
  ballParameters.ball_obs = true;
  obsParameters.RLE = false;

  values = rle_ball.split(",");
  if(values.size()!=4) {
     ROS_ERROR("Bad Configuration (1) in %s",WORLDFILENAME);
     return false;
  }
  ballRLE.value_a = values.at(0).toInt();
  ballRLE.value_b = values.at(1).toInt();
  ballRLE.value_c = values.at(2).toInt();
  ballRLE.window = values.at(3).toInt();
  ballRLE.ball_obs = false;
  ballRLE.RLE = true;

  file.close();

  return true;
}

bool ImageProcessor::WriteWorldConfiguration()
{
  QFile file(worldPath);
  if(!file.open(QIODevice::WriteOnly)){
  ROS_ERROR("Error writing to %s.",WORLDFILENAME);
  return false;
}
  QTextStream in(&file);

  in << "OBS[0]=" << obsParameters.value_a << "," << obsParameters.value_b << "\r\n";
  in << "BALL[1]=" << ballParameters.value_a << "," << ballParameters.value_b << "\r\n";
  in << "RLEBALL[2]=" << ballRLE.value_a << "," << ballRLE.value_b << "," << ballRLE.value_c << "," << ballRLE.window << "\r\n";
  QString message = QString("#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
  in << message;

  file.close();
  return true;
}


// Returns error of Property in use
Point2d ImageProcessor::getPropError(int prop_in_use)
{
  return omniCamera->getError(prop_in_use);
}

void ImageProcessor::setCalibrationTargets(cameraProperty::ConstPtr msg)
{
  if(msg->blue==true)omniCamera->setSatTarget(msg->val_a);
  else omniCamera->setLumiTarget(msg->val_a);

}

/*
// Detects ball position based on ball points or image segmentation
void ImageProcessor::detectBallPosition()
{
    ballCandidates.clear();
    int size = 60;int cx = 0, cy = 0;
    int size2 = size/2;
    Vec3b *colorRow = NULL; uchar *tempRow = NULL;Q_UNUSED(colorRow);
    int classifier = 0;
    for(unsigned int centroids = 0; centroids < ballCentroids.size(); centroids++){
        cx = ballCentroids[centroids].y; cy = ballCentroids[centroids].x;
        Mat tempCandidate = Mat(size,size,CV_8UC1,Scalar(0));
        for(int row=-size2+cx;row<size2+cx;row++){
            colorRow = original.ptr<Vec3b>(row);
            tempRow = tempCandidate.ptr<uchar>(row-cx+size2);
            for(int col=-size2+cy; col<size2+cy;col++){
                classifier = getClassifier(col,row);
                if(classifier != UAV_ORANGE_BIT){
                    tempRow[col-cy+size2] = 255;
                }

            }
        }

        //Analyse tempCandidate which was the candidate (binary representation)
        vector<Mat> contours; Moments moment;
        double area;
        Point2f center; float radius;
        findContours(tempCandidate, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        for (unsigned int i = 0; i < contours.size(); i++){
            if (contourArea(contours[i])>3 && contourArea(contours[i])<2500){
                moment = moments(contours[i], true);
                area = moment.m00;
                Rect rBound = boundingRect(contours[i]); // calculate min enclosing circle and rectangle
                minEnclosingCircle((Mat)contours[i], center, radius);

                double wh = fabs(1 - ((double)rBound.width / rBound.height)); // calculate circle ratios
                double pi = fabs(1 - (fabs(area)) / (CV_PI * pow(radius, 2)));
                if (wh<=0.6 && pi<=0.6){
                    Point cc = center;
                    cc.x += ballCentroids[centroids].x-size2;
                    cc.y += ballCentroids[centroids].y-size2;
                    ballCandidates.push_back(Point3d(cc.x,cc.y,radius));
                    circle(original,cc,radius,Scalar(255,255,0),2);
                }
            }
        }
    }
}
*/

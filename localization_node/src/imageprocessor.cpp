#include "imageprocessor.h"

//Constructor of the class
ImageProcessor::ImageProcessor(bool begin)
{
    // Initialize GigE Camera Driver
    topCam = new BlackFlyCamera(false); //TopCam Handler

    // Initialize basic robot and field definitions
    if(!initializeBasics()){
      ROS_ERROR("Error Reading main.cfg.");
      exit(7);
    } else ROS_INFO("Reading information for %s.",agent.toStdString().c_str());
      
    // Assign mask image
    mask = imread(maskPath.toStdString());
    if(mask.empty()){
        ROS_ERROR("Error Reading mask.png.");
        exit(1);
    }else ROS_INFO("mask.png Ready.");

    // Read color segmentation Look Up Table
    if(!readLookUpTable()){ // Read and Initialize Look Up Table
        memset(&YUVLookUpTable,UAV_NOCOLORS_BIT,LUT_SIZE);
        ROS_ERROR("Error Reading vision.cfg.");
        exit(2);
    } else ROS_INFO("vision.cfg Ready.");

    // Initialize world mapping parameters
    if(!initWorldMapping()){ // Initialize World Mapping
        ROS_ERROR("Error Reading visionparams.cfg.");
        exit(3);
    } else ROS_INFO("visionparams.cfg Ready.");

    variablesInitialization(); // Initialize common Variables
    rleModInitialization(); // Initialize RLE mod data

    // Initialize GigE Camera Image Feed
    if(begin){
        if(startCamera()) {
            startImaging();
        } else {
            ROS_ERROR("Error Starting GigE Camera.");
            exit(4);
        }
    }
}

// Miscellaneous variables Initialization
void ImageProcessor::variablesInitialization()
{
    robotHeight = 0.74;
    processed = new Mat(480,480,CV_8UC3,Scalar(0,0,0));
    double morph_size = 1.5;
    MAX_DISTANCE = 6.0;
    element = getStructuringElement(2, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
    sizeRelThreshold = 0.65;
    piThreshold = 0.6;
}

// World mapping configuration initialization, returns true if successfully initialized
bool ImageProcessor::initWorldMapping()
{
    QFile file(visionParamsPath);
    if(!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QTextStream in(&file);

    QString realDist = in.readLine(); // Read from file
    QString mappDist = in.readLine();

    QStringList realDists = realDist.split(","); // Split
    for(int i=0;i<realDists.size();i++)distReal.push_back(realDists[i].toDouble());
    QStringList mappedDists = mappDist.split(",");
    for(int i=0;i<mappedDists.size();i++)distPix.push_back(mappedDists[i].toDouble());

    QString line = in.readLine();
    QStringList values = line.split(",");

    QStringList realBall = in.readLine().split(",");
    for(int i=0;i<realBall.size();i++)ballReal.push_back(realBall[i].toDouble());
    QStringList pixBall = in.readLine().split(",");
    for(int i=0;i<pixBall.size();i++)ballPix.push_back(pixBall[i].toDouble());

    if(values.size()!=2) {
        return false;
    }else {
        centerX = values.at(0).toInt();
        centerY = values.at(1).toInt();
    }

    distLookUpTable.clear();
    distLookUpTable = vector<vector<Point2d> >(480*480,vector<Point2d>(0));

    for(int i=0; i<=480; i++){ // columns
        for(int j=0; j<=480; j++){ // rows
            double dist = d2pWorld(d2p(Point(centerX,centerY),Point(j,i)));
            double angulo = (atan2((j-centerX),(i-centerY))*(180.0/M_PI))-45;
            while(angulo<0.0)angulo+=360.0;
            while(angulo>360.0)angulo-=360.0;
            distLookUpTable[j].push_back(Point2d(dist,angulo));
        }
    }

    line = in.readLine();
    linImu.clear();
    QStringList imuV = line.split(",");
    for(int i=0;i<imuV.size();i++)
        linImu.push_back(imuV.at(i).toInt());
    line = in.readLine();
    linTrue.clear();
    QStringList imuT = line.split(",");
    for(int i=0;i<imuT.size();i++)
        linTrue.push_back(imuT.at(i).toInt());

    m.resize(linTrue.size()); b.resize(linTrue.size());

    for(unsigned int i=0;i<m.size()-1;i++){
        m.at(i) = (linTrue.at(i+1)-linTrue.at(i))/(linImu.at(i+1)-linImu.at(i));
        b.at(i) = linTrue.at(i)-m.at(i)*linImu.at(i);
    }

    file.close();
    return true;
}

bool ImageProcessor::initializeBasics()
{
    QFile file(mainFile);
    if(!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QTextStream in(&file);

    agent = in.readLine();
    field = in.readLine();

    visionParamsPath = QString(configFolderPath)+agent+QString(visParamsFileName);
    lutPath = QString(configFolderPath)+agent+QString(lutFileName);
    maskPath = QString(configFolderPath)+agent+QString(maskFileName);

    file.close();
    return true;
}

QString ImageProcessor::getField()
{
    return field;
}

float ImageProcessor::linearizeIMU(float imu)
{
    if(imu>0.0 && imu<=105.0) {
        return imu*0.857143+0;
    }else if(imu>105 && imu<=205) {
        return imu*0.9-4.5;
    }else if(imu>205 && imu<=280) {
        return imu*1.2-66;
    }else if(imu>280 && imu<=360) {
       return imu*1.125-45;
    }else return 0;
}

// Initializes data to perform RLE analysis
void ImageProcessor::rleModInitialization()
{
    // Create Scan Lines used by RLE Algorithm
    idxImage = Mat(480,480,CV_8UC1,Scalar(0));
    linesRad = ScanLines(idxImage,UAV_RADIAL, Point(centerX,centerY), 180, 30, 260,2,1);
    linesCir = ScanLines(idxImage,UAV_CIRCULAR,Point(centerX,centerY),15,60,240,0,0);
}

// Preprocessed current image, preparing it for RLE scan
void ImageProcessor::preProcessIdx()
{
    vector<int> s;
    Point temp;

    // Pre process radial sensors
    idxImage.setTo(0);
    int classifier = 0;
    for (unsigned k = 0 ; k < linesRad.scanlines.size() ; k++){
        s = linesRad.getLine(k);
        for(unsigned i = 0; i < s.size(); i++)
        {
            temp = linesRad.getPointXYFromInteger(s[i]);
            classifier = getClassifier(temp.x,temp.y);
            idxImage.ptr()[s[i]] = classifier;

        }
    }

    // Pre process circular sensors
    for (unsigned k = 0 ; k < linesCir.scanlines.size() ; k++){
        s = linesCir.getLine(k);
        for(unsigned i = 0; i < s.size(); i++)
        {
            temp = linesCir.getPointXYFromInteger(s[i]);
            classifier = getClassifier(temp.x,temp.y);
            idxImage.ptr()[s[i]] = classifier;

        }
    }

    buffer->copyTo(original);
}

// Detects interest points, as line points, ball points and obstacle points using RLE
void ImageProcessor::detectInterestPoints()
{
    // Preprocess camera image, indexing it with predefined labels
    preProcessIdx(); //Optimize this and re-write valid points
    // Run Different RLE's
    rleBallRad = RLE(linesRad, UAV_GREEN_BIT, UAV_ORANGE_BIT, UAV_GREEN_BIT, 3, 2, 3, 30);
    rleBallCir = RLE(linesCir, UAV_GREEN_BIT, UAV_ORANGE_BIT, UAV_GREEN_BIT, 3, 2, 3, 30);
    rleLinesCir = RLE(linesCir, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 5, 2, 3, 30);
    rleLinesRad = RLE(linesRad, UAV_GREEN_BIT, UAV_WHITE_BIT, UAV_GREEN_BIT, 2, 2, 1, 10);
    rleObs = RLE(linesRad, UAV_GREEN_BIT, UAV_BLACK_BIT, UAV_GREEN_BIT, 2, 10, 0, 20);

    //Analyze and parse RLE's
    linePoints.clear();
    obstaclePoints.clear();
    ballPoints.clear();
    rleLinesRad.pushData(linePoints,idxImage,UAV_WHITE_BIT);
    rleLinesCir.pushData(linePoints,idxImage,UAV_WHITE_BIT);
    rleObs.pushData(obstaclePoints,idxImage,UAV_BLACK_BIT);
    rleBallCir.pushData(ballPoints,idxImage,UAV_ORANGE_BIT);
    rleBallRad.pushData(ballPoints,idxImage,UAV_ORANGE_BIT);

    //Detect Ball Candidates and using K-Means Algorithm
    ballCentroids.clear();
    kMeans ballKMeans(ballPoints,(int)ceil(ballPoints.size()/4)+1,30);
    ballCentroids = ballKMeans.getClusters();
}

// Returns world distance (in meters) given a pixel distance
double ImageProcessor::d2pWorld(int pixeis)
{
    unsigned int index = 0;
    while(pixeis>distPix[index] && index<(distPix.size()-1))index++;
    if(index<=0)return 0;
    if(pixeis>distPix[distPix.size()-1]){
        return -1;
    }
    else return distReal[index-1]+(((pixeis-distPix[index-1])*(distReal[index]-distReal[index-1]))/(distPix[index]-distPix[index-1]));
}

// Maps a point (pixel) to world values, returning world distance and angle
Point2d ImageProcessor::worldMapping(Point p)
{
    if(p.x<0 || p.x>=480 || p.y<0 || p.y>=480) return Point2d(100.0,100.00);
    return Point2d(distLookUpTable[p.x][p.y].x,distLookUpTable[p.x][p.y].y);
}

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

// Returns if the distance vs size of a ball candidate is valid or not
bool ImageProcessor::ballMorphology(double dist, double rads)
{
    if(rads>26|| dist> 3.0 || rads<=4) return false;
    if(dist<0.45) return true;
    unsigned int index = 0;
    while(dist>ballReal[index] && index<(ballReal.size()-1))index++;
    double expected = 0.0;
    if(index==0) expected = 26+(((dist-0.3)*(ballPix[index]-26))/(ballReal[index]-0.3));
    else expected = ballPix[index-1]+(((dist-ballReal[index-1])*(ballPix[index]-ballPix[index-1]))/(ballReal[index]-ballReal[index-1]));
    double error = abs(expected-rads);
    if((error/expected)<=0.8) return true;
    else return false;
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
    if(x<0 || x>=480 || y<0 || y>=480) return UAV_NOCOLORS_BIT;
    Vec3b *mask_color = mask.ptr<Vec3b>(y);
    if(mask_color[x][2]==255 && mask_color[x][0]==0) return UAV_NOCOLORS_BIT;
    Vec3b *color = buffer->ptr<Vec3b>(y);
    long int index = (color[x][2]<<16) + (color[x][1]<<8) + (color[x][0]);
    return YUVLookUpTable[index];
}

// Prints the name of a classifier
void ImageProcessor::printClassifier(int classifier)
{
    QString name = "";
    switch(classifier){
        case UAV_NOCOLORS_BIT:{
            name = "mask or unknown";
            break;
        }
        case UAV_WHITE_BIT:{
            name = "line";
            break;
        }
        case UAV_GREEN_BIT:{
            name = "field";
            break;
        }
        case UAV_BLACK_BIT:{
            name = "obstacle";
            break;
        }case UAV_ORANGE_BIT:{
            name = "ball";
            break;
        }
    }

    cout << name.toStdString() << endl;
}

// Sets the center of the image (preferably, 240x240)
void ImageProcessor::setCenter(int x, int y)
{
    centerX = x;
    centerY = y;
}

// Returns the current center of the image
Point ImageProcessor::getCenter()
{
    return Point(centerX,centerY);
}

// Sets the path of the static image
void ImageProcessor::setStaticImagePath(QString pth)
{
    staticImgPath = pth;
    staticImg = imread(pth.toStdString());
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

// Returns the distance between two points
int ImageProcessor::d2p(Point p1, Point p2)
{
    return sqrt(pow(p2.x-p1.x,2)+pow(p2.y-p1.y,2));
}

// Returns the availability of new camera frames
bool ImageProcessor::frameAvailable()
{
    return topCam->frameAvailable();
}

// Returns image from camera, *success = true if new image is available
Mat *ImageProcessor::getImage(bool *success)
{
    buffer = topCam->getImage(success);
    buffer->copyTo(original);
    return buffer;
}

// Returns binary image of the buffer, given YUV(or HSV) ranges
void ImageProcessor::getBinary(Mat *in,int ymin,int ymax, int umin, int umax, int vmin, int vmax)
{
    //Returns binary representation of a certain range
    Vec3b *pixel; // iterator to run through captured image
    rgb pix; hsv pix2;

    for(int i = 0; i < 480; ++i){
        pixel = in->ptr<Vec3b>(i);
        for (int j = 0; j<480; ++j){
            pix.r = pixel[j][2]; pix.g = pixel[j][1]; pix.b = pixel[j][0];
            pix2 = rgbtohsv(pix);

            if((pix2.h>=ymin)&&(pix2.h<=ymax) && (pix2.s>=umin)&&(pix2.s<=umax) && (pix2.v>=vmin)&&(pix2.v<=vmax))
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
    int nRows = 480; int nCols = 480;
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
                }else if ( YUVLookUpTable[index] == UAV_BLACK_BIT)//se é obstaculo ou desconhecido
                {
                    pixel[j][2] = 0;
                    pixel[j][1] = 0;
                    pixel[j][0] = 0;
                }else if ( YUVLookUpTable[index] == UAV_NOCOLORS_BIT)//se é obstaculo ou desconhecido
                {
                    pixel[j][2] = 127;
                    pixel[j][1] = 127;
                    pixel[j][0] = 127;
                }
            }
        }
    }

}

// Returns original caemra image
Mat* ImageProcessor::getOriginal()
{
    return &original;
}

// Returns true if camera was successfully initialized
bool ImageProcessor::isReady()
{
    return topCam->isCameraReady();
}

// Returns true if camera's image feed is ready
bool ImageProcessor::startCamera()
{
    return topCam->startCamera();
}

// Prints to stdout camera parameters
void ImageProcessor::printCameraInfo()
{
    topCam->printCameraInfo();
}

// Returns true if imaging procedure was successfuly started
bool ImageProcessor::startImaging()
{
    return topCam->startImaging();
}

// Closes camera feed
void ImageProcessor::closeCamera()
{
    topCam->closeCamera();
}

// Returns image in the defined static path
Mat* ImageProcessor::readStaticImage()
{
    buffer = &staticImg;
    return buffer;
}

// Updates Look up Table values of the selected pixel, with a radious of rad around it, for the given label
void ImageProcessor::updateLookUpTable(Mat *buffer, int x, int y, int label, int rad)
{
    for(int i=x-rad;i<=x+rad;i++){
        for(int j=y-rad;j<=y+rad;j++){
            if(i<0 || i>480) return;
            if(j<0 || j>480) return;

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

    QString line = in.readLine();
    QStringList values = line.split(",");

    /* 255*255*255 values */
    if(values.size()!=LUT_SIZE) {
        return false;
    }
    int val = UAV_NOCOLORS_BIT;
    for(int value = 0;value<LUT_SIZE;value++){
        val = values.at(value).toInt();
        YUVLookUpTable[value] = val;
    }

    file.close();
    return true;
}

// Writes new look up table configuration to vision.cfg file
bool ImageProcessor::writeLookUpTable()
{
    QFile file("../Configs/vision.cfg");
    if(!file.open(QIODevice::WriteOnly)){
        cout << "Error Writing lut.cfg"<< endl;
        return false;
    }
    QTextStream in(&file);

    /* 255*255*255 values */
    QString toWrite = "";
    int counter = 0;
    for(int value = 0;value<LUT_SIZE;value++){
        if(value==LUT_SIZE-1){
            toWrite = QString::number(YUVLookUpTable[value]);
            counter++;
        } else {
            toWrite = QString::number(YUVLookUpTable[value])+",";
            counter+=2;
        }
        in << toWrite;
        toWrite.clear();
    }
    in << "\n";
    /* write distances */

    file.close();
    return true;
}

// Resets the look up table (puts everything to zero)
void ImageProcessor::resetLookUpTable()
{
    memset(&YUVLookUpTable,UAV_NOCOLORS_BIT,LUT_SIZE);
}

// Generates new look up table given the ranges in values
void ImageProcessor::generateLookUpTable(int values[4][3][2])
{
    int y,u,v;
    unsigned int index;
    for (int r=0; r<256; r++) // classify every RGB color into our LUT
        for (int g=0; g<256; g++)
            for (int b=0; b<256; b++)
            {
                 y=(9798*r+19235*g+3736*b)>>15;u=((18514*(b-y))>>15)+128;v=((23364*(r-y))>>15)+128;
                 index = (r<<16)+(g<<8)+b;

                //-- initialize on update --
                YUVLookUpTable[index] = UAV_NOCOLORS_BIT;
                //-- Reference Colour range --
                if (((y>=values[1][0][0]) && (y<=values[1][0][1])) && ((u>=values[1][1][0]) && (u<=values[1][1][1])) &&
                        ((v>=values[1][2][0]) && (v<=values[1][2][1]))){
                    YUVLookUpTable[index] = UAV_WHITE_BIT;
                }else if (((y>=values[0][0][0]) && (y<=values[0][0][1])) && ((u>=values[0][1][0]) && (u<=values[0][1][1])) &&
                          ((v>=values[0][2][0]) && (v<=values[0][2][1]))){
                    YUVLookUpTable[index] = UAV_GREEN_BIT;
                } else if (((y>=values[2][0][0]) && (y<=values[2][0][1])) && ((u>=values[2][1][0]) && (u<=values[2][1][1])) &&
                           ((v>=values[2][2][0]) && (v<=values[2][2][1]))){
                    YUVLookUpTable[index] = UAV_BLACK_BIT;
                }else if (((y>=values[3][0][0]) && (y<=values[3][0][1])) && ((u>=values[3][1][0]) && (u<=values[3][1][1])) &&
                          ((v>=values[3][2][0]) && (v<=values[3][2][1]))){
                    YUVLookUpTable[index] = UAV_ORANGE_BIT;
                }
            }
}

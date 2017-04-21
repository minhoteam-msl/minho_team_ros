#ifndef KINECTVISION_H
#define KINECTVISION_H

#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_sync.h>
#include "imageprocessor.h"
#include "minho_team_ros/hardwareInfo.h"
#include "sensor_msgs/LaserScan.h"
#include "minho_team_ros/goalKeeperInfo.h"
#include "ros/ros.h"
#include "Utils/types.h"
#include "lidarlocalization.h"

using namespace cv;
using namespace std;
using namespace ros;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using sensor_msgs::LaserScan; //Namespace for laserScan information msg - SUBSCRIBING
using minho_team_ros::goalKeeperInfo; //Namespace for gk info information msg - PUBLISHING

class kinectVision
{

public:
    explicit kinectVision(char *argv,QObject *parent = 0);

	// Image reading using libfreenect sync
	bool getImages(Mat &depth_,Mat&rgb_);
	void setRobotPose(Point3d pose);
	
	double updateLimits (int mode, int minId);
	// Setup functions
	void configFilePaths();
    void generateLookUpTable();
    bool readParameters();
    void initVariables();	
private :
	double calculate_margin(int RunningMargin);
	double checkLimits( int mode , double RunningMargin);
	void detectGameBall();
	//Image processing Functions
	void filterByColor(bool show);
	void filterByAnatomy(bool show);
	void chooseBestCandidate(bool show);
	bool isInRange(int z, int val);
	// Math mappings
    double getZ(int pixVal);
    double getStdArea(double mVal);
    double getStdRads(double pixVal);
	Point2d mapPointToWorld(double rx, double ry, double angle, float dist, double theta, double offset);
	//Predictions
	bool intrestingEventHappened(double x, double y, double xi);
	bool crossGoalLine(double x);
	void predictBallPosition();
	// Localization
	void updateLidar(vector<float> *distances);
	void updateOdometry(float angle,int enc1,int enc2,int enc3);
private:
	// setup
	QString lutFile, paramFile,configFolderPath;
	ros::NodeHandle *parent;
	lidarLocalization *localization;
	//utilities
	QTimer *retrieveTimer,;
	Mat depthImage, rgbImage; 
	Mat binary, clear;
	QTime fpsReader; 
	int _index;	  
	int definedRatems;
	//Process Variables
	int YUVLookUpTable[256*256*256];
	vector<Point3i> candidates;
    vector<Vec6d> refinedCandidates;
    Mat elementdil,elementero;
    //Kinect Parameters
    double heightFromGround;
    double hFov, vFov;
    double convVertical, convHorizontal;
    int nRows, nCols;
    int LUTRanges[6];
    Mat aux;
    //world model
    Point3d ballPosition3D,lastBallPosition3D;
	Point3d ballVelocities;
	Point3d ballImpactZone;
	QTime timerVel;
	Point3d robotPose;
	Point3d ballPosWorld,lastBallPosWorld;
};

#endif // KINECTVISION_H

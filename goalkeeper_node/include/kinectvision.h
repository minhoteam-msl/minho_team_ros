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
#include "minho_team_ros/goalKeeperInfo.h"
#include "ros/ros.h"
#include "Utils/types.h"
#include <sys/time.h> 

using namespace cv;
using namespace std;
using namespace ros;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using minho_team_ros::goalKeeperInfo; //Namespace for gk info information msg - PUBLISHING

class kinectVision
{

public:
    kinectVision(ros::NodeHandle *par,fieldDimensions field);
    void updateLocalizationData(Point3f pose, Point3f vels);
    
    
    // Process
    double calculate_margin(int RunningMargin);
	double updateLimits (int mode, int minId);
	double checkLimits(int mode, double RunningMargin);
	
    // Image reading using libfreenect sync
	bool getImages(Mat &depth_,Mat&rgb_);
	bool detectGameBall();
	//Image processing Functions
	void filterByColor(bool show);
	void filterByAnatomy(bool show);
	void chooseBestCandidate(bool show);
	hsv rgbtohsv(rgb in);
	
	// Setup functions
	void configFilePaths();
    void generateLookUpTable();
    bool readParameters();
    void initVariables();	


	// Math mappings
	bool isInRange(int z, int val);
	double getZ(int pixVal);
    double getStdArea(double mVal);
    double getStdRads(double pixVal);
	Point2d mapPointToWorld(double rx, double ry, double angle, float dist, double theta, double offset);
	
	//Predictions
	bool intrestingEventHappened(double x, double y, double xi);
	bool crossGoalLine(double x);
	void predictBallPosition();
	void publishData();
	std::vector<std::string> split(const std::string& s, char seperator);
	
private: 
    ros::NodeHandle *parent;
    std::string lutFile, paramFile,configFolderPath;
    
    //State variables
    goalKeeperInfo current_state, last_state;
    //world model
    Point3d ballPosition3D,lastBallPosition3D;
	Point3d ballVelocities;
	Point3d ballImpactZone;

	Point3d robotPose;
	Point3d ballPosWorld,lastBallPosWorld;
	//
	ros::Publisher gk_info_pub;
    
    //Process Variables
	int YUVLookUpTable[256*256*256];
	Mat depthImage, rgbImage; 
	Mat binary, clear;
	vector<Point3i> candidates;
    vector<Vec6d> refinedCandidates;
    Mat elementdil,elementero;
    int LUTRanges[6];
    Mat aux;
    int _index;
    int nRows, nCols;
    double convVertical, convHorizontal;
    double hFov, vFov;
    double heightFromGround;
    
    int save_mind;
    int finalcounter;
    bool goodCandidateFound;
    fieldDimensions currentField;
    float goal_line_x;
    float side_line_y;
    struct timeval t1, t2;
    
};

#endif // KINECTVISION_H

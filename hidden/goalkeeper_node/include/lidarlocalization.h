#include "Utils/types.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>
#include "ros/ros.h"
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/baseStationInfo.h"
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;
using minho_team_ros::hardwareInfo;
using minho_team_ros::robotInfo;
using minho_team_ros::baseStationInfo; //Namespace for gk info information msg - PUBLISHING

class lidarLocalization
{
public:
    lidarLocalization();
    // Setup and Output
    void initField(std::string file_);
    Point3d getPose();
    Point3d getVelocities();
    vector<Point2f> getWorldPoints();
    fieldDimensions getField();
    std::vector<std::string> split(const std::string& s, char seperator);


    void readMapConfFile(std::string file_);
    struct nodo parseString(std::string str);
    double getDistance(double x,double y);
    double positionError(double rx, double ry, double angle);
    double errorFunction(double error);
    void calculateGlobalOptimum();
    void calculateBestLocalPose(double radius);
    
    // Estimators
    void updateOdometryEstimate(const hardwareInfo::ConstPtr &msg);
    void updateLidarEstimate(vector<float> *distances);
    void updateBaseStationInfo(const baseStationInfo::ConstPtr &msg);
    void fuseEstimates();
    void doReloc();
    
    // Math
    Point world2WorldModel(Point2d pos);
    Point2d mapPointToWorld(double rx,double ry,double angle, float dist, double theta, double offset);
    void assertPoseToGlobalPosition();
private:
    // Math
    void mapDetectedPoints(vector<float> *distances);
    float NormalizeAngle(float angulo);
    bool configFilePaths();
    void initKalmanFilter();
    double normalizeAngle(double angulo);
    void computeVelocities();
    int getQuadrant(double angle);
    

	//Localization
    int scale, outputResolution;
    double outputResolution_m,ceilaux;
    double xTransform, yTransform;
    float xMax,yMax;
    vector<vector<struct nodo> >map;
    vector<Point2d>worldPoints;
    vector<Point2f>detectedPoints;
    Mat field,worldModel;
    fieldDimensions fieldAnatomy;
    struct localizationEstimate odometry;
    struct localizationEstimate lidar;
    hardwareInfo current_hardware_state, last_hardware_state;
    
    
    Point3d pose,lastPose,poseM,lastPoseM;
    double meanError;
    Point2d tipPositionRight, tipPositionLeft;
    bool halfIsRight;
    // File Paths
    std::string mainFilePath, configFolderPath;
    std::string fieldPath, mapPath;
    bool readyHardware;
    // Kalman Filter
    struct MTKalmanFilter kalman;
    baseStationInfo bsInfo;
    Point3d velocities;
    bool doGlobalLocalization;
    
};

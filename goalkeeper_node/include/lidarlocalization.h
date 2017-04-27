#include "Utils/types.h"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdlib.h>
#include <QFile>
#include <QTextStream>
#include "ros/ros.h"
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/baseStationInfo.h"
#include <QTime>

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
    void initField(QString file_);
    Point3d getPose();
    Point3d getVelocities();
    vector<Point2f> getWorldPoints();
    fieldDimensions getField();


    void readMapConfFile(QString file_);
    struct nodo parseString(QString str);
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
    QString mainFilePath, configFolderPath;
    QString fieldPath, mapPath;
    bool readyHardware;
    // Kalman Filter
    struct MTKalmanFilter kalman;
    baseStationInfo bsInfo;
    Point3d velocities;
    bool doGlobalLocalization;
    
};

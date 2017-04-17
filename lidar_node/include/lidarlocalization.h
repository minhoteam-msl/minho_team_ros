#ifndef LIDARLOCALIZATION_H
#define LIDARLOCALIZATION_H

#include "Utils/types.h"

class lidarLocalization
{
public:
    explicit lidarLocalization(QObject *parent = 0);
    // Setup and Output
    void initField(QString file_);
    void drawWorldModel();
    Point3d getPose();
    Point3d getVelocities();
    vector<Point2f> getWorldPoints();
    void setWorldModelView(Mat *cop);
    void readMapConfFile(QString file_);
    struct nodo parseString(QString str);
    double getDistance(double x,double y);
    double positionError(double rx, double ry, double angle);
    double errorFunction(double error);
    void calculateGlobalOptimum();
    void calculateBestLocalPose(double radius);
    // Estimators
    void updateOdometryEstimate(float robAngle, int enc1, int enc2, int enc3);
    void updateLidarEstimate(vector<float> *distances);
    void fuseEstimates();
    // Math
    Point world2WorldModel(Point2d pos);
    Point2d mapPointToWorld(double rx,double ry,double angle, float dist, double theta, double offset);
    void generateDetectedPoints();
    void assertPoseToGlobalPosition();
private:
    // Math
    void mapDetectedPoints(vector<float> *distances);
    float NormalizeAngle(float angulo);
    void configFilePaths();
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
    struct hardware currentHardware;
    struct hardware lastHardware;
    Point3d pose,lastPose,poseM,lastPoseM;
    double meanError;
    Point2d tipPositionRight, tipPositionLeft;
    bool halfIsRight;
    // File Paths
    QString mainFilePath, configFolderPath;
    QString fieldPath, mapPath;
    bool readyHardware;
    QMutex hardwareMutex;
    // Kalman Filter
    struct MTKalmanFilter kalman;
    Point3d velocities;
    
};

#endif // LIDARLOCALIZATION_H

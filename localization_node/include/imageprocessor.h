#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include "kmeans.h"
#include "RLE.h"
#include <QTime>
#include "ScanLines.h"
#include "Vec.h"
#include "blackflycam.h"
#include "ros/ros.h"
#include "minho_team_ros/mirrorConfig.h"
#include "minho_team_ros/visionHSVConfig.h"
#include "minho_team_ros/imageConfig.h"
#include "minho_team_ros/cameraProperty.h"
#include "minho_team_ros/PID.h"
#include "minho_team_ros/ROI.h"
#include "minho_team_ros/worldConfig.h"
#include <iostream>
#include <vector>
#include "Blob.h"
//#include <QTime>
#define LINE_LIMIT 3
#define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember))

using namespace std;
using minho_team_ros::mirrorConfig;
using minho_team_ros::visionHSVConfig;
using minho_team_ros::imageConfig;
using minho_team_ros::cameraProperty;
using minho_team_ros::PID;
using minho_team_ros::ROI;
using minho_team_ros::worldConfig;

class ImageProcessor
{
public:
  QTime loctime;
   ImageProcessor(int rob_id, bool begin, bool *init_success);
   typedef Mat* (ImageProcessor::*imageAcquisitionFunction)(bool *success);
   /* Detection Functions */
   void detectInterestPoints(); // Detects linePoints, ballPoints and obstaclePoints
   //void detectBallPosition(); // Given the detected ballRLE find the optimal candidate
   void creatWorld();

   /* Image Output Functions */
   Mat *getImage(bool *success); // Returns camera image pointer
   Mat *getStaticImage(bool *success); // Returns static image pointer
   void getBinary(Mat *in,  minho_team_ros::label labelconf); // Returns thresholded HSV image
   void getSegmentedImage(Mat *buffer); // Returns buffer's segmented image
   void drawInterestInfo(Mat *buffer);
   void drawWorldInfo(Mat *buffer);
   void drawScanlines(Mat *buffer);
   void paintPixel(int x, int y, int classifier, Mat *buf); // Paints a certain pixel in the image
   hsv rgbtohsv(rgb in); // Converts rgb to hsv

   /* Camera driver wrapping functions */
   bool frameAvailable(); // Returns true if a new camera frame is available
   bool isReady(); // Returns true if camera is ready
   bool startImaging(); // Returns true if imaging was successfuly started
   void printCameraInfo(); // Prints in stdout some camera information
   void closeCamera(); // Closes camera feed

   /* Look up Table fuctions */
   void updateLookUpTable(Mat* buffer, int x, int y, int label, int rad); // Updates LUT cell value
   bool readLookUpTable(); // Reads look up table from file
   bool writeLookUpTable(); // Writes look up table to file
   void resetLookUpTable(); // Resets defined look up table
   void generateLookUpTable(); // Generates look up table based on values' ranges
   void updateLabelLutConf(LABEL_t label,minho_team_ros::label lb_conf);
   int getClassifier(int x,int y); // Returns classifier given a pixel and LUT configuration

   /* Camera-Robot Information */
   void setCenter(int x,int y,int tilt); // Sets robot center
   void setBuffer(Mat *buf); // Sets current buffer
   double getRobotHeight(); // Returns robot height
   Point getCenter(); // Returns robot center

   /* Mathmatic Functions */
   int d2p(Point p1,Point p2); // Returns distance between two points
   double d2pWorld(int pixeis); // Returns world distance given pixel distance
   Point2d worldMapping(Point p); // Maps point to world (world dist, angle)
   Point3d worldMappingP3(Point p); // Maps point to world (world dist, angle)

   /* Initializations */
   void variablesInitialization(); // Inits other variables
   bool initWorldMapping(); // Inits world mapping variables
   void generateMirrorConfiguration();
   bool writeMirrorConfig();
   bool writeImageConfig();
   void updateDists(double max, double step, vector<short unsigned int>pix_dists);
   mirrorConfig getMirrorConfAsMsg();
   inline double getMaxDistance() { return mirrorConf.max_distance; }
   visionHSVConfig getVisionConfAsMsg();
   imageConfig getImageConfAsMsg();
   bool initializeBasics(int rob_id);
   QString getField();
   inline QString getWorldFileName() { return fieldMapPath;}
   inline QString getKalmanFileName() { return kalmanPath;}
   void drawWorldPoints(Mat *buffer);

   void mapPoints(int robot_heading);
   Point2d mapPointToRobot(double orientation, Point2d dist_lut);

   /* WORLD STATE INFORMATION Buffers */
   vector<Point>linePoints;
   vector<int>linePointsLength;
   vector<Point>obstaclePoints;
   vector<Point>ballCentroids;
   vector<Point>ballPoints;

   vector<Point2d> mappedObstaclePoints, mappedBallPoints;
   vector<Point3d> mappedLinePoints;

   /* Image Acquisition function pointer */
   imageAcquisitionFunction acquireImage;

   /* Camera Calibrations Functions */
   void setCamPropreties(cameraProperty::ConstPtr msg); // Set Camera Properties
   vector<float>getCamProperties(); // Vector that contains camera Properties
   void setPropControlerPID(PID::ConstPtr msg); // Set controler of Camera Properties
   vector<float> getPropControlerPID(); // Returns Camera Properties value
   Point2d getPropError(int prop_in_use); // Returns error of property in use
   vector<ROI> getRois(); //  Returns ROI's in use
   void setROIs(ROI::ConstPtr msg); // Set ROI's to use
   void changeBlobsConfiguration(worldConfig::ConstPtr msg);
   void changeRLEConfiguration(worldConfig::ConstPtr msg);
   void setCalibrationTargets(cameraProperty::ConstPtr msg);
   void pushData_Validator();

   //Blobs
   Blob obsBlob, ballBlob;

   inline worldConfig getBallConfAsMsg(){ return ballParameters;}
   inline worldConfig getObsConfAsMsg(){ return obsParameters;}
   inline worldConfig getRLEConfAsMsg(){ return ballRLE;}

private:
   /* Camera Driver and parameters*/
   BlackflyCam *omniCamera;
   imageConfig imageConf;
   /*Image Containers */
   Mat *buffer,mask,static_image;
   Mat original;
   QString staticImgPath;
   Mat element;
   bool camera;

   /*Lut Variables*/
   int YUVLookUpTable[256*256*256];
   visionHSVConfig lutconfig;

   /* mirrorConfig and World Mapping */
   double sizeRelThreshold, piThreshold;
   mirrorConfig mirrorConf;
   vector<double> distPix;
   vector<double> distReal;
   vector<int> distPixVal;
   vector<vector<Point3d> >distLookUpTable;
   double robotHeight;

   // Implementation USING RLE
   void rleModInitialization(); // Inits RLE Mode sensors and RLE's
   void preProcessIndexedImage(); // Preprocesses image, labeling it for RLE scan
   ScanLines linesRad, linesCir; Mat idxImage;
   RLE rleBallRad,rleLinesRad,rleObs,rleBallCir,rleLinesCir,rleLinesRad_2,rleObs_2;

   //ConfigFiles strings
   QString field; QString agent;
   QString mirrorParamsPath, imageParamsPath ,maskPath,lutPath, pidPath;
   QString fieldMapPath, worldPath, kalmanPath;
   QString imgFolderPath;

   //ROI Variables
   ROI whiteRoi, blackRoi;

   // World Construction
   worldConfig ballParameters, obsParameters, ballRLE;
   bool getWorldConfiguration();
   bool WriteWorldConfiguration();

};

#endif // IMAGEPROCESSOR_H

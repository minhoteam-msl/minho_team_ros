#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <QObject>
#include "imageprocessor.h"
#include "configserver.h"
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/requestReloc.h"

using namespace ros;
using namespace cv;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using minho_team_ros::robotInfo; //Namespace for vision information msg - PUBLISHING
using minho_team_ros::requestReloc;

class Localization : public QObject
{
   Q_OBJECT
public:
   explicit Localization(ros::NodeHandle *par, bool *init_success, bool use_camera, QObject *parent = 0); // Constructor
   ~Localization();
private:
   //Major components
   ImageProcessor *processor;
   ConfigServer *confserver;
   QTimer *parentTimer;
   Mat *buffer, processed;
   int requiredTiming;
   float time_interval;
   //Confserver variables
   bool assigning_images;
   uint8_t assigning_type;
   //Hardware estimate variables
   bool is_hardware_ready;
   hardwareInfo current_hardware_state, last_hardware_state;
   robotInfo current_state, last_state, last_vel_state;
   localizationEstimate odometry;
   localizationEstimate vision;
   MTKalmanFilter kalman;
   //ROS publishers and subscribers
   ros::Publisher robot_info_pub;
   ros::Subscriber hardware_info_sub;
   //ROS Services
   ros::ServiceServer reloc_service;
   ros::ServiceServer ext_debug_service;
   bool send_extended_debug_info;
private slots:
   void initVariables();
   void initializeKalmanFilter();
   void stopImageAssigning();
   void changeImageAssigning(uint8_t type);
   void changeLookUpTableConfiguration(visionHSVConfig::ConstPtr msg);
   void changeMirrorConfiguration(mirrorConfig::ConstPtr msg);
   void changeImageConfiguration(imageConfig::ConstPtr msg);
   bool doReloc(requestReloc::Request &req,requestReloc::Response &res);
public slots:
   void hardwareCallback(const hardwareInfo::ConstPtr &msg);   
   void discoverWorldModel(); // Main Funcition
   void fuseEstimates(); // Fuse vision and odometry estimations
   void computeVelocities(); // Computes ball and robot velocities
   void decideBallPossession(); // decides wether the robot has or not the ball
   // Math Utilities
   float normalizeAngleRad(float angle);
   float normalizeAngleDeg(float angle);
};

#endif // LOCALIZATION_H

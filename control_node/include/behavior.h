#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "ros/ros.h"
#include "minho_team_ros/controlInfo.h" // PUBLISH
#include "minho_team_ros/robotInfo.h" // SUBSCRIBE
#include "minho_team_ros/aiInfo.h" // SUBSCRIBE
#include "minho_team_ros/requestKick.h" // CALLING SERVICE
#include "minho_team_ros/position.h"
#include <QFile>
#include <QString>
#include <iostream>
#include <sstream>
#include <pthread.h>
#include <time.h>
//defines for signal
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>

//include CGAL for Voronoi Diagram
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <iterator>

using namespace ros;
using namespace std;
using minho_team_ros::controlInfo;
using minho_team_ros::robotInfo;
using minho_team_ros::aiInfo;
using minho_team_ros::requestKick;
using minho_team_ros::position;

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 Point_2;
typedef K::Iso_rectangle_2 Iso_rectangle_2;
typedef K::Segment_2 Segment_2;
typedef K::Ray_2 Ray_2;
typedef K::Line_2 Line_2;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay_triangulation_2;

class Behavior
{
public:
   Behavior(std::string topics_base_name, ros::NodeHandle *par);
   
   void robotInfoCallback(const robotInfo::ConstPtr &msg);
   void aiInfoCallback(const aiInfo::ConstPtr &msg);
   void doWork();
   
private:
   /// \brief pointer to parent ros node
   ros::NodeHandle *parent;
   /// \brief ROS subscriber for robotInfo
   ros::Subscriber robot_info_sub;
   /// \brief ROS subscriber for aiInfo
   ros::Subscriber ai_info_sub;
   /// \brief ROS publisher for controlInfo
   ros::Publisher control_info_pub;
   /// \brief ROS service client requestKick
   ros::ServiceClient kick_service;
   
   // DATA STORAGE
   /// \brief struct to hold control commands (to be published)
   controlInfo control;
   /// \brief struct to hold most recent robotInfo message
   robotInfo robot_info;
   /// \brief struct to hold most recent aiInfo message
   aiInfo ai_info;
   
   // PID VARIABLES
   float error, previous_error;
   float derivative, integral;
   float kp_rot, ki_rot, kd_rot;
   
   // MATH FUNCTIONS
   float Distance(float positionAX, float positionAY, float positionBX, float positionBY);
   int PsiTarget(double positionAX, double positionAY, double positionBX, double positionBY, bool normalize);
   int RotationRobot_for_Target_PID(int psi_target, int rotational_speed_max);
   int SpeedRobot_TargetDistance_Log(double distanceRobot_Target, int linear_speed_max);
};

 
#endif // BEHAVIOR_H

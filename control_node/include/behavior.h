/*
 * LAR - Laboratório de Automação e Robótica
 * MinhoTeam robotic soccer team
 * http://www.robotica.dei.uminho.pt/robocup2016/
 * DEI - University of Minho, Portugal
 *
 * behavior.h
 *
 * Created: ,2016
 * Author: Tiago Maia
 *
 */

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

//
#include "fundamental.h"
#include "voronoi.h"
#include "dijkstrashortestpath.h"
#include "motion.h"


//
enum ROLE {GOALKEEPER = 0, DEFENSE, SUP_ATTACKER, ATTACKER };
enum ACTION {STOP = 0, GO_TO_POSITION, KICK_PASS, ENGAGE_BALL, TEST };


class Behavior
{
public:
   Behavior(std::string topics_base_name, int rob_id, bool mode_real, ros::NodeHandle *par, Fundamental *fund, Voronoi *vor, DijkstraShortestPath *dijk, Motion *mot);
   bool controlConfService(requestControlConfig::Request &req, requestControlConfig::Response &res);
   void robotInfoCallback(const robotInfo::ConstPtr &msg);
   void aiInfoCallback(const aiInfo::ConstPtr &msg);
   void controlConfigCallback(const controlConfig::ConstPtr &msg);
   void doWork();


private:
   bool initParameters();
   bool readControlParameters();
   bool writeControlParameters();
   void goToPosition1(robotInfo robot, aiInfo ai, controlConfig cconfig);
   void goToPosition2(robotInfo robot, aiInfo ai, controlConfig cconfig, const vector<Point>& path, int max_vel);

   /// \brief pointer to parent ros node
   ros::NodeHandle *parent;
   /// \brief ROS subscriber for robotInfo
   ros::Subscriber robot_info_sub;
   /// \brief ROS subscriber for aiInfo
   ros::Subscriber ai_info_sub;
    /// \brief ROS subscriber for controlConfig
   ros::Subscriber control_config_sub;
   /// \brief ROS publisher for controlInfo
   ros::Publisher control_info_pub;
   /// \brief ROS service client requestKick
   ros::ServiceClient kick_service;
   /// \brief Service to relay the configuration
   ros::ServiceServer service_control_config;
   /// \brief ROS publisher for pathData
   ros::Publisher path_data_pub;

   // DATA STORAGE
   /// \brief struct to hold control commands (to be published)
   controlInfo control_info;
   /// \brief struct to hold most recent robotInfo message
   robotInfo robot_info;
   /// \brief struct to hold most recent aiInfo message
   aiInfo ai_info;
   /// \brief struct to hold most recent controlConfig message
   controlConfig control_config;
   /// \brief struct to hold most recent pathData message
   pathData path_data;

   //
   Fundamental *fundamental;
   Voronoi *voronoi;
   DijkstraShortestPath *dijkstra_path;
   Motion *motion;
   //
   int robot_id;
   bool mode_real;
   QString controlparam_file;
   //
   QElapsedTimer timer_work;
   


};

 
#endif // BEHAVIOR_H

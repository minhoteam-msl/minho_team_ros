//C++ includes
#include <iostream>
#include <sstream>
#include "ros/ros.h"

//ROS includes
#include "std_msgs/String.h"
#include "minho_team_ros/hardwareInfo.h"
#include "sensor_msgs/LaserScan.h"

using namespace ros;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using sensor_msgs::LaserScan; //Namespace for laserScan information msg - SUBSCRIBING

void hardwareInfoCallback(const hardwareInfo::ConstPtr& msg)
{
	gk->updateOdometry(msg->imu_value,msg->encoder_1,msg->encoder_2,msg->encoder_3);
}

void laserScanCallback(const LaserScan::ConstPtr& msg)
{
	vector<float> ranges(msg->ranges);
	gk->updateLidar(&ranges);
}

void setup(ros::Publisher *pub,int publisherID)
{
	gk = new goalKeeper();
	gk->setROSPublisher(pub,publisherID);
}

int main(int argc, char **argv)
{
	//Initialize ROS
	ros::init(argc, argv, "lidar_node",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle lidar_node;
	ROS_WARN("Subscribing to hardwareInfo and scan topics for lidar localization");
	//Initialize hardwareInfo subscriber
	ros::Subscriber hardware_info_sub = lidar_node.subscribe("hardwareInfo", 1, hardwareInfoCallback);
	//Initialize scan subscriber
	ros::Subscriber scan_info_sub = lidar_node.subscribe("scan", 1, laserScanCallback);
	//Initialize controlInfo publisher
	//ros::Publisher control_info_pub = goalkeeper_node.advertise<controlInfo>("controlInfo", 1000);
	//Initialize goalkeeperInfo publisher
	ros::Publisher goalkeeper_info_pub = goalkeeper_node.advertise<goalkeeperInfo>("goalkeeperInfo", 1000);
	//Application Specific Objects
	setup(&goalkeeper_info_pub,0);
	ROS_WARN("MinhoTeam goalkeeper_node started running on ROS.");


	ros::AsyncSpinner spinner(2);
	spinner.start();
	return 0;
}

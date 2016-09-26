//ROS includes
#include "ros/ros.h"
//Application includes
#include <QCoreApplication>
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"

using namespace ros;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using minho_team_ros::robotInfo; //Namespace for vision information msg - PUBLISHING
//Node specific objects

void temp_callback(const hardwareInfo::ConstPtr& msg)
{
   ROS_INFO("Received stuff");
}

int main(int argc, char **argv)
{
	QCoreApplication a(argc, argv);
	ROS_WARN("Attempting to start GigE Vision services of localization_node.");

	//Initialize ROS
	ros::init(argc, argv, "localization_node",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle localization_node;
	//Initialize hardwareInfo publisher
	ros::Publisher robot_info_pub = localization_node.advertise<robotInfo>("robotInfo", 1000);
	//Initialize controlInfo subscriber
	ros::Subscriber hardware_info_sub = localization_node.subscribe("hardwareInfo", 
	                                                            1000, &temp_callback);
	                                                            
	                                                            //&hardware::controlInfoCallback,
	                                                       //     hd);
	ROS_WARN("MinhoTeam localization_node started running on ROS.");


	ros::AsyncSpinner spinner(2);
	spinner.start();
	return a.exec();
	
	return 0;
}

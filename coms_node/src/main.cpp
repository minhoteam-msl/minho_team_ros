//ROS includes
#include "ros/ros.h"
//Application includes
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"

using namespace ros;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using minho_team_ros::robotInfo; //Namespace for robot information msg - SUBSCRIBING

int main(int argc, char **argv)
{
	ROS_WARN("Attempting to start Coms services of coms_node.");
	//Initialize ROS
	ros::init(argc, argv, "coms_node",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle localization_node;
	
	ROS_WARN("MinhoTeam coms_node started running on ROS.");
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	return 0;
}

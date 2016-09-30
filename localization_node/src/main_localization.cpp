//ROS includes
#include "ros/ros.h"
//Application includes
#include <QCoreApplication>
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "localization.h"

using namespace ros;
using namespace cv;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using minho_team_ros::robotInfo; //Namespace for vision information msg - PUBLISHING

//Node specific objects
Localization *localization;
int main(int argc, char **argv)
{
	QCoreApplication a(argc, argv);
	ROS_WARN("Attempting to start GigE Vision services of localization_node.");
	//Initialize ROS
	ros::init(argc, argv, "localization_node",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle localization_node;
	
	localization = new Localization(&localization_node);

	ros::Publisher robot_info_pub = localization_node.advertise<robotInfo>("robotInfo", 1000);
	ros::Subscriber hardware_info_sub = localization_node.subscribe("hardwareInfo", 
	                                                            1000, 
	                                                            &Localization::hardwareCallback
	                                                            ,localization);
	ROS_WARN("MinhoTeam localization_node started running on ROS.");
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	return a.exec();
	return 0;
}

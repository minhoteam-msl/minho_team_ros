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
	bool use_camera = true;
	QString mode = QString::fromLocal8Bit(argv[1]);
	if(argc>1 && mode=="-static"){
	   ROS_WARN("Image acquisition set to static image."); 
	   use_camera = false;  
	} else ROS_WARN("Image acquisition set to GigE Camera.");
	bool correct_initialization = true;
	localization = new Localization(&localization_node,&correct_initialization,use_camera);
	if(!correct_initialization) { ROS_ERROR("Killing node on incorrect initialization ..."); ros::shutdown(); a.exit(0); return 0; }

	ROS_WARN("MinhoTeam localization_node started running on ROS.");
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	return a.exec();
	return 1;
}

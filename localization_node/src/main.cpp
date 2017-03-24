//ROS includes
#include "ros/ros.h"
//Application includes
#include <QCoreApplication>
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "localization.h"

//Network includes for getRobotIdByIP()
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>

using namespace ros;
using namespace cv;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using minho_team_ros::robotInfo; //Namespace for vision information msg - PUBLISHING

int getRobotIdByIP();

//Node specific objects
Localization *localization;
int main(int argc, char **argv)
{
	int side = 0;
	QCoreApplication a(argc, argv);
	ROS_WARN("Attempting to start GigE Vision services of localization_node.");
	//Initialize ROS
	ros::init(argc, argv, "localization_node",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle localization_node;
	bool use_camera = true;
	QString mode = QString::fromLocal8Bit(argv[1]);
	if(argc>2 && mode=="-static"){
	   ROS_WARN("Image acquisition set to static image.");
	   use_camera = false;
		 //side = atoi(argv[2]);
	} else ROS_WARN("Image acquisition set to GigE Camera.");
	side = 0;
	bool correct_initialization = true;
	int robot_id = 1;
	// Scan ip address for
	robot_id = getRobotIdByIP();
	if(robot_id<0) { robot_id = 1; ROS_ERROR("Error in Robot ID by IP address ... Defaulting to 1."); }
	localization = new Localization(robot_id,&localization_node,&correct_initialization,use_camera, side);
	if(!correct_initialization) { ROS_ERROR("Killing node on incorrect initialization ..."); ros::shutdown(); a.exit(0); return 0; }

	ROS_WARN("MinhoTeam localization_node started running on ROS.");

	ros::AsyncSpinner spinner(2);
	spinner.start();
	return a.exec();
	return 1;
}

int getRobotIdByIP()
{
   int	fd;
	struct ifreq if_info;
	int if_index;
   std::string ifname = "wlan0";
	memset(&if_info, 0, sizeof(if_info));
	strncpy(if_info.ifr_name, ifname.c_str(), IFNAMSIZ-1);

	if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		ROS_ERROR("Error getting UDP socket");
		return -1;
	}
	if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
	{
		ROS_ERROR("Error sending IOCTL 1");
		close(fd);
		return -1;
	}
	if_index = if_info.ifr_ifindex;

	if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
	{
		ROS_ERROR("Error sending IOCTL 2");
		close(fd);
		return -1;
	}

	close(fd);
	int id = if_info.ifr_hwaddr.sa_data[5];
   if(id<1 || id>5) id = -1;
   return id;
}

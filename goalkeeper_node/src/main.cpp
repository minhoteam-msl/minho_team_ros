//C++ includes
#include <iostream>
#include <sstream>
#include <fstream>
#include "ros/ros.h"

//ROS includes
#include "std_msgs/String.h"
//#include "kinectvision.h"
#include "minho_team_ros/hardwareInfo.h"
#include "sensor_msgs/LaserScan.h"
#include "minho_team_ros/goalKeeperInfo.h"
#include "lidarlocalization.h"
#include "Utils/rttimer.h"
#include <boost/process.hpp>

using namespace ros;
using namespace std;

using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using sensor_msgs::LaserScan; //Namespace for laserScan information msg - SUBSCRIBING
using minho_team_ros::goalKeeperInfo; //Namespace for gk info information msg - PUBLISHING

//kinectVision *gkvision;
lidarLocalization *localization;

void hardwareInfoCallback(const hardwareInfo::ConstPtr& msg)
{
	localization->updateOdometryEstimate(msg);
}

void laserScanCallback(const LaserScan::ConstPtr& msg)
{
	vector<float> ranges(msg->ranges);
	localization->updateLidarEstimate(&ranges);
}

bool checkHardwareAvailability()
{
    // detect hokuyo : ls /dev/* | grep hokuyo | wc -l
    system("ls /dev/* | grep hokuyo | wc -l > /tmp/avhokuyo.test");
    // detect kinect : lsusb | grep Xbox | wc -l 
    system("lsusb | grep Xbox | wc -l  > /tmp/avkinect.test");
    
    bool avhokstate = false;
    bool avkinstate = false;
    
    // read hokuyo output file
    ifstream avhok ("/tmp/avhokuyo.test");
    if (avhok.is_open()){
        std::string line;
        getline(avhok,line);
        if(line.size()>0 && atoi(line.c_str())>=1) avhokstate = true;
        else ROS_ERROR("Failed to find hokuyo device.");
        system("pkill -f urg_node");
        avhok.close();
    }
    
    if(avhok) { 
        std::string executable_name = "rosrun";
        std::vector<std::string> args;
        args.push_back("urg_node"); args.push_back("urg_node");
        std::string exe = boost::process::find_executable_in_path(executable_name); 
        boost::process::create_child(exe,args);
        ROS_INFO("Booted urg_node");
    }
    // read kinect output file
    ifstream avkin ("/tmp/avkinect.test");
    if (avkin.is_open()){
        std::string line;
        getline(avkin,line);
        if(line.size()>0 && atoi(line.c_str())>=2) avkinstate = true;
        else ROS_ERROR("Failed to find kinect device.");
        avkin.close();
    }

    return true;
}

int main(int argc, char **argv)
{
	//Initialize ROS
	ros::init(argc, argv, "goalkeeper_node",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle gk_node;
	ROS_WARN("Subscribing to hardwareInfo and scan topics for goalkeeper node");
	//Initialize hardwareInfo subscriber
	ros::Subscriber hardware_info_sub = gk_node.subscribe("hardwareInfo", 1, hardwareInfoCallback);
	//Initialize scan subscriber
	ros::Subscriber scan_info_sub = gk_node.subscribe("scan", 1, laserScanCallback);
	ros::Publisher gk_info_pub = gk_node.advertise<goalKeeperInfo>("goalKeeperInfo", 1);
	ros::AsyncSpinner spinner(2);
	
	goalKeeperInfo data;

	if(checkHardwareAvailability()){
        //gkvision = new kinectVision(&gk_node);
        localization = new lidarLocalization();
	    ROS_WARN("MinhoTeam goalkeeper_node started running on ROS.");
	    spinner.start();
	} else ROS_ERROR("Failed to find hardware modules for goalkeeper node");
	
	while(ros::ok()) { 
	    Point3d pose = localization->getPose();
	    
	    data.robot_info.robot_pose.x = pose.x;
	    data.robot_info.robot_pose.y = pose.y;
	    data.robot_info.robot_pose.z = pose.z;
	    
	    gk_info_pub.publish(data);
	    
	    thsleepms(30); 
	
	}
	return 0;
}

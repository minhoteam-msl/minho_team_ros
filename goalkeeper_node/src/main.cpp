//C++ includes
#include <iostream>
#include <sstream>
#include <fstream>
#include "ros/ros.h"

//ROS includes
#include "std_msgs/String.h"
#include "kinectvision.h"
#include "minho_team_ros/hardwareInfo.h"
#include "sensor_msgs/LaserScan.h"
#include "minho_team_ros/goalKeeperInfo.h"
#include "minho_team_ros/baseStationInfo.h"
#include "minho_team_ros/interestPoint.h"
#include "minho_team_ros/requestReloc.h"
#include "lidarlocalization.h"
#include "Utils/rttimer.h"
#include <boost/process.hpp>

#define DATA_UPDATE_HZ 40
#define DATA_UPDATE_USEC 1000000/DATA_UPDATE_HZ

using namespace ros;
using namespace std;

using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using sensor_msgs::LaserScan; //Namespace for laserScan information msg - SUBSCRIBING
using minho_team_ros::baseStationInfo; //Namespace for laserScan information msg - SUBSCRIBING
using minho_team_ros::goalKeeperInfo; //Namespace for gk info information msg - PUBLISHING
using minho_team_ros::interestPoint; //Namespace for gk info information msg - PUBLISHING
using minho_team_ros::requestReloc;

kinectVision *gkvision = NULL;
lidarLocalization *localization = NULL;

/// \brief main sending thread
pthread_t send_info_thread;
/// \brief periodic_info struct containing info to make
/// thread a timed function
periodic_info pi_sendth;

goalKeeperInfo keeper;
ros::Publisher gk_info_pub;

void hardwareInfoCallback(const hardwareInfo::ConstPtr& msg)
{
	if(localization)localization->updateOdometryEstimate(msg);
}

void laserScanCallback(const LaserScan::ConstPtr& msg)
{
	vector<float> ranges(msg->ranges);
	if(localization)localization->updateLidarEstimate(&ranges);
}

void baseStationCallback(const baseStationInfo::ConstPtr& msg)
{
	if(localization)localization->updateBaseStationInfo(msg);
}

bool relocCallback(requestReloc::Request &req,requestReloc::Response &res)
{
    if(localization)localization->doReloc();
    ROS_INFO("Reloc request received!");
    return true;
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
    
    pi_sendth.id = 1; pi_sendth.period_us = DATA_UPDATE_USEC;

    return true;
}

/// \brief main thread to send robot information update over UDP socket
/// \param signal - system signal for timing purposes
void* updateLocalizationData(void *data)
{
   static int counter = 0;
   periodic_info *info = (periodic_info *)(data);
   make_periodic(info->period_us,info);

   while(ros::ok()){
      if(gkvision)gkvision->updateLocalizationData(localization->getPose(),localization->getVelocities());
      wait_period(info);
   }
   return NULL;
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
	ros::Subscriber bs_info_sub = gk_node.subscribe("/basestation/baseStationInfo", 1, baseStationCallback);
	gk_info_pub = gk_node.advertise<goalKeeperInfo>("goalKeeperInfo", 1);
	ros::ServiceServer reloc_service = gk_node.advertiseService("requestReloc",
                                    relocCallback);

	ros::AsyncSpinner spinner(2);

	ROS_WARN("Checking hardware modules ...");
	if(checkHardwareAvailability()){
        //gkvision = new kinectVision(&gk_node);
        localization = new lidarLocalization();
	    ROS_WARN("MinhoTeam goalkeeper_node started running on ROS.");
	    spinner.start();
	} else ROS_ERROR("Failed to find hardware modules for goalkeeper node");
	
	pi_sendth.id = 1; pi_sendth.period_us = DATA_UPDATE_USEC;
    pthread_create(&send_info_thread, NULL, updateLocalizationData,&pi_sendth);
	pthread_join(send_info_thread,NULL);
	
	return 0;
}

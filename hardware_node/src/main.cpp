//ROS includes
#include "ros/ros.h"
//Application includes
#include "hardware.h"
#include <QCoreApplication>

using namespace std;
using namespace ros;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - PUBLISHING
using minho_team_ros::controlInfo; //Namespace for control information msg - SUBSCRIBING
using minho_team_ros::teleop; //Namespace for teleop msg - SUBSCRIBING

//Node specific objects
hardware *hd;
QSerialPort *serial;
int main(int argc, char **argv)
{
	QCoreApplication a(argc, argv);
	/* This program interfaces MinhoTeam's hardware via QSerialPort
	 * and uses ROS to communicate with the remaining nodes.     */
	ROS_WARN("Attempting to start Serial Port services of hardware_node.");
	//Hardware Serial Port Communication Object Initialization
	hd = new hardware(&a);
	//Initialize ROS Node with name hardware_node
	ros::init(argc, argv, "hardware_node",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle hardware_node;
	//Initialize hardwareInfo publisher and pass it to child class
	ros::Publisher hardware_info_pub = hardware_node.advertise<hardwareInfo>("hardwareInfo", 1000);
	hd->setROSPublisher(&hardware_info_pub);
	//Initialize controlInfo subscriber
	ros::Subscriber control_info_sub = hardware_node.subscribe("controlInfo", 
	                                                            1000, 
	                                                            &hardware::controlInfoCallback,
	                                                            hd);
	//Initialize teleop subscriber                                                            
	ros::Subscriber teleop_sub = hardware_node.subscribe("teleop", 
                                                         1000, 
                                                         &hardware::teleopCallback,
                                                         hd);
	ROS_WARN("MinhoTeam hardware_node started running on ROS.");


   //Run ROS AsyncSpinner
	ros::AsyncSpinner spinner(2);
	spinner.start();
	return a.exec();
	
	return 0;
}

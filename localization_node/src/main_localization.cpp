//ROS includes
#include "ros/ros.h"
//Application includes
#include <QCoreApplication>
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"

/////////////////////// TEST
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
///////////////////////


using namespace ros;
using namespace cv;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using minho_team_ros::robotInfo; //Namespace for vision information msg - PUBLISHING
//Node specific objects

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
	/*ros::Subscriber hardware_info_sub = localization_node.subscribe("hardwareInfo", 
	                                                            1000, 
	                                                            &CLASS::CALLBACK
	                                                            CLASS);*/
	ROS_WARN("MinhoTeam localization_node started running on ROS.");


   /////////////////////// TEST
   /*Mat test_image;
   test_image = imread("/home/pedro/catkin_ws/src/minho_team_ros/localization_node/config/sample.png");
   ros::Rate loop_rate(2);
   image_transport::ImageTransport it(localization_node);
   image_transport::Publisher pub = it.advertise("camera", 1);
   sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", test_image).toImageMsg();
  
	while(ros::ok()){
	   ROS_INFO("Sending Image");
	   pub.publish(msg);
	   ros::spinOnce();
	   loop_rate.sleep();
	}
	   ///////////////////////*/
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	return a.exec();
	return 0;
}

#include <iostream>
//ROS includes
#include "ros/ros.h"
#include "minho_team_ros/controlInfo.h"
using minho_team_ros::controlInfo; //Namespace for control information msg - PUBLISHING

// Test program to send the robot in square motion
controlInfo msg;
void switchMove(int *id);
int main(int argc, char **argv)
{
   ROS_WARN("Attempting to start hardware_test_node");   
   ros::init(argc, argv, "hw_test_node",ros::init_options::NoSigintHandler);
   ros::NodeHandle nh;
   ros::Publisher control_info_pub = nh.advertise<controlInfo>("controlInfo", 100);
   msg.linear_velocity = 30;
   double tempo = (100.0/1000.0); // em segundos
   ros::Rate loop_rate(1.0/tempo);
   int counter = 0;
   int moveId = 0;
   while(ros::ok()){
      control_info_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
      if(counter>30) { counter = 0; moveId++; switchMove(&moveId); }
      else counter++;
   }
}

void switchMove(int *id)
{
   if((*id)>3) *id = 0;
   
   switch((*id)){
      case 0:{
         msg.movement_direction = 0;
         break;
      }
      case 1:{
         msg.movement_direction = 90;
         break;
      }
      case 2:{
         msg.movement_direction = 180;
         break;
      }
      case 3:{
         msg.movement_direction = 270;
         break;
      }
   }
}

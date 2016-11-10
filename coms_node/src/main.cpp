//ROS includes
#include "ros/ros.h"
//Application includes
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/goalKeeperInfo.h"
#include "minho_team_ros/interAgentInfo.h"
#include <iostream>
#include <string.h>
#include <sstream>

using namespace ros;
using minho_team_ros::hardwareInfo; //Namespace for hardwareInfo msg - SUBSCRIBING
using minho_team_ros::robotInfo; //Namespace for robotInfo msg - SUBSCRIBING
using minho_team_ros::goalKeeperInfo; //Namespace for goalKeeperInfo msg - SUBSCRIBING
using minho_team_ros::interAgentInfo; //Namespace for interAgentInfo msg - SENDING OVER UDP/SUBSCRIBING OVER UDP

// ###### GLOBAL DATA ######
// \brief subscriber for hardwareInfo message
ros::Subscriber hw_sub;
// \brief subscriber for robotInfo message
ros::Subscriber robot_sub;
// \brief subscriber for goalKeeperInfo message
ros::Subscriber gk_sub;

/// \brief variable to build the topic name for hardwareInfo message
std::stringstream hw_topic_name;
/// \brief variable to build the topic name for robotInfo message
std::stringstream robot_topic_name;
/// \brief variable to build the topic name for goalKeeperInfo message
std::stringstream gk_topic_name;

/// \brief message to store most recent hardwareInfo message received
/// and to store the most recent goalKeeperInfo or robotInfo received. 
/// Acts as dual purpose message container, as interAgentInfo
/// which will be serialized contains hardwareInfo and goalKeeperInfo
/// messages, to allow both goalkeeper and non goalkeeper agents to use
/// the same data structures
interAgentInfo message;

// #########################

// ######## THREADS ########
// #########################

// ### FUNCTION HEADERS ####
/// \brief callback function of hardwareInfo messages/topic of ROS
/// \param msg - message containing received data of hardwareInfo topic
void hardwareInfoCallback(const hardwareInfo::ConstPtr &msg);

/// \brief callback function of robotInfo messages/topic of ROS
/// \param msg - message containing received data of robotInfo topic
void robotInfoCallback(const robotInfo::ConstPtr &msg);

/// \brief callback function of goalKeeperInfo messages/topic of ROS
/// \param msg - message containing received data of goalKeeperInfo topic
void goalKeeperInfoCallback(const goalKeeperInfo::ConstPtr &msg);

/// \brief serializes an interAgentInfo message to send over UDP as a string
/// of bytes.
/// \param msg - pointer to interAgentInfo object to be serialized
/// \param packet - uint8_t vector containing the serialized message
void serializeInterAgentInfo(interAgentInfo *msg, uint8_t *packet);

/// \brief deserializes a string of bytes into a interAgentInfo message
/// \param packet - uint8_t vector containing message to deserialize
/// \param msg - pointer to interAgentInfo destination object
void deserializeInterAgentInfo(uint8_t *packet, interAgentInfo *msg);


boost::shared_array<uint8_t> buffer;
// #########################


/// \brief this node acts as a bridge between the other agents (robots and base
/// station) and each robot's ROS. It uses UDP broadcast transmission and sends
/// serialized ROS messages.

int main(int argc, char **argv)
{
	ROS_WARN("Attempting to start Coms services of coms_node.");
	//Initialize ROS
	ros::init(argc, argv, "coms_node",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle coms_node;
	
	//Setup sockets
	
	
	//Setup ROS
	hw_topic_name << "/hardwareInfo";
	robot_topic_name << "/robotInfo";
	gk_topic_name << "/goalKeeperInfo";
	
	hw_sub = coms_node.subscribe(hw_topic_name.str().c_str(),
	                             1,&hardwareInfoCallback);
	robot_sub = coms_node.subscribe(robot_topic_name.str().c_str(),
	                             1,&robotInfoCallback);
	gk_sub = coms_node.subscribe(gk_topic_name.str().c_str(),
	                             1,&goalKeeperInfoCallback);
	//Setup Threads
	
	
	
	
	ROS_WARN("MinhoTeam coms_node started running on ROS.");
	// Run spinner
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	
	// Wait threads to join
	uint8_t *packet;
	while(ros::ok()){ 
	   ROS_INFO("IN: %d",message.is_goalkeeper);
	   serializeInterAgentInfo(&message,packet);
	   deserializeInterAgentInfo(packet,&message);
	}
	return 0;
}

// ###### FUNCTIONS ########
/// \brief callback function of hardwareInfo messages/topic of ROS
/// \param msg - message containing received data of hardwareInfo topic
void hardwareInfoCallback(const hardwareInfo::ConstPtr &msg)
{
   //call mutex
   message.hardware_info = (*msg);
}

/// \brief callback function of robotInfo messages/topic of ROS
/// Sets the robotInfo portion of goalKeeperInfo message contained
/// within interAgentInfo. Sets is_goalkeeper flag to false
/// \param msg - message containing received data of robotInfo topic
void robotInfoCallback(const robotInfo::ConstPtr &msg)
{
   //call mutex
   message.agent_info.robot_info = (*msg); 
   message.is_goalkeeper = false;  
}

/// \brief callback function of goalKeeperInfo messages/topic of ROS
/// Sets the goalKeeperInfo within interAgentInfo. 
/// Sets is_goalkeeper flag to true
/// \param msg - message containing received data of goalKeeperInfo topic
void goalKeeperInfoCallback(const goalKeeperInfo::ConstPtr &msg)
{
   //call mutex
   message.agent_info = (*msg);  
   message.is_goalkeeper = true;  
}

/// \brief serializes an interAgentInfo message to send over UDP as a string
/// of bytes.
/// \param msg - pointer to interAgentInfo object to be serialized
/// \param packet - uint8_t vector containing the serialized message
void serializeInterAgentInfo(interAgentInfo *msg, uint8_t *packet)
{  
   msg->is_goalkeeper = true;
   uint32_t serial_size = ros::serialization::serializationLength( *msg );
   buffer.reset(new uint8_t[serial_size]);
   ros::serialization::OStream stream( buffer.get(), serial_size );
   ros::serialization::serialize( stream, *msg );
   packet = buffer.get();
}

/// \brief deserializes a string of bytes into a interAgentInfo message
/// \param packet - uint8_t vector containing message to deserialize
/// \param msg - pointer to interAgentInfo destination object
void deserializeInterAgentInfo(uint8_t *packet, interAgentInfo *msg)
{  
   interAgentInfo temp;
   uint32_t serial_size = 105;
   ros::serialization::IStream i_stream(packet, serial_size);
   ros::serialization::deserialize(i_stream, temp);
   
   ROS_INFO("OUT: %d",temp.is_goalkeeper);
   msg->is_goalkeeper = false;
}
// #########################

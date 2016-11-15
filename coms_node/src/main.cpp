//ROS includes
#include "ros/ros.h"
//Application includes
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/goalKeeperInfo.h"
#include "minho_team_ros/interAgentInfo.h"
#include "minho_team_ros/position.h"
#include <iostream>
#include <string.h>
#include <sstream>
#include <pthread.h>
#include <semaphore.h>
#include <time.h>
//defines for signal
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include<arpa/inet.h>
#include<sys/socket.h>
#include "thpool.h"
#include "multicast.h"

#define BUFFER_SIZE 1024
#define DATA_UPDATE_HZ 33
#define DATA_UPDATE_USEC 1000000/DATA_UPDATE_HZ
#define MAX_DELAY_TIME_US 5000

/// \brief struct to represet a udp packet, containing
/// a serialized ROS message
typedef struct udp_packet{
   uint8_t *packet;
   uint32_t packet_size;
}udp_packet;

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
/// \brief variable to build the node name
std::stringstream node_name;

/// \brief message to store most recent hardwareInfo message received
/// and to store the most recent goalKeeperInfo or robotInfo received. 
/// Acts as dual purpose message container, as interAgentInfo
/// which will be serialized contains hardwareInfo and goalKeeperInfo
/// messages, to allow both goalkeeper and non goalkeeper agents to use
/// the same data structures
interAgentInfo message;

/// \brief shared_array of uint8 (unsigned char) used in message 
/// deserialization
boost::shared_array<uint8_t> serialization_buffer;
// #########################


// ###### SOCKET DATA#######
/// \brief UDP socket descriptor
int socket_fd;
/// \brief received datagram size variable
int recvlen = 0;
/// \brief used ip address in multicast interface : has to be "172.16.49.X"
std::string ip_base; 
/// \brief agent id, based on ip. ip_base.agent_id build up the ip address
/// of the machine. for oficial robots this ranges from 1 to 6, basestation
/// is 7 and the above are dummy id's
uint8_t agent_id;
// #########################


// ###### THREAD DATA ######
/// \brief a mutex to avoid multi-thread access to message
pthread_mutex_t message_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief a mutex to avoid multi-thread access to socket
pthread_mutex_t socket_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief thread pool to allow dynamic task assignment
threadpool thpool_t;
///  \brief buffer to hold received data
uint8_t buffer[BUFFER_SIZE];
/// \brief main udp data receiving thread
pthread_t recv_monitor_thread;
/// \brief struct to specify timer options for sending thread
struct itimerval timer;
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

/// \brief serializes a message to send over UDP as a string
/// of bytes.
/// \typename Message - type of ROS mesasge to serialize
/// \param msg - pointer to interAgentInfo object to be serialized
/// \param packet - uint8_t vector containing the serialized message
/// \param packet_size - size of the generated packet
/// WARN : ONLY FOR INTERAGENT MESSAGES (SENT WITH UDP)
template<typename Message>
void serializeROSMessage(Message *msg, uint8_t **packet, uint32_t *packet_size);

/// \brief deserializes a string of bytes into a message
/// \typename Message - type of ROS mesasge to deserialize
/// \param packet - uint8_t vector containing message to deserialize
/// \param msg - pointer to interAgentInfo destination object
/// WARN : ONLY FOR INTERAGENT MESSAGES (SENT WITH UDP)
template<typename Message>
void deserializeROSMessage(udp_packet *packet, void *msg);

/// \brief main thread to send robot information update over UDP socket
/// \param signal - system signal for timing purposes
static void sendRobotInformationUpdate(int signal);

/// \brief wrapper function for error and exit
/// \param msg - message to print error
void die(std::string msg);

/// \brief initializes multicast socket for sending/receiveing information to 
/// multicast address
void setupMultiCastSocket();

/// \brief main udp receiving thread. This thread deals with datagram reception
/// and sends the received data to be dealt by a thread using threadpool
/// \param socket - pointer to socket descriptor to use 
void* udpReceivingThread(void *socket);

/// \brief function used by threadpool threads to process incoming data
/// sent by other agents. Thread returns to pool after doing its work
/// \param packet - data packet to be deserialized into a ROS message
void processReceivedData(void *packet);
// #########################


/// \brief this node acts as a bridge between the other agents (robots and base
/// station) and each robot's ROS. It uses UDP broadcast transmission and sends
/// serialized ROS messages.
int main(int argc, char **argv)
{
   //Setup robotid and mode
   // #########################
   if(argc>1 && argc!=3) { 
      ROS_ERROR("Must enter robot id and mode as parameter for the simulated robot.\n \
                Please use -s for simulation, followed by the robot's ID.");
      exit(1); 
   }
   
   bool mode_real = true;
   int robot_id = 0;
   if(argc==3){
      robot_id = atoi(argv[2]);
      if(robot_id<0 || robot_id>6){
         ROS_ERROR("Must enter robot id correctly. Robot id's range from 1 to 6 and 0 to localhost.");
         exit(2);     
      }
      if(!strcmp(argv[1],"-s")) mode_real = false;
      else { 
         ROS_ERROR("Must enter mode correctly. Please use -s for simulation.");
         exit(3); 
      }
   }
   
   ROS_WARN("Attempting to start Coms services of coms_node.");
   node_name << "coms_node";
   // #########################
   
   //Setup Sockets
   // #########################
   setupMultiCastSocket();  
   srand(time(NULL)); 
	// #########################
	
	if(mode_real) { robot_id = agent_id; ROS_INFO("Running coms_node for Robot %d.",robot_id); }
   else { 
      node_name << robot_id;
      ROS_INFO("Running coms_node for Robot %d in simulation.",robot_id);
   }
	
	//Initialize ROS
	// #########################
	ros::init(argc, argv, node_name.str().c_str(),ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle coms_node;
	//Setup ROS
	if(!mode_real){
	   hw_topic_name << "/minho_gazebo_robot" << robot_id;
	   robot_topic_name << "/minho_gazebo_robot" << robot_id;
	   gk_topic_name << "/minho_gazebo_robot" << robot_id;
	}
	
	hw_topic_name << "/hardwareInfo";
	robot_topic_name << "/robotInfo";
	gk_topic_name << "/goalKeeperInfo";
	
	hw_sub = coms_node.subscribe(hw_topic_name.str().c_str(),
	                             1,&hardwareInfoCallback);
	robot_sub = coms_node.subscribe(robot_topic_name.str().c_str(),
	                             1,&robotInfoCallback);
	gk_sub = coms_node.subscribe(gk_topic_name.str().c_str(),
	                             1,&goalKeeperInfoCallback);
	message.agent_id = robot_id;                             
   // #########################
   
	//Setup Updated timer thread
	// #########################
	signal(SIGALRM,sendRobotInformationUpdate);
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = DATA_UPDATE_USEC;
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = DATA_UPDATE_USEC;
	// #########################
	
	//Setup Thread pool and rece
	// iving thread
	// #########################
	thpool_t = thpool_init(6); //5 threads per agent ?
	pthread_create(&recv_monitor_thread, NULL, udpReceivingThread, &socket_fd);
	// #########################
	
	// Run functions and join threads
	// #########################
	ROS_WARN("MinhoTeam coms_node started running on ROS.");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	setitimer (ITIMER_REAL, &timer, NULL);
	while(ros::ok());
	pthread_join(recv_monitor_thread, NULL);
	thpool_destroy(thpool_t);
	closeSocket(socket_fd);
   pthread_exit(NULL);
	// #########################
	
	return 0;
}

// ###### FUNCTIONS ########
/// \brief callback function of hardwareInfo messages/topic of ROS
/// \param msg - message containing received data of hardwareInfo topic
void hardwareInfoCallback(const hardwareInfo::ConstPtr &msg)
{
   pthread_mutex_lock (&message_mutex); //Lock mutex
   message.hardware_info = (*msg);
   pthread_mutex_unlock (&message_mutex); //Unlock mutex
}

/// \brief callback function of robotInfo messages/topic of ROS
/// Sets the robotInfo portion of goalKeeperInfo message contained
/// within interAgentInfo. Sets is_goalkeeper flag to false
/// \param msg - message containing received data of robotInfo topic
void robotInfoCallback(const robotInfo::ConstPtr &msg)
{
   pthread_mutex_lock (&message_mutex); //Lock mutex
   message.agent_info.robot_info = (*msg); 
   message.is_goalkeeper = false;  
   pthread_mutex_unlock (&message_mutex); //Unlock mutex
}

/// \brief callback function of goalKeeperInfo messages/topic of ROS
/// Sets the goalKeeperInfo within interAgentInfo. 
/// Sets is_goalkeeper flag to true
/// \param msg - message containing received data of goalKeeperInfo topic
void goalKeeperInfoCallback(const goalKeeperInfo::ConstPtr &msg)
{
   pthread_mutex_lock (&message_mutex); //Lock mutex
   message.agent_info = (*msg);  
   message.is_goalkeeper = true;  
   pthread_mutex_unlock (&message_mutex); //Unlock mutex
}

/// \brief serializes a message to send over UDP as a string
/// of bytes.
/// \typename Message - type of ROS mesasge to serialize
/// \param msg - pointer to interAgentInfo object to be serialized
/// \param packet - uint8_t vector containing the serialized message
/// \param packet_size - size of the generated packet
/// WARN : ONLY FOR INTERAGENT MESSAGES (SENT WITH UDP)
template<typename Message>
void serializeROSMessage(Message *msg, uint8_t **packet,  uint32_t *packet_size)
{  
   pthread_mutex_lock (&message_mutex); //Lock mutex
   uint32_t serial_size = ros::serialization::serializationLength( *msg );
   serialization_buffer.reset(new uint8_t[serial_size]);
   (*packet_size) = serial_size;
   ros::serialization::OStream stream( serialization_buffer.get(), serial_size );
   ros::serialization::serialize( stream, *msg);
   (*packet) = serialization_buffer.get();
	pthread_mutex_unlock (&message_mutex); //Unlock mutex
}

/// \brief deserializes a string of bytes into a message
/// \typename Message - type of ROS mesasge to deserialize
/// \param packet - uint8_t vector containing message to deserialize
/// \param msg - pointer to interAgentInfo destination object
/// WARN : ONLY FOR INTERAGENT MESSAGES (SENT WITH UDP)
template<typename Message>
void deserializeROSMessage(udp_packet *packet, Message *msg)
{  
   ros::serialization::IStream istream(packet->packet, packet->packet_size);
   ros::serialization::deserialize(istream, *msg);
}

/// \brief main thread to send robot information update over UDP socket
/// \param signal - system signal for timing purposes
static void sendRobotInformationUpdate(int signal)
{
   if(signal==SIGALRM){ // Takes 0.08ms to do
      uint8_t *packet;
      uint32_t packet_size;
      serializeROSMessage<interAgentInfo>(&message,&packet,&packet_size);
      // Send packet of size packet_size through UDP
      if (sendData(socket_fd,packet,packet_size) <= 0){
         die("Failed to send a packet.");
      } 
	}
}

/// \brief wrapper function for error and exit
/// \param msg - message to print error
void die(std::string msg) {
    ROS_ERROR("%s",msg.c_str());
    exit(0);
}

/// \brief initializes multicast socket for sending information to 
/// multicast address
void setupMultiCastSocket()
{
   socket_fd = openSocket("wlan0",&ip_base,&agent_id);
   if(socket_fd<0) exit(0);
   ROS_INFO("UDP Multicast System started.");
}

/// \brief main udp receiving thread. This thread deals with datagram reception
/// and sends the received data to be dealt by a thread using threadpool
/// \param socket - pointer to socket descriptor to use 
void* udpReceivingThread(void *socket)
{
   while(ros::ok()){
      bzero(buffer, BUFFER_SIZE);   
      if((recvlen = recv(*(int*)socket, buffer, BUFFER_SIZE,0)) > 0 ){
         udp_packet *relay_packet = new udp_packet;
         relay_packet->packet = new uint8_t[recvlen];
         relay_packet->packet_size = recvlen;
         memcpy(relay_packet->packet,buffer,recvlen);
         thpool_add_work(thpool_t, processReceivedData, relay_packet);
      }
   }
   
   return NULL;
}

/// \brief function used by threadpool threads to process incoming data
/// sent by other agents. Thread returns to pool after doing its work
/// \param packet - data packet to be deserialized into a ROS message
void processReceivedData(void *packet)
{
   // deserialize message
   interAgentInfo incoming_data;
   deserializeROSMessage<interAgentInfo>((udp_packet *)packet,&incoming_data); 
   delete((udp_packet *)packet);
   //work out stuff here
   /*ROS_INFO("Robot %d : %.2f %.2f %.2f", incoming_data.agent_id,
                             incoming_data.agent_info.robot_info.robot_pose.x,
                             incoming_data.agent_info.robot_info.robot_pose.y,
                             incoming_data.agent_info.robot_info.robot_pose.z);*/
   return;
           
}
// #########################

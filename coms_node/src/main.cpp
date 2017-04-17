//ROS includes
#include "ros/ros.h"
//Application includes
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/goalKeeperInfo.h"
#include "minho_team_ros/interAgentInfo.h"
#include "minho_team_ros/position.h"
#include "minho_team_ros/baseStationInfo.h"
#include "minho_team_ros/requestReloc.h"
#include "minho_team_ros/requestResetIMU.h"
#include "std_msgs/UInt8.h"
#include <iostream>
#include <string.h>
#include <sstream>
#include "Utils/rttimer.h"
#include "thpool.h"
#include "multicast.h"
#include "ros/topic_manager.h"
#include <boost/filesystem.hpp>
#include <boost/process.hpp>

#define BUFFER_SIZE 5120
#define DATA_UPDATE_HZ 30
#define DATA_UPDATE_USEC 1000000/DATA_UPDATE_HZ
#define MAX_DELAY_TIME_US 5000
#define NUM_ROBOT_AGENTS 5
#define TOTAL_AGENTS NUM_ROBOT_AGENTS+1

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
using minho_team_ros::baseStationInfo; //Namespace for baseStationInfo msg - SENDING OVER UDP/SUBSCRIBING OVER UDP
using std_msgs::UInt8;
using minho_team_ros::requestReloc; // Namespace for requestReloc service
using minho_team_ros::requestResetIMU; // Namespace for requestResetIMU service

// ###### GLOBAL DATA ######
// \brief subscriber for hardwareInfo message
ros::Subscriber hw_sub;
// \brief subscriber for robotInfo message
ros::Subscriber robot_sub;
// \brief subscriber for goalKeeperInfo message
ros::Subscriber gk_sub;
/// \brief requestResetIMU ROS Service Client
ros::ServiceClient resetIMUService;
/// \brief requestReloc ROS Service Client
ros::ServiceClient requestRelocService;

/// \brief variable to build the topic name for hardwareInfo message
std::stringstream hw_topic_name;
/// \brief variable to build the topic name for robotInfo message
std::stringstream robot_topic_name;
/// \brief variable to build the topic name for goalKeeperInfo message
std::stringstream gk_topic_name;
/// \brief variable to build the node name
std::stringstream node_name;
/// \brief variables to build service names
std::stringstream reloc_name, reset_name;

/// \brief vector of publishers for all 6 agents of the game
ros::Publisher publishers[TOTAL_AGENTS];
/// \brief vector counting the number of subscribers for all
/// \6 published topics by the coms_node
uint8_t num_subscribers[TOTAL_AGENTS] = {0,0,0,0,0,0};

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
/// \brief a mutex to avoid multi-thread access to ROS publishers
pthread_mutex_t publishers_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief thread pool to allow dynamic task assignment
threadpool thpool_t;
///  \brief buffer to hold received data
uint8_t buffer[BUFFER_SIZE];
/// \brief main udp data receiving thread
pthread_t recv_monitor_thread;
/// \brief main sending thread
pthread_t send_info_thread;
/// \brief periodic_info struct containing info to make
/// thread a timed function
periodic_info pi_sendth;
/// \brief int variable to control thread running
volatile bool _thrun = true;
/// \brief auxilizar variables to test connection of gzserver
std::string pgrep;
std::vector<std::string> pgrep_args;
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
void* sendRobotInformationUpdate(void *data);

/// \brief checks if Gzserver is running in case of simulated robots
bool checkGzserverConnection();

/// \brief wrapper function for error and exit
/// \param msg - message to print error
void error(std::string msg);

/// \brief initializes multicast socket for sending/receiveing information to 
/// multicast address
/// \param receive_own_packets - boolean value to define whether the robot 
/// receives or not its own packets (for simulated robots)
void setupMultiCastSocket(uint8_t receive_own_packets);

/// \brief main udp receiving thread. This thread deals with datagram reception
/// and sends the received data to be dealt by a thread using threadpool
/// \param socket - pointer to socket descriptor to use 
void* udpReceivingThread(void *socket);

/// \brief function used by threadpool threads to process incoming data
/// sent by other agents. Thread returns to pool after doing its work
/// \param packet - data packet to be deserialized into a ROS message
void processReceivedData(void *packet);

/// \brief gets syncronization agent id's for this agent. The primary
/// syncronizator is id-1
int getSyncId() {if(agent_id==1) return 6; else return agent_id-1;}

/// \brief holds state of robot sync (if inside sync ring or not)
bool syncState[NUM_ROBOT_AGENTS+1];
/// \brief holds state of agent
bool agentState[NUM_ROBOT_AGENTS+1];
// #########################


bool mode_real = true;

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
   
   srand(time(NULL));
   int robot_id = 0;
   if(argc==3){
      robot_id = atoi(argv[2]);
      if(robot_id<1 || robot_id>5){
         ROS_ERROR("Must enter robot id correctly. Robot id's range from 1 to 5.");
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
   int receive_own_packets = 0;
   if(!mode_real) receive_own_packets = 1;
   
   //Setup Sockets
   // #########################
   setupMultiCastSocket(receive_own_packets);        
   srand(time(NULL)); 
	// #########################
   
   if(mode_real){ 
      if(agent_id<=0) { agent_id = 1; ROS_ERROR("Error in Robot ID by IP address ... Defaulting to 1."); }
      ROS_INFO("Running coms_node for Robot %d.",agent_id);
   }else { 
      agent_id = robot_id;
      node_name << (int)robot_id << std::time(0);
      ROS_INFO("Running coms_node for Robot %d in simulation.",agent_id);
   }
   // #########################
	
	//Initialize ROS
	// #########################
	ros::init(argc, argv, node_name.str().c_str(),ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle coms_node;
	std::stringstream base_relay_topic;
	//Setup ROS
	message.msg_id = 1;
	if(!mode_real){
	   hw_topic_name << "/minho_gazebo_robot" << (int)agent_id;
	   robot_topic_name << "/minho_gazebo_robot" << (int)agent_id;
	   gk_topic_name << "/minho_gazebo_robot" << (int)agent_id;
	   base_relay_topic << "/minho_gazebo_robot" << (int)agent_id;
	   reloc_name << "/minho_gazebo_robot" << (int)agent_id;
	   reset_name << "/minho_gazebo_robot" << (int)agent_id;
	}
	
	hw_topic_name << "/hardwareInfo";
	robot_topic_name << "/robotInfo";
	gk_topic_name << "/goalKeeperInfo";
	reloc_name << "/requestReloc";
	reset_name << "/requestResetIMU";
	
	std::string relay_topic_name = "/interAgentInfo";
    std::string relay_topic_name_bs = "/baseStationInfo";
	   
	
	hw_sub = coms_node.subscribe(hw_topic_name.str().c_str(),
	                             1,&hardwareInfoCallback);
	robot_sub = coms_node.subscribe(robot_topic_name.str().c_str(),
	                             1,&robotInfoCallback);
	gk_sub = coms_node.subscribe(gk_topic_name.str().c_str(),
	                             1,&goalKeeperInfoCallback);
    resetIMUService = coms_node.serviceClient<minho_team_ros::requestResetIMU>(reset_name.str().c_str());
    
    requestRelocService = coms_node.serviceClient<minho_team_ros::requestReloc>(reloc_name.str().c_str());
    
	message.agent_id = agent_id;   
	// for robot agents (1 to NUM_ROBOT_AGENTS) -> 0 to NUM_ROBOT_AGENTS-1
   for(int a = 0; a < NUM_ROBOT_AGENTS; a++){
      std::stringstream rl_agent_topic;
      rl_agent_topic << base_relay_topic.str() << "/agent" << a+1 << relay_topic_name;
      publishers[a] = coms_node.advertise<interAgentInfo>(rl_agent_topic.str().c_str(),1);
   }
   // for base station agent (NUM_ROBOT_AGENTS+1) -> NUM_ROBOT_AGENTS
   std::stringstream rl_agent_topic;
   rl_agent_topic << base_relay_topic.str() << "/basestation" << relay_topic_name_bs;
   publishers[NUM_ROBOT_AGENTS] = coms_node.advertise<baseStationInfo>(rl_agent_topic.str().c_str(),1);
   // #########################
	
	if(!mode_real){ //check for connection with gzserver
      pgrep = boost::process::find_executable_in_path("pgrep");
      pgrep_args.push_back("-c");
      pgrep_args.push_back("gzserver");
	}
	// #########################
	
	//Setup Thread pool and rece
	// iving thread
	// #########################
   pi_sendth.id = 1; pi_sendth.period_us = DATA_UPDATE_USEC;
	thpool_t = thpool_init(TOTAL_AGENTS); //5 threads per agent ?
	pthread_create(&recv_monitor_thread, NULL, udpReceivingThread, &socket_fd);
   pthread_create(&send_info_thread, NULL, sendRobotInformationUpdate,&pi_sendth);
	// #########################
	
	// Run functions and join threads
	// #########################
	ROS_WARN("MinhoTeam coms_node started running on ROS.");
	ros::AsyncSpinner spinner(2);
	spinner.start();
   pthread_join(send_info_thread, NULL);
   pthread_cancel(recv_monitor_thread);
   pthread_join(recv_monitor_thread, NULL);
   thpool_wait(thpool_t);
	thpool_destroy(thpool_t);
	closeSocket(socket_fd);
   ROS_ERROR("Exited coms_node");
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
void* sendRobotInformationUpdate(void *data)
{
   static int counter = 0;
   periodic_info *info = (periodic_info *)(data);
   make_periodic(info->period_us,info);

   while(ros::ok()){
      if(counter<30) counter++; else { 
         //if(!checkGzserverConnection()) break; 
         counter = 0; 
      }
      uint8_t *packet;
      uint32_t packet_size;
      message.msg_id = 1;
      serializeROSMessage<interAgentInfo>(&message,&packet,&packet_size);
      // Send packet of size packet_size through UDP
      if (sendData(socket_fd,packet,packet_size) <= 0){
         error("Failed to send a packet.");
      }  

      for(int a = 0; a < TOTAL_AGENTS; a++){
         pthread_mutex_lock(&publishers_mutex); //Lock mutex
         num_subscribers[a] = publishers[a].getNumSubscribers();
         pthread_mutex_unlock(&publishers_mutex); //Unlock mutex
      }  

      thsleepms(rand()%4);
      wait_period(info);
   }
   return NULL;
}

/// \brief wrapper function for error and exit
/// \param msg - message to print error
void error(std::string msg) {
    ROS_ERROR("%s",msg.c_str());
}

/// \brief initializes multicast socket for sending information to 
/// multicast address
/// \param receive_own_packets - boolean value to define whether the robot 
/// receives or not its own packets (for simulated robots)
void setupMultiCastSocket(uint8_t receive_own_packets)
{
   socket_fd = openSocket("wlan0",&ip_base,&agent_id,receive_own_packets);
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
      if(!_thrun){ break;} 
   }

   return NULL;
}

/// \brief function used by threadpool threads to process incoming data
/// sent by other agents. Thread returns to pool after doing its work
/// \param packet - data packet to be deserialized into a ROS message
void processReceivedData(void *packet)
{
   if(!_thrun) return;
   // deserialize message
   udp_packet *temp = (udp_packet *)packet;
    UInt8 msg;
    ros::serialization::IStream istream(temp->packet, sizeof(msg.data));
    ros::serialization::deserialize(istream, msg);
    if(msg.data==1){
        interAgentInfo incoming_data;
        deserializeROSMessage<interAgentInfo>((udp_packet *)packet,&incoming_data); 
        delete((udp_packet *)packet);
        if(incoming_data.agent_id==agent_id) return;
        // Publish to matching topic
        if(incoming_data.agent_id>=1 && incoming_data.agent_id<=TOTAL_AGENTS){
        pthread_mutex_lock(&publishers_mutex); //Lock mutex
        if(num_subscribers[incoming_data.agent_id-1]>0){
            publishers[incoming_data.agent_id-1].publish(incoming_data);
        }
        pthread_mutex_unlock(&publishers_mutex); //Unlock mutex
        }
    } else {
        //comes from basestation
        baseStationInfo incoming_data;
        deserializeROSMessage<baseStationInfo>((udp_packet *)packet,&incoming_data); 
        delete((udp_packet *)packet);
        if(incoming_data.agent_id==agent_id) return;
        // Publish to matching topic
        if(incoming_data.agent_id>=1 && incoming_data.agent_id<=TOTAL_AGENTS){
            pthread_mutex_lock(&publishers_mutex); //Lock mutex
            if(num_subscribers[incoming_data.agent_id-1]>0){
                publishers[incoming_data.agent_id-1].publish(incoming_data);
            }
            pthread_mutex_unlock(&publishers_mutex); //Unlock mutex
        
            //check requests from base station
            int requestsAgent = incoming_data.requests[agent_id-1];

            if(requestsAgent==1){
                ROS_ERROR("Reloc request received");
                requestReloc srv;
                requestRelocService.call(srv);
            } else if(requestsAgent==2){
                ROS_ERROR("Reset IMU request received");
                requestResetIMU srv;
                resetIMUService.call(srv);
            }
        }
    }
}

/// \brief checks if Gzserver is running in case of simulated robots
/// \param signal - system signal for timing purposes
bool checkGzserverConnection()
{
   if(!mode_real){
      boost::process::context ctx; 
      ctx.streams[boost::process::stdout_id] = boost::process::behavior::pipe(); 
      boost::process::child c = boost::process::create_child(pgrep, pgrep_args, ctx); 
      boost::process::pistream is(c.get_handle(boost::process::stdout_id)); 
      char output[2];
      is.read(output,2);
      c.wait();
      if(atoi(output)<2) {
         ROS_ERROR("COMS_NODE: gzserver not running ... killing processes."); 
         _thrun = false;
         return false;
      } 
   }
   
   return true;
}
// #########################

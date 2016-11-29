//ROS includes
#include "behavior.h"
using namespace ros;
using namespace std;

/// \brief main worker thread
pthread_t worker_thread;
/// \brief behaviour class object
Behavior *behavior = NULL;
/// \brief struct to specify timer options for sending thread
struct itimerval timer;

pthread_cond_t condition_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t condition_mutex = PTHREAD_MUTEX_INITIALIZER;

#define DATA_UPDATE_HZ 30
#define DATA_UPDATE_USEC 1000000/DATA_UPDATE_HZ

static void syncronizeThread(int signal)
{
   if(signal==SIGALRM){
      pthread_cond_signal( &condition_cond ); //trigger signal
	}   
}

void* doWork(void*)
{
   while(ros::ok()){
      pthread_cond_wait( &condition_cond, &condition_mutex ); //wait for signal
      behavior->doWork();
   }
   
   return NULL;
}

int main(int argc, char **argv)
{
   if((argc>1 && argc!=3) && (argc>1 && argc!=2)) { 
      ROS_ERROR("Must enter robot id and mode as parameter for the simulated robot.\n Please use -s for simulation, followed by the robot's ID. Flag -m to run Matlab UDP bridge [-m or -sm]. [1]");
      exit(1); 
   }
   
   bool mode_real = true;
   bool run_matlab = false;
   int robot_id = 0;
   if(argc==3){
      robot_id = atoi(argv[2]);
      if(robot_id<1 || robot_id>5){
         ROS_ERROR("Must enter robot id correctly. Robot id's range from 1 to 5.");
         exit(2);     
      }
      if(!strcmp(argv[1],"-s")) mode_real = false;
      else if(!strcmp(argv[1],"-sm")) { run_matlab = true; mode_real = false; }
      else { 
         ROS_ERROR("Must enter mode correctly. Please use -s for simulation and/or -m to run Matlab UDP bridge [-m or -sm].");
         exit(3); 
      }
   } else if(argc==2){
      if(!strcmp(argv[1],"-m")) run_matlab = true;
      else { 
         ROS_ERROR("Must enter mode correctly. Please use -s for simulation and/or -m to run Matlab UDP bridge [-m or -sm] followed by robot's ID.");
         exit(3); 
      }
   }
   
   ROS_WARN("Attempting to start control services of control_node.");
   stringstream node_name;
   stringstream topic_base_name;
   node_name << "control_node";
   
   if(!mode_real) { 
      ROS_INFO("Running control_node for Simulated Robot %d",robot_id);
      node_name << robot_id; topic_base_name << "minho_gazebo_robot" << robot_id; 
   } 

   //Initialize ROS
	// #########################
	ros::init(argc, argv, node_name.str().c_str(),ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle control_node;
	
	behavior = new Behavior(topic_base_name.str(), run_matlab, &control_node);
	ROS_WARN("MinhoTeam control_node started running on ROS.");
	ros::AsyncSpinner spinner(2);
	spinner.start(); 
	
	//Setup Updated timer thread
	// #########################
	signal(SIGALRM,syncronizeThread);
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = DATA_UPDATE_USEC;
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = DATA_UPDATE_USEC;
	// #########################
	
	setitimer(ITIMER_REAL, &timer, NULL);
	pthread_create(&worker_thread, NULL, doWork, NULL);
	
	pthread_join(worker_thread, NULL);
	return 0;
}

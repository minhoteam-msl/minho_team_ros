//ROS includes
#include "behavior.h"
#include <boost/filesystem.hpp>
#include <boost/process.hpp>
using namespace ros;
using namespace std;

/// \brief main worker thread
pthread_t worker_thread;
/// \brief behaviour class object
Behavior *behavior = NULL;
/// \brief struct to specify timer options for sending thread
struct itimerval timer;
/// \brief variable to control execution based on existance of
/// gzserver if in simulation mode
bool run_program = true;
/// \brief mutexes to protect data
pthread_cond_t condition_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t condition_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief auxilizar variables to test connection of gzserver
std::string pgrep;
std::vector<std::string> pgrep_args;
/// \brief boolean variable to define operation mode (real or
/// simulation)
bool mode_real = true;

#define DATA_UPDATE_HZ 30
#define DATA_UPDATE_USEC 1000000/DATA_UPDATE_HZ

/// \brief function that is called at a frequency of DATA_UPDATE_HZ to 
/// activate a condition variable to manage the control loop execution
/// \param signal - system signal
static void syncronizeThread(int signal);

/// \brief function that waits for a condition variable to call the control loop
void* doWork(void*);

/// \brief checks if Gzserver is running in case of simulated robots
/// \param signal - system signal for timing purposes
bool checkGzserverConnection(int signal);

int main(int argc, char **argv)
{
   if((argc>1 && argc!=3) && (argc>1 && argc!=2)) { 
      ROS_ERROR("Must enter robot id and mode as parameter for the simulated robot.\n Please use -s for simulation, followed by the robot's ID. Flag -m to run Matlab UDP bridge [-m or -sm]. [1]");
      exit(1); 
   }
   
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
   
   ROS_WARN("Attempting to start control services of control_node.");
   stringstream node_name;
   stringstream topic_base_name;
   node_name << "control_node";
   
   if(!mode_real) { 
      ROS_INFO("Running control_node for Simulated Robot %d",robot_id);
      node_name << robot_id; topic_base_name << "minho_gazebo_robot" << robot_id; 
      //check for connection with gzserver
      pgrep = boost::process::find_executable_in_path("pgrep");
      pgrep_args.push_back("-c");
      pgrep_args.push_back("gzserver");
   } 

   //Initialize ROS
	// #########################
	ros::init(argc, argv, node_name.str().c_str(),ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle control_node;
	
	behavior = new Behavior(topic_base_name.str(), robot_id, &control_node);
	ROS_WARN("MinhoTeam control_node started running on ROS.");
	ros::AsyncSpinner spinner(2);
	
	//Setup Updated timer thread
	// #########################
	signal(SIGALRM,syncronizeThread);
	timer.it_interval.tv_sec = 0;
	timer.it_interval.tv_usec = DATA_UPDATE_USEC;
	timer.it_value.tv_sec = 0;
	timer.it_value.tv_usec = DATA_UPDATE_USEC;
	// #########################
	
	setitimer(ITIMER_REAL, &timer, NULL);
   spinner.start(); 
	pthread_create(&worker_thread, NULL, doWork, NULL);
	
	pthread_join(worker_thread, NULL);
	spinner.stop();
	ros::shutdown();
   ROS_ERROR("Exited control_node");
	return 0;
}

/// \brief function that is called at a frequency of DATA_UPDATE_HZ to 
/// activate a condition variable to manage the control loop execution
/// \param signal - system signal
static void syncronizeThread(int signal)
{
   static int counter = 0;
   if(signal==SIGALRM){
      if(counter<30) counter++; else { if(!checkGzserverConnection(signal)) return; counter = 0; }
      pthread_cond_signal( &condition_cond ); //trigger signal
	}   
}

/// \brief function that waits for a condition variable to call the control loops
void* doWork(void*)
{
   while(ros::ok() && run_program){
      pthread_cond_wait( &condition_cond, &condition_mutex ); //wait for signal
      behavior->doWork();
   }
   
   pthread_exit(NULL);
   return NULL;
}

/// \brief checks if Gzserver is running in case of simulated robots
/// \param signal - system signal for timing purposes
bool checkGzserverConnection(int signal)
{
   if(signal==SIGALRM && !mode_real){
      boost::process::context ctx; 
      ctx.streams[boost::process::stdout_id] = boost::process::behavior::pipe(); 
      boost::process::child c = boost::process::create_child(pgrep, pgrep_args, ctx); 
      boost::process::pistream is(c.get_handle(boost::process::stdout_id)); 
      char output[2];
      is.read(output,2);
      c.wait();
      if(atoi(output)!=2) {
         ROS_ERROR("CONTROL_NODE: gzserver not running ... killing processes."); 
         run_program = false;
         timer.it_interval.tv_sec = 0;
         timer.it_interval.tv_usec = 0;
         timer.it_value.tv_sec = 0;
         timer.it_value.tv_usec = 0;
         setitimer(ITIMER_REAL, &timer, NULL);
         return true;
      } 
   }
   
   return true;
}

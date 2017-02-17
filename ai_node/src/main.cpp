//Network includes for getRobotIdByIP()

#include <boost/filesystem.hpp>
#include <boost/process.hpp>
#include "ai.h"

#define DATA_UPDATE_HZ 30
#define DATA_UPDATE_USEC 1000000/DATA_UPDATE_HZ

using namespace std;

/// \brief main worker thread
pthread_t worker_thread;
/// \brief struct to specify timer options for sending thread
struct itimerval timer;

pthread_cond_t condition_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t condition_mutex = PTHREAD_MUTEX_INITIALIZER;

/// \brief int variable to control thread running
bool _thrun = true;
/// \brief tells if is simulated or real robot 
bool mode_real = true;
/// \brief auxilizar variables to test connection of gzserver
std::string pgrep;
std::vector<std::string> pgrep_args;
// \brief utility class object pointer
AI *ai;

/// \brief returns robot id by its ip
int getRobotIdByIP();

/// \brief checks if Gzserver is running in case of simulated robots
/// \param signal - system signal for timing purposes
bool checkGzserverConnection(int signal);

/// \brief frees condition variable in dowork thread, calling
/// a function periodically
static void syncronizeThread(int signal)
{
    static int counter = 0;
    if(signal==SIGALRM){
      if(counter<30) counter++; else { if(!checkGzserverConnection(signal)) return; counter = 0; }
      pthread_cond_signal(&condition_cond); //trigger signal
    }
}

/// \brief periodically calls a function from the utility class
void* doWork(void*)
{
    while(ros::ok() && _thrun){
      pthread_cond_wait( &condition_cond, &condition_mutex ); //wait for signal
      ai->computeAI();
    }

    return NULL;
}

/// \brief sets up ros node and some other stuff
int main(int argc, char **argv)
{
   if((argc>1 && argc!=3) && (argc>1 && argc!=2)) {
   ROS_ERROR("Must enter robot id and mode as parameter for the simulated robot.\n Please use -s for simulation, followed by the robot's ID.");
   exit(1);
   }

   mode_real = true;
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
   } else if(argc==2) {
      ROS_ERROR("Must enter mode correctly. Please use -s for simulation.");
      exit(3);
   }

   if(mode_real){
   robot_id = getRobotIdByIP();
   if(robot_id<0) { robot_id = 1; ROS_ERROR("Error in Robot ID by IP address ... Defaulting to 1."); }
   }

   ROS_WARN("Attempting to start ai services of ai_node.");
   stringstream node_name;
   stringstream topic_base_name;
   node_name << "ai_node";

   if(!mode_real) {
   ROS_INFO("Running control_node for Simulated Robot %d",robot_id);
   node_name << robot_id; topic_base_name << "minho_gazebo_robot" << robot_id;
   }

   //Initialize ROS
   // #########################
   ros::init(argc, argv, node_name.str().c_str(),ros::init_options::NoSigintHandler);
   //Request node handler
   ros::NodeHandle ai_node;
   ai = new AI(&ai_node,mode_real, robot_id);

   ROS_WARN("MinhoTeam ai_node started running on ROS.");
   ros::AsyncSpinner spinner(2);
   spinner.start();

   if(!mode_real){ //check for connection with gzserver
      pgrep = boost::process::find_executable_in_path("pgrep");
      pgrep_args.push_back("-c");
      pgrep_args.push_back("gzserver");
   }
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

int getRobotIdByIP()
{
   int	fd;
	struct ifreq if_info;
	int if_index;
   std::string ifname = "wlan0";
	memset(&if_info, 0, sizeof(if_info));
	strncpy(if_info.ifr_name, ifname.c_str(), IFNAMSIZ-1);

	if ((fd=socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		ROS_ERROR("Error getting UDP socket");
		return -1;
	}
	if (ioctl(fd, SIOCGIFINDEX, &if_info) == -1)
	{
		ROS_ERROR("Error sending IOCTL 1");
		close(fd);
		return -1;
	}
	if_index = if_info.ifr_ifindex;

	if (ioctl(fd, SIOCGIFADDR, &if_info) == -1)
	{
		ROS_ERROR("Error sending IOCTL 2");
		close(fd);
		return -1;
	}
	
	close(fd);
	int id = if_info.ifr_hwaddr.sa_data[5];
   if(id<1 || id>5) id = -1;
   return id;
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
         ROS_ERROR("AI_NODE: gzserver not running ... killing processes."); 
         _thrun = false;
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

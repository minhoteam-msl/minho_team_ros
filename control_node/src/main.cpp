#include "behavior.h"
#include "Utils/getIP.h"

/// \brief main worker thread
pthread_t worker_thread;
/// \brief fundamental class object
Fundamental *aptFundamental = NULL;
/// \brief voronoi class object
Voronoi *aptVoronoi = NULL;
/// \brief dijkstrashortestpath class object
DijkstraShortestPath *aptDijkstraShortestPath = NULL;
/// \brief motion class object
Motion *aptMotion = NULL;
/// \brief behaviour class object
Behavior *aptBehavior = NULL;
/// \brief struct to specify timer options for sending thread
struct itimerval timer;

pthread_cond_t condition_cond = PTHREAD_COND_INITIALIZER;
pthread_mutex_t condition_mutex = PTHREAD_MUTEX_INITIALIZER;

#define DATA_UPDATE_HZ 30
#define DATA_UPDATE_USEC 1000000/DATA_UPDATE_HZ

//
static void syncronizeThread(int signal)
{
    if(signal==SIGALRM){
      pthread_cond_signal( &condition_cond ); //trigger signal
    }
}

//
void* doWork(void*)
{
    while(ros::ok()){
      pthread_cond_wait( &condition_cond, &condition_mutex ); //wait for signal
      aptBehavior->doWork();
    }

    return NULL;
}


//
int main(int argc, char **argv)
{
    if((argc>1 && argc!=3)) {
      ROS_ERROR("Must enter robot id and mode as parameter for the simulated robot.\n");
      exit(1);
    }

    bool mode_real = true;
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
    
    if(mode_real){
      robot_id = getRobotIdByIP(std::string("wlan0"));
      if(robot_id<0) { robot_id = 1; ROS_ERROR("Error in Robot ID by IP address ... Defaulting to 1."); }
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

    //
    aptFundamental = new Fundamental();
    aptVoronoi = new Voronoi(aptFundamental);
    aptDijkstraShortestPath = new DijkstraShortestPath(aptFundamental, aptVoronoi);
    aptMotion = new Motion(aptFundamental);
    aptBehavior = new Behavior(topic_base_name.str(), robot_id, mode_real, &control_node, aptFundamental, aptVoronoi, aptDijkstraShortestPath, aptMotion);
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

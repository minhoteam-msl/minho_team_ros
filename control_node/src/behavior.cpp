#include "behavior.h"

pthread_mutex_t robot_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ai_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t control_config_mutex = PTHREAD_MUTEX_INITIALIZER;

Behavior::Behavior(std::string topics_base_name, int rob_id, ros::NodeHandle *par, Fundamental *fund, Voronoi *vor, DijkstraShortestPath *dijk, Motion *mot)
{
    motion = mot;
    dijkstra_path = dijk;
    voronoi = vor;
    fundamental = fund;
    parent = par;
    robot_id = rob_id;
    stringstream robot_topic_name; robot_topic_name << topics_base_name;
    stringstream control_topic_name; control_topic_name << topics_base_name;
    stringstream ai_topic_name; ai_topic_name << topics_base_name;
    stringstream kick_service_name; kick_service_name << topics_base_name;
    stringstream control_config_topic_name; control_config_topic_name << topics_base_name;
    stringstream control_config_sv_topic_name; control_config_sv_topic_name << topics_base_name;
    stringstream path_topic_name; path_topic_name << topics_base_name;

    robot_topic_name << "/robotInfo";
    control_topic_name << "/controlInfo";
    ai_topic_name << "/aiInfo";
    kick_service_name << "/requestKick";
    control_config_topic_name << "/controlConfig";
    control_config_sv_topic_name << "/requestControlConfig";
    path_topic_name << "/pathData";

    //## Setup ROS Pubs and Subs ##
    //#############################
    ROS_INFO("Getting configurations from Robot %d", robot_id);
    //Initialize controlInfo publisher
    control_info_pub = parent->advertise<controlInfo>(control_topic_name.str(), 1);
    path_data_pub = parent->advertise<pathData>(path_topic_name.str(), 1);
    //Initialize robotInfo subscriber
    robot_info_sub = par->subscribe(robot_topic_name.str(),
                                       1,
                                       &Behavior::robotInfoCallback,
                                       this);
    ai_info_sub = par->subscribe(ai_topic_name.str(),
                                       1,
                                       &Behavior::aiInfoCallback,
                                       this);
    control_config_sub = par->subscribe(control_config_topic_name.str(),
                                       1,
                                       &Behavior::controlConfigCallback,
                                       this);

    kick_service = par->serviceClient<requestKick>(kick_service_name.str());

    service_control_config = par->advertiseService(control_config_sv_topic_name.str(),
                                  &Behavior::controlConfService,
                                  this);
    //**************************************************************************************

    control_config.P = 0.45; control_config.I = 0.00; control_config.D = 0.0;
    control_config.max_linear_velocity = 60;
    control_config.max_angular_velocity = 60;
    control_config.acceleration = 0.15;
    control_config.deceleration = 0.15;
    if(!initParameters()) ROS_ERROR("Failed to initialize control parameters");
    //

}

//
bool Behavior::initParameters()
{
    QString home = QString::fromStdString(getenv("HOME"));
    QString cfgDir = home+QString(CCONFIGFOLDERPATH);

    controlparam_file = cfgDir+QString("Robot")+QString::number(robot_id)+"/"+QString(CONTROLFILENAME);

    return readControlParameters();
}

//
bool Behavior::readControlParameters()
{
    QFile file(controlparam_file);
    if(!file.open(QIODevice::ReadOnly)) {
      ROS_ERROR("Failed to read %s.",CONTROLFILENAME);
      return false;
    }
    QTextStream in(&file);

    QString line = "";
    while(!in.atEnd()){
      line = in.readLine();
      if(line[0]=='#') continue;
      else {
         QString label_name = line.left(line.indexOf("="));
         QString value = line.right(line.size()-line.indexOf('=')-1);

         if(label_name=="P"){
            control_config.P = value.toFloat();
         } else if(label_name=="I"){
            control_config.I = value.toFloat();
         } else if(label_name=="D"){
            control_config.D = value.toFloat();
         } else if(label_name=="lin_max"){
            control_config.max_linear_velocity = value.toInt();
         } else if(label_name=="ang_max"){
            control_config.max_angular_velocity = value.toInt();
         } else if(label_name=="accel"){
            control_config.acceleration = value.toFloat();
         } else if(label_name=="decel"){
            control_config.deceleration = value.toFloat();
         } else { ROS_ERROR("Bad Configuration (2) in %s",CONTROLFILENAME); return false; }
      }
    }

    file.close();
    return true;
}

//
bool Behavior::writeControlParameters()
{
    QFile file(controlparam_file);
    if(!file.open(QIODevice::WriteOnly)){
     ROS_ERROR("Error writing to %s.",CONTROLFILENAME);
     return false;
    }
    QTextStream in(&file);

    in << "P="<< control_config.P << "\n";
    in << "I="<< control_config.I << "\n";
    in << "D="<< control_config.D << "\n";
    in << "lin_max="<< control_config.max_linear_velocity << "\n";
    in << "ang_max="<< control_config.max_angular_velocity << "\n";
    in << "accel="<< control_config.acceleration << "\n";
    in << "decel="<< control_config.deceleration << "\n";

    QString message = QString("#DONT CHANGE THE ORDER OF THE CONFIGURATIONS");
    in << message;
    file.close();
    return true;
}

//
bool Behavior::controlConfService(requestControlConfig::Request &req, requestControlConfig::Response &res)
{
    res.config = control_config;
}

//
void Behavior::robotInfoCallback(const robotInfo::ConstPtr &msg)
{
    pthread_mutex_lock(&robot_info_mutex); //Lock mutex
    robot_info = *msg;
    pthread_mutex_unlock(&robot_info_mutex); //Unlock mutex
}

//
void Behavior::aiInfoCallback(const aiInfo::ConstPtr &msg)
{
    pthread_mutex_lock(&ai_info_mutex); //Lock mutex
    ai_info = *msg;
    pthread_mutex_unlock(&ai_info_mutex); //Unlock mutex
}

//
void Behavior::controlConfigCallback(const controlConfig::ConstPtr &msg)
{
    pthread_mutex_lock(&control_config_mutex); //Lock mutex
    control_config = *msg;
    writeControlParameters();
    pthread_mutex_unlock(&control_config_mutex); //Unlock mutex
}


//*************************************************************************************************************
void Behavior::doWork()
{
    pthread_mutex_lock(&robot_info_mutex); //Lock mutex
    robotInfo robot_info_copy = robot_info; // Make a copy to hold the mutex for less time
    pthread_mutex_unlock(&robot_info_mutex); //Unlock mutex

    pthread_mutex_lock(&ai_info_mutex); //Lock mutex
    aiInfo ai_info_copy = ai_info; // Make a copy to hold the mutex for less time
    pthread_mutex_unlock(&ai_info_mutex); //Unlock mutex

    pthread_mutex_lock(&control_config_mutex); //Lock mutex
    controlConfig control_config_copy = control_config; // Make a copy to hold the mutex for less time
    pthread_mutex_unlock(&control_config_mutex); //Unlock mutex

    /// \brief Implement behaviour based on ai info
    /// \brief Roles : 0 - GK | 1 - DEFENSE | 2 - ATTACKER | 3 - SUPPORT ATTACKER
    /// \brief Actions : 0 - STOP | 1 - GO_TO_POSITION | 2 - KICK_PASS | 3 - ENGAGE_BALL

    switch(ai_info_copy.action) {
       case aSTOP: {
           control_info.linear_velocity = control_info.angular_velocity = 0;
       } break;

       case aSLOWMOVE: {
           goToPosition2(robot_info_copy, ai_info_copy, control_config_copy,30);

       } break;

       case aFASTMOVE: {
           goToPosition2(robot_info_copy, ai_info_copy, control_config_copy,50);

       } break;

       default: {
           control_info.linear_velocity = control_info.angular_velocity = 0;
       } break;
    }

    path_data_pub.publish(path_data);
    control_info_pub.publish(control_info);
}


//
void Behavior::goToPosition1(robotInfo robot, aiInfo ai, controlConfig cconfig)
{
    int target_angle = fundamental->cartesian2polar_angleDeg_halfCircle(robot.robot_pose.x, robot.robot_pose.y,
                                                                          ai.target_pose.x, ai.target_pose.y);

    control_info.angular_velocity = motion->angularVelocity(target_angle, robot, cconfig);

    if(control_info.angular_velocity == 0) {
        //fundamental->distance(robot.robot_pose.x, robot.robot_pose.y, );
        control_info.linear_velocity = 30;
    }
}

//
void Behavior::goToPosition2(robotInfo robot, aiInfo ai, controlConfig cconfig, int max_vel)
{
    int target_angle = 0;
    float x_min = 0.0;
    float x_max = 0.0;
    float y_min = 0.0;
    float y_max = 0.0;

    //ver melhor esta função (angle)

    target_angle = fundamental->cartesian2polar_angleDegNormalize(robot.robot_pose.x, robot.robot_pose.y,
                                                                          ai.target_pose.x, ai.target_pose.y);

    control_info.movement_direction = motion->movementDirection(robot, target_angle);

    x_min = ai.target_pose.x - 0.10;
    x_max = ai.target_pose.x + 0.10;
    y_min = ai.target_pose.y - 0.10;
    y_max = ai.target_pose.y + 0.10;

    if(robot.robot_pose.x>x_min && robot.robot_pose.x<x_max && robot.robot_pose.y>y_min && robot.robot_pose.y<y_max) {
        control_info.linear_velocity = 0;

        int target_next_angle;

        if((int)ai.target_pose.z > 180)
            target_next_angle = (int)ai.target_pose.z - 360;
        else
            target_next_angle = (int)ai.target_pose.z;

        control_info.angular_velocity = motion->angularVelocity(target_next_angle, robot, cconfig);

    }
    else {
        control_info.linear_velocity = max_vel;
    }
}

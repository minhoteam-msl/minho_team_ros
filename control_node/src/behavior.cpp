#include "behavior.h"

pthread_mutex_t robot_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ai_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t control_config_mutex = PTHREAD_MUTEX_INITIALIZER;

Behavior::Behavior(std::string topics_base_name, int robot_id, bool mode_real, ros::NodeHandle *par, Fundamental *fund,
                   Voronoi *vor, DijkstraShortestPath *dijk, Motion *mot)
{
    motion = mot;
    dijkstra_path = dijk;
    voronoi = vor;
    fundamental = fund;
    parent = par;
    this->mode_real = mode_real;
    this->robot_id = robot_id;
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

    control_config.Kp_rot = 0.45; control_config.Ki_rot = 0.0; control_config.Kd_rot = 0.0;
    control_config.Kp_lin = 0.45; control_config.Ki_lin = 0.0; control_config.Kd_lin = 0.0;
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
    QString commonDir = home+QString(COMMON_PATH);
    QString cfgDir = commonDir+QString(CTRL_CFG_PATH);

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
    while(!in.atEnd()) {
        line = in.readLine();
        if(line[0]=='#') continue;
        else {
            QString label_name = line.left(line.indexOf("="));
            QString value = line.right(line.size()-line.indexOf('=')-1);

            if(label_name == "Kp_rot") {
                control_config.Kp_rot = value.toFloat();
            }else if(label_name == "Ki_rot") {
                control_config.Ki_rot = value.toFloat();
            }else if(label_name == "Kd_rot") {
                control_config.Kd_rot = value.toFloat();
            }else if(label_name == "Kp_lin") {
                control_config.Kp_lin = value.toFloat();
            }else if(label_name == "Ki_lin") {
                control_config.Ki_lin = value.toFloat();
            }else if(label_name == "Kd_lin") {
                control_config.Kd_lin = value.toFloat();
            }else if(label_name == "lin_max") {
                control_config.max_linear_velocity = value.toInt();
            }else if(label_name == "ang_max") {
                control_config.max_angular_velocity = value.toInt();
            }else if(label_name == "accel") {
                control_config.acceleration = value.toFloat();
            }else if(label_name == "decel") {
                control_config.deceleration = value.toFloat();
            }else {
                ROS_ERROR("Bad Configuration (2) in %s",CONTROLFILENAME);
                return false;
            }
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

    in << "Kp_rot="<< control_config.Kp_rot << "\n";
    in << "Ki_rot="<< control_config.Ki_rot << "\n";
    in << "Kd_rot="<< control_config.Kd_rot << "\n";
    in << "Kp_lin="<< control_config.Kp_lin << "\n";
    in << "Ki_lin="<< control_config.Ki_lin << "\n";
    in << "Kd_lin="<< control_config.Kd_lin << "\n";
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
    //*************************************************************************************************
    vector<Point> path;
    control_info = controlInfo();  // !!!!!

    /// \brief Implement behaviour based on ai info
    float heading_error = 0.0;
    switch(ai_info_copy.action) {
    case aSTOP: {
        control_info.linear_velocity = control_info.angular_velocity = 0;
        control_info.dribbler_on = false;
    } break;

    case aAPPROACHPOSITION: {
        if(USE_PATH)
            dijkstra_path->Test1(robot_info_copy, Point(ai_info_copy.target_pose.x, ai_info_copy.target_pose.y), path);
        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 0.5);
        control_info.dribbler_on = false;
    } break;

    case aFASTMOVE: {
        if(USE_PATH)
            dijkstra_path->Test1(robot_info_copy, Point(ai_info_copy.target_pose.x, ai_info_copy.target_pose.y), path);
        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 1.0);
        control_info.dribbler_on = false;
    } break;

    case aRECEIVEBALL: {
        if(USE_PATH)
            dijkstra_path->Test1(robot_info_copy, Point(ai_info_copy.target_pose.x, ai_info_copy.target_pose.y), path);
        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 0.5);
        control_info.dribbler_on = true;
    } break;

    case aENGAGEBALL: {
        if(USE_PATH)
            dijkstra_path->Test1(robot_info_copy, Point(ai_info_copy.target_pose.x, ai_info_copy.target_pose.y), path);
        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 1.0);
        float distToBall = sqrt(
                    (robot_info_copy.robot_pose.x-ai_info_copy.target_pose.x)*
                    (robot_info_copy.robot_pose.x-ai_info_copy.target_pose.x)
                    +((robot_info_copy.robot_pose.y-ai_info_copy.target_pose.y)*
                    (robot_info_copy.robot_pose.y-ai_info_copy.target_pose.y)));
        if(distToBall<0.33){control_info.linear_velocity = control_info.angular_velocity = 0;}
        control_info.dribbler_on = true;
    } break;

    case aAPPROACHBALL: {
        if(robot_info_copy.sees_ball){
            obstacle ballobs;
            ballobs.x = robot_info_copy.ball_position.x;
            ballobs.y = robot_info_copy.ball_position.y;
            robot_info_copy.obstacles.push_back(ballobs);
        }
        if(USE_PATH)
            dijkstra_path->Test1(robot_info_copy, Point(ai_info_copy.target_pose.x, ai_info_copy.target_pose.y), path);
        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 0.5);
        control_info.dribbler_on = false;
    } break;

    case aPASSBALL: {
        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 1.0);
        control_info.linear_velocity = 0;
        heading_error = atan2(sin((robot_info_copy.robot_pose.z-ai_info_copy.target_pose.z)*DEGTORAD), cos((robot_info_copy.robot_pose.z-ai_info_copy.target_pose.z)*DEGTORAD))*DEGTORAD;
        ROS_INFO("HEADING %.2f",heading_error);
        
        if(fabs(heading_error)<5.0){
            requestKick srv;
            srv.request.kick_is_pass = true;
            srv.request.kick_strength = ai_info_copy.target_kick_strength;
            kick_service.call(srv);
        }
        control_info.dribbler_on = true;
    } break;

    case aKICKBALL: {
        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 1.0);
        control_info.linear_velocity = 0;
        
        float ha = robot_info_copy.robot_pose.z;
        while(ha<0) ha += 360.0;
        while(ha>360) ha -= 360.0;
        
        float hb = ai_info_copy.target_pose.z;
        while(hb<0) hb += 360.0;
        while(hb>360) hb -= 360.0;
        
        if(ha<90&&hb>270) hb -= 360.0;
        if(ha>270 && hb<90) hb += 360.0;
        
        float heading_error = fabs(ha-hb);
        ROS_INFO("Heading error %.2f", heading_error);
        
        if(heading_error<5.0){
            requestKick srv;
            srv.request.kick_is_pass = false;
            srv.request.kick_strength = ai_info_copy.target_kick_strength;
            kick_service.call(srv);
        }

        control_info.dribbler_on = true;
    } break;

    case aHOLDBALL: {
        control_info.linear_velocity = control_info.angular_velocity = 0;
        control_info.dribbler_on = true;
    } break;

    case aDRIBBLEBALL: {
        if(USE_PATH)
            dijkstra_path->Test1(robot_info_copy, Point(ai_info_copy.target_pose.x, ai_info_copy.target_pose.y), path);
        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 1.0);
        control_info.dribbler_on = true;
    } break;

    case 50: { // action = TEST

        //dijkstra_path->Test1(robot_info_copy, Point(robot_info_copy.ball_position.x, robot_info_copy.ball_position.y), path);

        /*int move_direction;
        control_info.linear_velocity = motion->linearVelocity(robot_info_copy, control_config_copy, path, move_direction);
        control_info.movement_direction = move_direction;*/

        dijkstra_path->Test1(robot_info_copy, Point(ai_info_copy.target_pose.x, ai_info_copy.target_pose.y), path);

        goToPosition2(robot_info_copy, ai_info_copy, control_config_copy, path, 1.0);

    } break;

    default: {
        control_info.linear_velocity = control_info.angular_velocity = 0;
    } break;
    }

    //************************************Send Visualizer************************************
    if(ai_info_copy.action != aSTOP) {
        // Send segments voronoi
        if(control_config_copy.send_voronoi_seg) {
            path_data.voronoi_seg.clear();
            voronoi->cut_VoronoiDiagram_FieldLimits();
            path_data.voronoi_seg = voronoi->get_VoronoiSegments_Visualizer();
        }else
            path_data.voronoi_seg.clear();
        // Send intersect segments
        if(control_config_copy.send_intersect_seg) {
            path_data.intersect_seg.clear();
            path_data.intersect_seg = dijkstra_path->get_IntersectSegments_Visualizer();
        }else
            path_data.intersect_seg.clear();
        // Send dijkstra path
        if(control_config_copy.send_dijk_path) {
            path_data.dijk_path.clear();
            path_data.dijk_path = dijkstra_path->get_DijkstraPath_Visualizer();
        }else
            path_data.dijk_path.clear();
        // Send dijkstra path with obstacles circle
        if(control_config_copy.send_dijk_path_obst_circle) {
            path_data.dijk_path_obst_circle.clear();
            path_data.dijk_path_obst_circle = dijkstra_path->get_DijkstraPath_with_ObstaclesCircle_Visualizer();
        }else
            path_data.dijk_path_obst_circle.clear();
        // Send smooth path
        if(control_config_copy.send_smooth_path) {
            path_data.smooth_path.clear();
            path_data.smooth_path = dijkstra_path->get_SmoothPath_Visualizer();
        }else
            path_data.smooth_path.clear();
        // Send smooth path with obstacles circle
        if(control_config_copy.send_smooth_path_obst_circle) {
            path_data.smooth_path_obst_circle.clear();
            path_data.smooth_path_obst_circle = dijkstra_path->get_SmoothPath_with_ObstaclesCircle_Visualizer();
        }else
            path_data.smooth_path_obst_circle.clear();
        // Send path
        if(control_config_copy.send_path) {
            path_data.path.clear();
            path_data.path = dijkstra_path->get_Path_Visualizer();
        }else
            path_data.path.clear();
        // Send path interpolation
        if(control_config_copy.send_path_interpolation) {
            path_data.path_interpolation.clear();
            path_data.path_interpolation = dijkstra_path->get_PathInterpolation_Visualizer();
        }else
            path_data.path_interpolation.clear();
        //Send obstacles circle
        if(control_config_copy.send_obstacles_circle) {
            path_data.obstacles_circle.clear();
            path_data.obstacles_circle = dijkstra_path->get_ObstaclesCircle_Visualizer();
        }else
            path_data.obstacles_circle.clear();
    }

    voronoi->clearVoronoi();
    dijkstra_path->clearDijkstraPath();
    path_data_pub.publish(path_data);
    control_info_pub.publish(control_info);
}

//
void Behavior::goToPosition1(robotInfo robot, aiInfo ai, controlConfig cconfig)
{
    int target_angle = fundamental->cartesian2polar_angleDeg_halfCircle(robot.robot_pose.x, robot.robot_pose.y,
                                                                          ai.target_pose.x, ai.target_pose.y);

    control_info.angular_velocity = motion->angularVelocity_PID(robot.robot_pose.z, (float)target_angle, cconfig);

    if(control_info.angular_velocity == 0) {
        control_info.linear_velocity = 30;
    }
}

//
void Behavior::goToPosition2(robotInfo robot, aiInfo ai, controlConfig cconfig, const vector<Point>& path, float percent_vel)
{
    int target_angle = 0;
    float x_min, x_max, y_min, y_max;
    static int stab_counter = 0;

    //ver melhor esta função (angle)
    /*target_angle = fundamental->cartesian2polar_angleDegNormalize(robot.robot_pose.x, robot.robot_pose.y,
                                                                          ai.target_pose.x, ai.target_pose.y);*/

    if(path.size() > 1) {
        target_angle = fundamental->cartesian2polar_angleDegNormalize(robot.robot_pose.x, robot.robot_pose.y,
                                                                  path.at(1).x(), path.at(1).y());
        control_info.movement_direction = motion->movementDirection((int)robot.robot_pose.z, target_angle);

        x_min = ai.target_pose.x - 0.10;
        x_max = ai.target_pose.x + 0.10;
        y_min = ai.target_pose.y - 0.10;
        y_max = ai.target_pose.y + 0.10;

 
        if(ai.action!=aPASSBALL && ai.action!=aKICKBALL && ai.action!=aDRIBBLEBALL){
            float target_next_angle;
            if(ai.target_pose.z > 180.0) target_next_angle = ai.target_pose.z - 360.0;
            else target_next_angle = ai.target_pose.z;
            control_info.angular_velocity = motion->angularVelocity_PID(robot.robot_pose.z, target_next_angle, cconfig);
        }

        if(robot.robot_pose.x>x_min && robot.robot_pose.x<x_max && robot.robot_pose.y>y_min && robot.robot_pose.y<y_max) {
            if(ai.action==aPASSBALL || ai.action==aKICKBALL || ai.action==aDRIBBLEBALL){
                float target_next_angle;
                if(ai.target_pose.z > 180.0) target_next_angle = ai.target_pose.z - 360.0;
                else target_next_angle = ai.target_pose.z;
                control_info.angular_velocity = motion->angularVelocity_PID(robot.robot_pose.z, target_next_angle, cconfig);
            }

        } else {
            //control_info.linear_velocity = max_vel;
            control_info.linear_velocity = motion->linearVelocity(cconfig, path, (int)ai.action, percent_vel);
            stab_counter = 0;
        }
    }else {
        if(ai.action==aPASSBALL || ai.action==aKICKBALL || ai.action==aDRIBBLEBALL){
                float target_next_angle;
                if(ai.target_pose.z > 180.0) target_next_angle = ai.target_pose.z - 360.0;
                else target_next_angle = ai.target_pose.z;
                control_info.angular_velocity = motion->angularVelocity_PID(robot.robot_pose.z, target_next_angle, cconfig);
        } else {
            control_info.linear_velocity = control_info.angular_velocity = 0;
            stab_counter = 0;
        }
    }
}



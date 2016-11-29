#include "behavior.h"

pthread_mutex_t robot_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t ai_info_mutex = PTHREAD_MUTEX_INITIALIZER;

Behavior::Behavior(std::string topics_base_name, bool run_matlab, ros::NodeHandle *par)
{
   parent = par;
   
   stringstream robot_topic_name; robot_topic_name << topics_base_name;
   stringstream control_topic_name; control_topic_name << topics_base_name;
   stringstream ai_topic_name; ai_topic_name << topics_base_name;
   stringstream kick_service_name; kick_service_name << topics_base_name;
   
   robot_topic_name << "/robotInfo";
   control_topic_name << "/controlInfo";
   ai_topic_name << "/aiInfo";
   kick_service_name << "/requestKick";
   
   //## Setup ROS Pubs and Subs ##
   //############################# 
   //Initialize controlInfo publisher
   control_info_pub = parent->advertise<controlInfo>(control_topic_name.str(), 1);
	//Initialize robotInfo subscriber
	robot_info_sub = par->subscribe(robot_topic_name.str(), 
                                       1, 
                                       &Behavior::robotInfoCallback,
                                       this);               
   ai_info_sub = par->subscribe(ai_topic_name.str(), 
                                       1, 
                                       &Behavior::aiInfoCallback,
                                       this);         
   kick_service = par->serviceClient<requestKick>(kick_service_name.str());                                       
   //############################# 
   
   error = previous_error = 0.0;
   derivative = integral = 0.0;
   kp_rot = 0.45; ki_rot = 0.00; kd_rot = 0.0;
   
   if(run_matlab){
      matlab = new ComsMatlab();
      matlab->openComs();
   }
}

void Behavior::robotInfoCallback(const robotInfo::ConstPtr &msg)
{
   pthread_mutex_lock(&robot_info_mutex); //Lock mutex
   robot_info = *msg;
   pthread_mutex_unlock(&robot_info_mutex); //Unlock mutex
}

void Behavior::aiInfoCallback(const aiInfo::ConstPtr &msg)
{
   pthread_mutex_lock(&ai_info_mutex); //Lock mutex
   ai_info = *msg;
   pthread_mutex_unlock(&ai_info_mutex); //Unlock mutex
}

void Behavior::doWork()
{
   pthread_mutex_lock(&robot_info_mutex); //Lock mutex   
   minho_team_ros::robotInfo robot = robot_info; // Make a copy to hold the mutex for less time
   pthread_mutex_unlock(&robot_info_mutex); //Unlock mutex
   
   pthread_mutex_lock(&ai_info_mutex); //Lock mutex
   minho_team_ros::aiInfo ai = ai_info; // Make a copy to hold the mutex for less time
   pthread_mutex_unlock(&ai_info_mutex); //Unlock mutex
   
   /// \brief implementation of Control Sequence
   /// \brief Roles : 0 - GK | 1 - DEFENSE | 2 - ATTACKER | 3 - SUPPORT ATTACKER
   /// \brief Actions : 0 - PARAR | 1 - IR P/ POSIÇÃO | 2 - ATRAS DA BOLA | 3 - CHUTAR/PASSAR
   
   if(ai.action == STOP) control.linear_velocity = control.angular_velocity = 0;
   else { // Implement behaviour based on ai info
   
   }
   control_info_pub.publish(control);
}

// MATH FUNCTIONS
float Behavior::Distance(float positionAX, float positionAY, float positionBX, float positionBY)
{
   float difX = positionBX - positionAX;
   float difY = positionBY - positionAY;

   return sqrt((difX*difX) + (difY*difY));
}

//Calcular ângulo do alvo em graus relativamente ao robô
//return ângulo
int Behavior::PsiTarget(double positionAX, double positionAY, double positionBX, double positionBY, bool normalize)
{
    int psiTargetAngleDegrees=0;

    psiTargetAngleDegrees = (int)((atan2((positionBY-positionAY),(positionBX-positionAX))*180.0)/M_PI);

    if(normalize){
        if(psiTargetAngleDegrees<0)
            psiTargetAngleDegrees +=360;

        if(psiTargetAngleDegrees>=0 && psiTargetAngleDegrees<90)
            psiTargetAngleDegrees = 360+psiTargetAngleDegrees-90;
        else
            psiTargetAngleDegrees -=90;
    }
    
    return psiTargetAngleDegrees;
}

//
int Behavior::RotationRobot_for_Target_PID(int psi_target, int rotational_speed_max)
{
    int robot_angle = robot_info.robot_pose.z;
    int rotational_speed=0;

    if(robot_angle>90 && robot_angle<360)
        robot_angle -=270;
    else
        robot_angle +=90;

    error = (double)(robot_angle - psi_target);

    if(error>180.0)
        error += -360.0;
    else if(error<-180.0)
        error += 360.0;

    derivative = (error-previous_error)/0.03333;

    rotational_speed = (int)(kp_rot*error + ki_rot*integral + kd_rot*derivative);

    if((error*previous_error)<=0) integral=0.0;

    if(rotational_speed>rotational_speed_max)
        rotational_speed=rotational_speed_max;
    else if(rotational_speed<-rotational_speed_max)
        rotational_speed=-rotational_speed_max;
    else
        integral += error*0.03333;

    previous_error = error;
    
    return rotational_speed;
}

int Behavior::SpeedRobot_TargetDistance_Log(double distanceRobot_Target, int linear_speed_max)
{
    int linear_speed=0;
    linear_speed = (int)(linear_speed_max*log(distanceRobot_Target+1.08));

    if(linear_speed>linear_speed_max)
        linear_speed = linear_speed_max;
    else if(linear_speed<0)
        linear_speed = 0;
    return linear_speed;
}

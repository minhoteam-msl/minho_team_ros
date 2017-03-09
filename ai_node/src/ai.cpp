#include "ai.h"

pthread_mutex_t bs_info_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t robot_info_mutex = PTHREAD_MUTEX_INITIALIZER;

AI::AI(ros::NodeHandle* parent, bool mode_real, int robot_id)
{
   node = parent;
   agent_id = robot_id;
   std::stringstream bs_topic_name, ai_topic_name, robot_topic_name;
   topic_base = "";
   if(!mode_real){
      topic_base += "/minho_gazebo_robot"+std::to_string((int)agent_id);
      bs_topic_name << "/minho_gazebo_robot" << (int)agent_id;
      ai_topic_name << "/minho_gazebo_robot" << (int)agent_id;
      robot_topic_name << "/minho_gazebo_robot" << (int)agent_id;
   }
   bs_topic_name << "/basestation/baseStationInfo";
   ai_topic_name << "/aiInfo";
   robot_topic_name << "/robotInfo";

   bs_sub = node->subscribe(bs_topic_name.str().c_str(),
            1,&AI::baseStationInfoCallback,this);
   rob_sub = node->subscribe(robot_topic_name.str().c_str(),
            1,&AI::robotInfoCallback,this);
   ai_pub = node->advertise<aiInfo>(ai_topic_name.str().c_str(),1);
   role = NULL;
   getNewRole(rSTOP,&role);
   
   bs.roles.resize(5,rSTOP); //default value
   bs.gamestate = sSTOPPED;
   ROS_INFO("Initial Role for Robot%d : %s",robot_id,
   role->getActiveRoleName().c_str());
}

AI::~AI()
{
}

void AI::computeAI()
{
   // Copy current base station info
   pthread_mutex_lock(&bs_info_mutex);   
   baseStationInfo bsInfo = bs;
   pthread_mutex_unlock(&bs_info_mutex);  

   pthread_mutex_lock(&robot_info_mutex);   
   robotInfo robInfo = robot;
   pthread_mutex_unlock(&robot_info_mutex);
   
   // Update my role
   if(bsInfo.roles[agent_id-1]!=role->getActiveRole()){
      getNewRole((Roles)bsInfo.roles[agent_id-1],&role);
      role->setField(field);
      role->setRosNodeHandle(node,topic_base);
   }
   
   // Determine action
   role->updateWorldModel(bsInfo,robInfo);
   role->computeAction(&ai);
   
   /*ROS_INFO("%s->%s",role->getActiveRoleName().c_str(),
                     role->getActiveActionName().c_str());*/
   
   ai_pub.publish(ai);
}

void AI::baseStationInfoCallback(const baseStationInfo::ConstPtr &msg)
{
   pthread_mutex_lock(&bs_info_mutex);
   bs = (*msg);
   pthread_mutex_unlock(&bs_info_mutex);   
}

void AI::robotInfoCallback(const robotInfo::ConstPtr &msg)
{
   pthread_mutex_lock(&robot_info_mutex);
   robot = (*msg);
   pthread_mutex_unlock(&robot_info_mutex);  
}

void AI::getNewRole(Roles role, Role **newrole)
{
   if((*newrole)) delete (*newrole); 
   switch(role){
      case rSTOP:{
         (*newrole) = new RoleStop();
         return;
      }
      case rGOALKEEPER:{
         (*newrole) = new RoleGoalKeeper();
         return;
      }
      case rSUP_STRIKER:{
         (*newrole) = new RoleSupStriker();
         return;
      }
      case rSTRIKER:{
         (*newrole) = new RoleStriker();
         return;
      }
      default: (*newrole) = new RoleStop();
               return;
   }
}

bool AI::initGameField()
{
    QString home = QString::fromStdString(getenv("HOME"));
    QString commonDir = home+QString(COMMON_PATH);
    QString fieldsDir = commonDir+QString(FIELDS_PATH);
    QString mainFile = commonDir+QString(MAINFILENAME);

    QFile file(mainFile);
    if(!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QTextStream in(&file);

    QString agent = "Robot"+QString::number(agent_id);
    QString fieldname = in.readLine();
    QString fieldPath = fieldsDir+fieldname+".view";

    ROS_INFO("System Configuration : %s in %s Field",agent.toStdString().c_str(),
    fieldname.toStdString().c_str());
    ROS_INFO("Looking for config files in %s",commonDir.toStdString().c_str());

    file.close();

    QFile file2(fieldPath);
    if(!file2.open(QIODevice::ReadOnly)) {
        ROS_ERROR("Error reading %s",fieldPath.toStdString().c_str());
        return false;
    }
    QTextStream in2(&file2);

    QString value; int counter = 0;
    while(!in2.atEnd()){
       value = in2.readLine();
       field.dimensions[counter] = value.right(value.size()-value.indexOf('=')-1).toInt();
       counter++;
    } 

   file2.close();
   return true;
}

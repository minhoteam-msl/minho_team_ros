#ifndef AI_H
#define AI_H

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/if.h>
#include <unistd.h>
#include <linux/if_ether.h>

#include "role.h"
#include "rstop.h"
#include "rgkeeper.h"
#include "rstriker.h"
#include "rsupstriker.h"
#include <sstream>
#include <ros/ros.h>
#include <minho_team_ros/baseStationInfo.h>
#include <minho_team_ros/robotInfo.h>
#include <minho_team_ros/aiInfo.h>
#include <QFile>
#include <QTextStream>
#include "Utils/types.h"

using namespace ros;
using minho_team_ros::aiInfo;
using minho_team_ros::baseStationInfo;
using minho_team_ros::robotInfo;

class AI
{
public:
   AI(ros::NodeHandle* parent, bool mode_real, int robot_id); // Constructor
   void computeAI();
   ~AI();
   void baseStationInfoCallback(const baseStationInfo::ConstPtr &msg);
   void robotInfoCallback(const robotInfo::ConstPtr &msg);
   void getNewRole(Roles role, Role **newrole);
   bool initGameField();
private:
   ros::NodeHandle* node;
   aiInfo ai;
   baseStationInfo bs;
   robotInfo robot;
   ros::Subscriber bs_sub,rob_sub;
   ros::Publisher ai_pub;
   int agent_id;
   Role *role;
   fieldDimensions field;
   std::string topic_base;
};

#endif // AI_H

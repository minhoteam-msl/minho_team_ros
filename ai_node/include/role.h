#ifndef ROLE_H
#define ROLE_H

#include "minho_team_ros/aiInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/baseStationInfo.h"
#include <ros/ros.h>
#include "Utils/types.h"
#include <iostream>

using minho_team_ros::aiInfo;
using minho_team_ros::baseStationInfo;
using minho_team_ros::robotInfo;

typedef struct vec2d{
   float x,y;
} vec2d;

class Role
{
  
public:
   Role(Roles role); // Constructor
   virtual ~Role();

   inline Roles getActiveRole(){return mRole;}
   inline Actions getActiveAction(){return mAction;}
   std::string getActiveActionName();
   inline void updateWorldModel(baseStationInfo bs,robotInfo rb)
   { mBsInfo=bs; mRobot=rb;}
   float orientationToTarget(float tarx, float tary);

   // pure virtual functions to implement in each subclass
   virtual void determineAction()=0;
   virtual void computeAction(aiInfo *ai)=0;
   virtual void setField(fieldDimensions fd)=0;
   virtual std::string getActiveRoleName()=0;
   virtual void setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base)=0;

   // public data
   Roles mRole;
   Actions mAction;
   baseStationInfo mBsInfo;
   robotInfo mRobot;
   fieldDimensions field;
   ros::NodeHandle *node; 
   aiInfo *mAI;
};

#endif // ROLE_H

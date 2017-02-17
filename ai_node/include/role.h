#ifndef ROLE_H
#define ROLE_H

#include "minho_team_ros/aiInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/baseStationInfo.h"
#include "ignition/math.hh"
#include <ros/ros.h>
#include "Utils/types.h"
#include <iostream>

using minho_team_ros::aiInfo;
using minho_team_ros::baseStationInfo;
using minho_team_ros::robotInfo;

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

   // pure virtual functions to implement in each subclass
   virtual void determineAction()=0;
   virtual void computeAction(aiInfo *ai)=0;
   virtual void setField(fieldDimensions fd)=0;
   virtual std::string getActiveRoleName()=0;

   // public data
   Roles mRole;
   Actions mAction;
   baseStationInfo mBsInfo;
   robotInfo mRobot;
   fieldDimensions field;
};

#endif // ROLE_H

#ifndef ROLE_H
#define ROLE_H

#include "minho_team_ros/aiInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/baseStationInfo.h"
#include "ignition/math.hh"
#include <ros/ros.h>
#include <iostream>

using minho_team_ros::aiInfo;
using minho_team_ros::baseStationInfo;
using minho_team_ros::robotInfo;

/// \brief defines the type of roles available to choose
typedef enum Roles{rSTOP=0,rGOALKEEPER,rSUP_STRIKER,rSTRIKER} Roles;
/// \brief defines the game states known by the system
typedef enum GameState{sSTOPPED = 0,sPARKING,sPRE_OWN_KICKOFF,
sOWN_KICKOFF,sPRE_THEIR_KICKOFF,sTHEIR_KICKOFF,sPRE_OWN_FREEKICK,
sOWN_FREEKICK,sPRE_THEIR_FREEKICK, sTHEIR_FREEKICK,sGAME_OWN_BALL,
sGAME_THEIR_BALL} GameState;
/// \brief defines the actions wich model the robots control approach
typedef enum Actions{aSTOP = 0, aSLOWMOVE, aFASTMOVE, aAPPROACHBALL,
aENGAGEBALL, aKICKBALL, aPASSBALL}Actions;

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
   virtual std::string getActiveRoleName()=0;

   // public data
   Roles mRole;
   Actions mAction;
   baseStationInfo mBsInfo;
   robotInfo mRobot;
};

#endif // ROLE_H

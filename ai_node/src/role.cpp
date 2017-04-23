#include "role.h"

Role::Role(Roles role)
{
   mRole = role;
   mAction = aSTOP;
   node = NULL;
}

Role::~Role()
{
}

std::string Role::getActiveActionName()
{
   switch(mAction){
      case aSTOP: return std::string("aSTOP");
      case aAPPROACHPOSITION: return std::string("aAPPROACHPOSITION");
      case aFASTMOVE: return std::string("aFASTMOVE");
      case aAPPROACHBALL: return std::string("aAPPROACHBALL");
      case aENGAGEBALL: return std::string("aENGAGEBALL");
      case aSLOWENGAGEBALL: return std::string("aSLOWENGAGEBALL");
      case aRECEIVEBALL: return std::string("aRECEIVEBALL");
      case aKICKBALL: return std::string("aKICKBALL");
      case aPASSBALL: return std::string("aPASSBALL");   
      case aDRIBBLEBALL: return std::string("aDRIBBLEBALL"); 
      case aHOLDBALL: return std::string("aHOLDBALL"); 
      default: return std::string("Invalid Action");
   }
}

float Role::orientationToTarget(float tarx, float tary)
{
   float orientation = atan2(tary-mRobot.robot_pose.y,tarx-mRobot.robot_pose.x)*(180.0/M_PI);
   // Normalize and adapt to our own referential
   while(orientation<0) orientation+= 360.0;
   while(orientation>360.0) orientation-= 360.0;
   if(orientation>=0.0&&orientation<90.0) 
      orientation = 360.0+orientation-90.0;
   else orientation -= 90.0;
   return orientation;
}

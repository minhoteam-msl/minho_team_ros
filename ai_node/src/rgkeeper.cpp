#include "rgkeeper.h"

RoleGoalKeeper::RoleGoalKeeper() : Role(rGOALKEEPER)
{
}

RoleGoalKeeper::~RoleGoalKeeper()
{
}

void RoleGoalKeeper::determineAction()
{
   if(mBsInfo.gamestate==sSTOPPED) mAction = aSTOP;
   else if(mBsInfo.gamestate==sPARKING){
      // Slow motion for parking
      mAction = aSLOWMOVE;
   }else { 
      // For now, do aFASTMOVE for every other game state
      mAction = aFASTMOVE;   
   }     
}

void RoleGoalKeeper::computeAction(aiInfo *ai)
{
   determineAction();
   (*ai) = aiInfo(); 
   ai->role = mRole;
   ai->action = mAction;  

   if(mAction==aSTOP)return;

   switch(mBsInfo.gamestate){
      case sPARKING:{
         // Go to parking spot
         // TODO:: replace hardcoded positions  
         if(mBsInfo.posxside) ai->target_pose.x = 0.8;
         else ai->target_pose.x = -0.8;
         ai->target_pose.y = 6.4;
         ai->target_pose.z = 180.0;
         break;
      }      
      case sPRE_OWN_KICKOFF:
      case sOWN_KICKOFF:
      case sPRE_THEIR_KICKOFF:
      case sTHEIR_KICKOFF:
      case sPRE_OWN_FREEKICK:
      case sOWN_FREEKICK:
      case sGAME_OWN_BALL:{
         // No harm situation, stay in the middle of the goalie
         // TODO:: replace hardcoded positions
         if(mBsInfo.posxside) { 
            ai->target_pose.x = 9.0;
            ai->target_pose.z = 90.0;
         } else {
            ai->target_pose.x = -9.0;
            ai->target_pose.z = 270.0;;
         }
         ai->target_pose.y = 0.0;
         break;
      }
      case sPRE_THEIR_FREEKICK:{
         // Harm situation, stay in the middle of the goalie 
         // but rotate towards the ball to maximize field of view
         // TODO:: replace hardcoded positions, head towards ball
         if(mRobot.sees_ball){
            if(mBsInfo.posxside) ai->target_pose.x = 9.0;
            else ai->target_pose.x = -9.0;
            ai->target_pose.y = 0.0; 
   
            //Compute heading to the ball
            ai->target_pose.z = atan2(mRobot.ball_position.y-mRobot.robot_pose.y,
            mRobot.ball_position.x-mRobot.robot_pose.x)*(180.0/M_PI);
            while(ai->target_pose.z<0) ai->target_pose.z+= 360.0;
            while(ai->target_pose.z>360.0) ai->target_pose.z-= 360.0;
            if(ai->target_pose.z>=0.0&&ai->target_pose.z<90.0) 
               ai->target_pose.z = 360.0+ai->target_pose.z-90.0;
            else ai->target_pose.z -= 90.0;
         } else { // If we dont see the ball, stay in the middle
            if(mBsInfo.posxside) { 
               ai->target_pose.x = 9.0;
               ai->target_pose.z = 90.0;
            } else {
               ai->target_pose.x = -9.0;
               ai->target_pose.z = 270.0;;
            }
            ai->target_pose.y = 0.0;
         }
         
         break;
      }
      case sTHEIR_FREEKICK:
      case sGAME_THEIR_BALL:{
         // Harm game situation, implement mixed game defense strategy
         // with spotting areas. If the ball has impact point inside
         // golie, move towards the point.
         // TODO:: replace hardcoded positions, implement mixed system
         if(mRobot.sees_ball){
            if(mBsInfo.posxside) { 
               ai->target_pose.x = 9.0;
               ai->target_pose.z = 90.0;
            } else {
               ai->target_pose.x = -9.0;
               ai->target_pose.z = 270.0;;
            }
            ai->target_pose.y = 0.0;
         } else { // If we dont see the ball, stay in the middle
            if(mBsInfo.posxside) { 
               ai->target_pose.x = 9.0;
               ai->target_pose.z = 90.0;
            } else {
               ai->target_pose.x = -9.0;
               ai->target_pose.z = 270.0;;
            }
            ai->target_pose.y = 0.0;
         }
         break;
      }
      default: break;
   }
}

std::string RoleGoalKeeper::getActiveRoleName()
{
   return std::string("rGOALKEEPER");
}

#include "rsupstriker.h"

/* What rSUPSTRIKER Does:
Parking - Goes to parking spot 3
Own Kickoff - Passes the ball to rSTRIKER
Their Kickoff - Stays midfield
Own Freekick - Passes the ball to rSTRIKER
Their Freekick - If existing, covers the nearest obstacle (enemy)
*/

RoleSupStriker::RoleSupStriker() : Role(rSUP_STRIKER)
{
}

RoleSupStriker::~RoleSupStriker()
{
}

void RoleSupStriker::setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base)
{

}

void RoleSupStriker::determineAction()
{
   if(mBsInfo.gamestate==sSTOPPED) { passed_after_engage = false; mAction = aSTOP; }
   else if(mBsInfo.gamestate==sPARKING||
           mBsInfo.gamestate==sPRE_THEIR_KICKOFF){
      // Slow motion for parking or precise positioning
      mAction = aAPPROACHPOSITION;
   }else if(mBsInfo.gamestate==sPRE_OWN_KICKOFF||
            mBsInfo.gamestate==sPRE_OWN_FREEKICK){
      // approach ball in order to pass it
      mAction = aAPPROACHBALL;   
   }else if(mBsInfo.gamestate==sOWN_KICKOFF||
            mBsInfo.gamestate==sOWN_FREEKICK){ 
      // When taking a foul, sup striker has to engage and pass the ball
      if(mRobot.has_ball)mAction = aPASSBALL;
      else mAction = aENGAGEBALL;
   }else {
      // For now, do aFASTMOVE for every other game state
      mAction = aFASTMOVE;
   }
}

void RoleSupStriker::computeAction(aiInfo *ai)
{
   static int stab_counter = 0;
   //mBsInfo.posxside = !mBsInfo.posxside; // Temporary, to test against running goalkeeper
   determineAction();
   (*ai) = aiInfo(); 
   ai->role = mRole;
   ai->action = mAction;  

   if(mAction==aSTOP)return;

   switch(mBsInfo.gamestate){
      case sPARKING:{
         // Go to parking spot
         if(mBsInfo.posxside) ai->target_pose.x = 2.4;
         else ai->target_pose.x = -2.4;
         ai->target_pose.y = side_line_y+0.5;
         ai->target_pose.z = 180.0;
         break;
      }      
      case sPRE_OWN_KICKOFF:{
         // Go to edge of big circle, to dont collide with the ball
         // approach ball
         passed_after_engage = false;
         stab_counter = 0;
         ai->target_pose.x = 0; 
         if(!mBsInfo.posxside){
            ai->target_pose.y = 0.5;
            ai->target_pose.z = 180.0;
         } else {
            ai->target_pose.y = -0.5;  
            ai->target_pose.z = 0.0; 
         }   

         if(mRobot.sees_ball)
            ai->target_pose.z = orientationToTarget(
            mRobot.ball_position.x,
            mRobot.ball_position.y);
         break;
      }
      case sOWN_KICKOFF:{
         float  tarx = 0.0, tary = 0.0;
         if(mRobot.has_ball) stab_counter++;
         else stab_counter = 0;
    
         float distFromMe = sqrt((mRobot.robot_pose.x-mRobot.ball_position.x)*
         (mRobot.robot_pose.x-mRobot.ball_position.x)
         +(mRobot.robot_pose.y-mRobot.ball_position.y)*
         (mRobot.robot_pose.y-mRobot.ball_position.y));
     
         if(mRobot.has_ball){
            if(stab_counter<=3) { mAction = aHOLDBALL; ai->target_pose = mRobot.robot_pose; break; }
            ai->target_pose.x = mRobot.robot_pose.x;
            ai->target_pose.y = mRobot.robot_pose.y;
            if(mBsInfo.posxside){
               tarx = 0.5; tary = center_circle_radius;
            } else {
               tarx = -0.5; tary = -center_circle_radius;
            }
            ai->target_kick_strength = 50;
            passed_after_engage = true;
            kick_spot.x = mRobot.ball_position.x;
            kick_spot.y = mRobot.ball_position.y;
          
         } else {
            stab_counter = 0;
            if(passed_after_engage && distFromMe<0.5
            && sqrt(mRobot.ball_velocity.x*mRobot.ball_velocity.x+
            mRobot.ball_velocity.y*mRobot.ball_velocity.y)<0.25){
                mAction = aENGAGEBALL; 
                // If the ball was kicked but never left the vicinity of
                // the robot, engage the ball
                tarx = mRobot.ball_position.x;
                tary = mRobot.ball_position.y;; 
                ai->target_pose.x = tarx; 
                ai->target_pose.y = tary; 
                
             } else if(passed_after_engage) { 
                mAction = aSTOP;  
             } else {
                ai->target_pose = mRobot.ball_position;
                tarx = mRobot.ball_position.x;
                tary = mRobot.ball_position.y;
             }
         }
         
         ai->target_pose.z = orientationToTarget(tarx,tary);
         
         break; 
      }
      case sPRE_THEIR_KICKOFF:
      case sTHEIR_KICKOFF:{
         // Go to middle half of left/right field and rotate towards ball
         if(mBsInfo.posxside){
            ai->target_pose.x = center_circle_radius;
            ai->target_pose.y = -side_line_y/2.0;
         } else {
            ai->target_pose.x = -center_circle_radius;
            ai->target_pose.y = side_line_y/2.0;  
         }
         
         float tarx = 0.0, tary = 0.0;
         if(mRobot.sees_ball){
            tarx = mRobot.ball_position.x;
            tary = mRobot.ball_position.y;
         }
         ai->target_pose.z = orientationToTarget(tarx,tary);
         break;
      }
      case sPRE_OWN_FREEKICK:{
         // place himself between the ball and the opposite goal, facing the other goal
         passed_after_engage = false;
         stab_counter = 0;
         if(!mBsInfo.posxside){
            ai->target_pose.x = mRobot.ball_position.x+0.5;
            ai->target_pose.y = mRobot.ball_position.y;
            ai->target_pose.z = 90.0;
         } else {
            ai->target_pose.x = mRobot.ball_position.x-0.5;
            ai->target_pose.y = mRobot.ball_position.y;
            ai->target_pose.z = 270.0; 
         }     
         break;   
      }
      case sOWN_FREEKICK:{
         float  tarx = 0.0, tary = 0.0;
         if(mRobot.has_ball) stab_counter++;
         else stab_counter = 0;
    
         float distFromMe = sqrt((mRobot.robot_pose.x-mRobot.ball_position.x)*
         (mRobot.robot_pose.x-mRobot.ball_position.x)
         +(mRobot.robot_pose.y-mRobot.ball_position.y)*
         (mRobot.robot_pose.y-mRobot.ball_position.y));
     
         if(mRobot.has_ball){
            if(stab_counter<=3) { mAction = aHOLDBALL; ai->target_pose = mRobot.robot_pose; break; }
            ai->target_pose.x = mRobot.robot_pose.x;
            ai->target_pose.y = mRobot.robot_pose.y;
            tary = mRobot.robot_pose.y;
            if(!mBsInfo.posxside) tarx = mRobot.robot_pose.x-1.0;
            else tarx = mRobot.robot_pose.x+1.0; 
            ai->target_kick_strength = 50;
            passed_after_engage = true;
            kick_spot.x = mRobot.ball_position.x;
            kick_spot.y = mRobot.ball_position.y;
         } else {
            stab_counter = 0;
            if(passed_after_engage && distFromMe<0.5
            && sqrt(mRobot.ball_velocity.x*mRobot.ball_velocity.x+
            mRobot.ball_velocity.y*mRobot.ball_velocity.y)<0.25){
                mAction = aENGAGEBALL; 
                // If the ball was kicked but never left the vicinity of
                // the robot, engage the ball
                tarx = mRobot.ball_position.x;
                tary = mRobot.ball_position.y;; 
                ai->target_pose.x = tarx; 
                ai->target_pose.y = tary; 
                
             } else if(passed_after_engage) { 
                mAction = aSTOP;  
             } else {
                ai->target_pose = mRobot.ball_position;
                tarx = mRobot.ball_position.x;
                tary = mRobot.ball_position.y;
             }
         }
         
         ai->target_pose.z = orientationToTarget(tarx,tary);
         
         break;
      }
      default: {mAction = aSTOP; break; }
   }

   ai->action = mAction;
}

std::string RoleSupStriker::getActiveRoleName()
{
   return std::string("rSUP_STRIKER");
}

void RoleSupStriker::setField(fieldDimensions fd)
{  
   field = fd;
   goal_line_x = (float)field.fieldDims.LENGTH/2000.0;
   side_line_y = (float)field.fieldDims.WIDTH/2000.0;
   small_area_x = goal_line_x-(float)field.fieldDims.AREA_LENGTH1/1000.0;
   small_area_y = (float)field.fieldDims.AREA_WIDTH1/2000.0;
   center_circle_radius = (float)field.fieldDims.CENTER_RADIUS/1000.0;
}

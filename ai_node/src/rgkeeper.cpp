#include "rgkeeper.h"

RoleGoalKeeper::RoleGoalKeeper() : Role(rGOALKEEPER)
{
}

RoleGoalKeeper::~RoleGoalKeeper()
{
}

void RoleGoalKeeper::setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base)
{
   node = parent;
   std::string topic_name = topic_base+"/goalKeeperInfo";
   gk_pub = node->advertise<goalKeeperInfo>(topic_name.c_str(),1);
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
         if(mBsInfo.posxside) ai->target_pose.x = 0.8;
         else ai->target_pose.x = -0.8;
         ai->target_pose.y = side_line_y+0.5;
         ai->target_pose.z = 180.0;
         break;
      }      
      case sPRE_OWN_KICKOFF:
      case sOWN_KICKOFF:
      case sPRE_THEIR_KICKOFF:
      case sTHEIR_KICKOFF:
      case sGAME_OWN_BALL:{
         // No harm situation, stay in the middle of the goalie
         if(mBsInfo.posxside) { 
            ai->target_pose.x = goal_line_x;
            ai->target_pose.z = 90.0;
         } else {
            ai->target_pose.x = -goal_line_x;
            ai->target_pose.z = 270.0;
         }
         ai->target_pose.y = 0.0;
         break;
      }
      case sPRE_THEIR_FREEKICK:{
         // Harm situation, stay in the middle of the goalie 
         // but rotate towards the ball to maximize field of view
         if(mRobot.sees_ball){
            if(mBsInfo.posxside) ai->target_pose.x = goal_line_x;
            else ai->target_pose.x = -goal_line_x;
            ai->target_pose.y = 0.0; 
            //Compute heading towards the ball
            ai->target_pose.z = atan2(mRobot.ball_position.y-mRobot.robot_pose.y,
            mRobot.ball_position.x-mRobot.robot_pose.x)*(180.0/M_PI);
            while(ai->target_pose.z<0) ai->target_pose.z+= 360.0;
            while(ai->target_pose.z>360.0) ai->target_pose.z-= 360.0;
            if(ai->target_pose.z>=0.0&&ai->target_pose.z<90.0) 
               ai->target_pose.z = 360.0+ai->target_pose.z-90.0;
            else ai->target_pose.z -= 90.0;
         } else { // If we dont see the ball, stay in the middle
            if(mBsInfo.posxside) { 
               ai->target_pose.x = goal_line_x;
               ai->target_pose.z = 90.0;
            } else {
               ai->target_pose.x = -goal_line_x;
               ai->target_pose.z = 270.0;
            }
            ai->target_pose.y = 0.0;
         }
         
         break;
      }

      case sPRE_OWN_FREEKICK: // remove later
      case sOWN_FREEKICK:
      case sTHEIR_FREEKICK:
      case sGAME_THEIR_BALL:{
         // Harm game situation, implement mixed game defense strategy
         // with spotting areas. If the ball has impact point inside
         // golie, move towards the point.
         if(mRobot.sees_ball){
            if(!preditImpactPosition()){
               if(mBsInfo.posxside) { 
                  if(mRobot.ball_position.y>=-1.5&&
                     mRobot.ball_position.y<=1.5&&
                     mRobot.ball_position.x>=small_area_x-0.1&&
                     mRobot.ball_position.x<=goal_line_x){
                     // ball inside small area, stay right in front of it
                     ai->target_pose.x = goal_line_x;
                     ai->target_pose.y = mRobot.ball_position.y;
                     if(ai->target_pose.y>0.8)ai->target_pose.y=0.8;
                     if(ai->target_pose.y<-0.8)ai->target_pose.y=-0.8;
                     ai->target_pose.z = 90.0;
                  }else{
                     ai->target_pose = spot_areas[0].posr; //default
                     for(int i=0;i<spot_areas.size();i++){
                        if(isInsideSpotAreaRight(i)){
                           ai->target_pose = spot_areas[i].posr;  
                        }
                     }
                  }
               } else {
                  if(mRobot.ball_position.y>=-1.5&&
                     mRobot.ball_position.y<=1.5&&
                     mRobot.ball_position.x<=-small_area_x+0.1&&
                     mRobot.ball_position.x>=-goal_line_x){
                     // ball inside small area, stay right in front of it
                     ai->target_pose.x = -goal_line_x;
                     ai->target_pose.y = mRobot.ball_position.y;
                     if(ai->target_pose.y>0.8)ai->target_pose.y=0.8;
                     if(ai->target_pose.y<-0.8)ai->target_pose.y=-0.8;
                     ai->target_pose.z = 270.0;
                  }else{
                     ai->target_pose = spot_areas[0].posl; //default
                     for(int i=0;i<spot_areas.size();i++){
                        if(isInsideSpotAreaLeft(i)){
                           ai->target_pose = spot_areas[i].posl;  
                        }
                     }
                  }
               }
               //Compute heading towards the ball
               ai->target_pose.z = 
               atan2(mRobot.ball_position.y-mRobot.robot_pose.y,
               mRobot.ball_position.x-mRobot.robot_pose.x)*(180.0/M_PI);
               while(ai->target_pose.z<0) ai->target_pose.z+= 360.0;
               while(ai->target_pose.z>360.0) ai->target_pose.z-= 360.0;
               if(ai->target_pose.z>=0.0&&ai->target_pose.z<90.0) 
                  ai->target_pose.z = 360.0+ai->target_pose.z-90.0;
               else ai->target_pose.z -= 90.0;
            }else{
               //compute stuff
               ai->target_pose = impact;
               if(mBsInfo.posxside) ai->target_pose.z = 90.0;
               else ai->target_pose.z = 270.0;

               goalKeeperInfo info;
               info.robot_info = mRobot;
               info.impact_zone = impact;
               gk_pub.publish(info);
            }
         } else { // If we dont see the ball, stay in the middle
            if(mBsInfo.posxside) { 
               ai->target_pose.x = goal_line_x;
               ai->target_pose.z = 90.0;
            } else {
               ai->target_pose.x = -goal_line_x;
               ai->target_pose.z = 270.0;
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

void RoleGoalKeeper::setField(fieldDimensions fd)
{  
   field = fd;
   goal_line_x = (float)field.fieldDims.LENGTH/2000.0;
   side_line_y = (float)field.fieldDims.WIDTH/2000.0;
   small_area_x = goal_line_x-(float)field.fieldDims.AREA_LENGTH1/1000.0;

   // Fill in spot areas
   spot_areas.clear();
   spot_areas.resize(3);
   spotArea section;

   //Fill center section
   section.range[0] = (0.0); section.range[1] = (goal_line_x);
   section.range[2] = (-1.5); section.range[3] = (1.5);
   section.posr.x = (goal_line_x); section.posr.y = (0.0);
   section.posl.x = (-goal_line_x); section.posl.y = (0.0);
   spot_areas[0] = section;

   //Fill near upper section
   section.range[0] =(goal_line_x-4.0); section.range[1] = (goal_line_x);
   section.range[2] = (-side_line_y); section.range[3] = (-1.5);
   section.posr.x = (goal_line_x-0.3); section.posr.y = (-0.8);
   section.posl.x = (-goal_line_x+0.3); section.posl.y = (0.8);
   spot_areas[1] = section;
   
   //Fill near lower section
   section.range[0] = (goal_line_x-4.0); section.range[1] = (goal_line_x);
   section.range[2] = (1.5); section.range[3] = (side_line_y);
   section.posr.x = (goal_line_x-0.3); section.posr.y = (0.8);
   section.posl.x = (-goal_line_x+0.3); section.posl.y = (-0.8);
   spot_areas[2] = section;
}

bool RoleGoalKeeper::isInsideSpotAreaRight(int idx)
{
   spotArea sarea = spot_areas[idx];
   if(mRobot.ball_position.x>=sarea.range[0]&&
      mRobot.ball_position.x<=sarea.range[1]&&
      mRobot.ball_position.y>=sarea.range[2]&&
      mRobot.ball_position.y<=sarea.range[3]) return true;
   return false;
}
bool RoleGoalKeeper::isInsideSpotAreaLeft(int idx)
{
   spotArea sarea = spot_areas[idx];
   if(mRobot.ball_position.x>=-sarea.range[1]&&
      mRobot.ball_position.x<=-sarea.range[0]&&
      mRobot.ball_position.y>=-sarea.range[3]&&
      mRobot.ball_position.y<=-sarea.range[2]) return true;
   return false;
}

bool RoleGoalKeeper::dangerousBall()
{
   if(mBsInfo.posxside){
      if(mRobot.ball_velocity.x<0.3) return false;
      else return true;
   }else{
      if(mRobot.ball_velocity.x>-0.3) return false;
      else return true;
   } 
}

bool RoleGoalKeeper::preditImpactPosition()
{
   if(!dangerousBall()) return false;

   bool potentialGoal = false, lowShot = false;
   float g = 9.8;
   float Xi = mRobot.ball_position.x, Yi = mRobot.ball_position.y;
   float Zi = mRobot.ball_position.z, Vz = mRobot.ball_velocity.z;
   if(Zi<0.3 && fabs(Vz)<0.25){ // ball to low and slow to 
   // be worth to do bounce prediction
      potentialGoal=lowShot=true;
   } else { // bounce prediction needed
   }


   if(potentialGoal&&lowShot){ // linear movement prediction
      float m = mRobot.ball_velocity.y/mRobot.ball_velocity.x;
      float b = Yi-m*Xi;
      float targetX = goal_line_x-0.3;
      if(!mBsInfo.posxside) targetX = -goal_line_x+0.3;
      float yimpact= m*targetX+b;
      if(yimpact<=1.0&&yimpact>=-1.0){
         impact.x = targetX;
         impact.y = yimpact;
         impact.z = 0.0;
         return true;
      } else return false;
   }
}

bool RoleGoalKeeper::crossedGoalLine(float ximpact)
{
   if(mBsInfo.posxside){
      if(ximpact>=(goal_line_x-0.25)&&ximpact<=(goal_line_x+0.5)) return true;
      else return false;
   }else{
      if(ximpact<=(-goal_line_x+0.25)&&ximpact>=(-goal_line_x-0.5)) return true;
      else return false;
   }
}

bool RoleGoalKeeper::intrestingEventHappened(double x, double y, double xi)
{
   if(y>=side_line_y || y<=-side_line_y) return true; 
   if(mBsInfo.posxside){
      if(x-xi<=-0.3) return true; // ball going towards keeper
      else if(x>=goal_line_x-0.25 && (y>=1.0 || y<=-1.0)) return true; // shot out of goalie posts
      else if(x<=goal_line_x+0.5 && x>=goal_line_x-0.25 && y<=1.0 && y>=-1.0) return true;
      else return false;   
   } 
   
   return false;
}

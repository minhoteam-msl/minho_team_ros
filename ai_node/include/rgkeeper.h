#ifndef RGKEEPER_H
#define RGKEEPER_H

#include "role.h"
#include "minho_team_ros/pose.h"
#include "minho_team_ros/goalKeeperInfo.h"

typedef struct spotArea{
   float range[4];
   minho_team_ros::pose posr;
   minho_team_ros::pose posl;
}spotArea;

using minho_team_ros::pose;
using minho_team_ros::goalKeeperInfo;

class RoleGoalKeeper : public Role
{
  
public:
   RoleGoalKeeper(); // Constructor
   ~RoleGoalKeeper();
   
   virtual void determineAction();
   virtual void computeAction(aiInfo *ai);
   virtual std::string getActiveRoleName();
   virtual void setField(fieldDimensions fd);
   virtual void setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base);
private:
   float goal_line_x;
   float side_line_y;
   float small_area_x;

   void goToMiddleOfGoalie();
   bool isInsideSpotAreaRight(int idx);
   bool isInsideSpotAreaLeft(int idx);   
   void computeGoalPreventionLocation();
   bool dangerousBall();
   bool predictImpactPosition(); // This will probably moved later
   bool crossedGoalLine(float ximpact);
   bool intrestingEventHappened(double x, double y, double xi);
   
   // somewhere else
   // Spotting areas
   // 0 - xmin, 1 - xmax, 2 - ymin, 3 - ymax
   // for right side field
   std::vector<spotArea> spot_areas;   
   minho_team_ros::pose impact;

   //remove later
   ros::Publisher gk_pub;
};

#endif

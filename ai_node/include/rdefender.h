#ifndef RSTRIKER_H
#define RSTRIKER_H

#include "role.h"

class RoleDefender : public Role
{
  
public:
   RoleDefender(); // Constructor
   ~RoleDefender();
   
   virtual void determineAction();
   virtual void computeAction(aiInfo *ai);
   virtual std::string getActiveRoleName();
   virtual void setField(fieldDimensions fd);
   virtual void setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base);
private:
   float goal_line_x;
   float side_line_y;
   float small_area_x,small_area_y;
   float big_area_x, big_area_y;
   float penalty_x;
   float center_circle_radius;
   
};

#endif

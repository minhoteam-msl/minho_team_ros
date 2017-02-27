#ifndef RSTRIKER_H
#define RSTRIKER_H

#include "role.h"

typedef struct vec2d{
   float x,y;
} vec2d;

class RoleStriker : public Role
{
  
public:
   RoleStriker(); // Constructor
   ~RoleStriker();

   virtual void determineAction();
   virtual void computeAction(aiInfo *ai);
   virtual std::string getActiveRoleName();
   virtual void setField(fieldDimensions fd);
   virtual void setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base);
private:
   float goal_line_x;
   float side_line_y;
   float small_area_x,small_area_y;
   float center_circle_radius;
   bool kicked_after_recv;
   std::vector<vec2d> collisionPoints;
   
   bool pathClearForShooting(float tarx, float *tary);
   vec2d getNearestPointToPath(float a, float c, vec2d point);
   void getObstaclesDisplacement(float *left,float *right);
   // remove later   
   int getKickStrength(float distance);
   std::vector<float> kick_dists, kick_strengths;
   
};

#endif

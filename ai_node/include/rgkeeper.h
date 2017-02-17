#ifndef RGKEEPER_H
#define RGKEEPER_H

#include "role.h"

class RoleGoalKeeper : public Role
{
  
public:
   RoleGoalKeeper(); // Constructor
   ~RoleGoalKeeper();

   virtual void determineAction();
   virtual void computeAction(aiInfo *ai);
   virtual std::string getActiveRoleName();
   virtual void setField(fieldDimensions fd);
private:
   float goal_line_X;
   float side_line_y;
};

#endif

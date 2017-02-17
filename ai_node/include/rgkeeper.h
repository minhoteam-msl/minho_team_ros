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
private:
};

#endif

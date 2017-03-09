#ifndef RSTOP_H
#define RSTOP_H

#include "role.h"

class RoleStop : public Role
{
  
public:
   RoleStop(); // Constructor
   ~RoleStop();

   virtual void determineAction();
   virtual void computeAction(aiInfo *ai);
   virtual std::string getActiveRoleName();
   virtual void setField(fieldDimensions fd);
   virtual void setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base);
};

#endif

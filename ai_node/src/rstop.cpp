#include "rstop.h"

/* What rSTOP Does:
All states - Remains stopped
*/

RoleStop::RoleStop() : Role(rSTOP)
{
}

RoleStop::~RoleStop()
{
}

void RoleStop::setRosNodeHandle(ros::NodeHandle *parent, std::string topic_base)
{
}

void RoleStop::determineAction()
{
   mAction = aSTOP;  
}

void RoleStop::computeAction(aiInfo *ai)
{
   determineAction();
   (*ai) = aiInfo(); 
   ai->role = mRole;
   ai->action = mAction;  
}

std::string RoleStop::getActiveRoleName()
{
   return std::string("rSTOP");
}

void RoleStop::setField(fieldDimensions fd)
{  
   field = fd;
}

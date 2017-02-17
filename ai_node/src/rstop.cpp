#include "rstop.h"

RoleStop::RoleStop() : Role(rSTOP)
{
}

RoleStop::~RoleStop()
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

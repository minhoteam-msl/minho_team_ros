#include "role.h"

Role::Role(Roles role)
{
   mRole = role;
   mAction = aSTOP;
}

Role::~Role()
{
}

std::string Role::getActiveActionName()
{
   switch(mAction){
      case aSTOP: return std::string("aSTOP");
      case aSLOWMOVE: return std::string("aSLOWMOVE");
      case aFASTMOVE: return std::string("aFASTMOVE");
      case aAPPROACHBALL: return std::string("aAPPROACHBALL");
      case aENGAGEBALL: return std::string("aENGAGEBALL");
      case aKICKBALL: return std::string("aKICKBALL");
      case aPASSBALL: return std::string("aPASSBALL");   
      default: return std::string("Invalid Action");
   }
}

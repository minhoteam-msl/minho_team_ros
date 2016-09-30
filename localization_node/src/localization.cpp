#include "localization.h"

Localization::Localization(ros::NodeHandle *par , QObject *parent) : QObject(parent)
{
   initVariables();
   
   //#### Initialize major components ####
   //##################################### 
   processor = new ImageProcessor(false); 
   confserver = new ConfigServer(par); 
   //##################################### 
   
   //## Setup connections to confserver ##
   //##################################### 
   connect(confserver,SIGNAL(stopImageAssigning()),this,SLOT(stopImageAssigning()));
   connect(confserver,SIGNAL(changedImageRequest(uint8_t)),this,SLOT(changeImageAssigning(uint8_t)));
   //#####################################   
}

Localization::~Localization()
{

}

void Localization::initVariables()
{
   qRegisterMetaType<uint8_t>("uint8_t");
   assigning_images = false;   
}

void Localization::stopImageAssigning()
{
   assigning_images = false; 
}

void Localization::changeImageAssigning(uint8_t type)
{
   assigning_type = type;  
   assigning_images = true;    
}

void Localization::hardwareCallback(const hardwareInfo::ConstPtr &msg)
{
}

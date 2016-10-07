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
   connect(confserver,SIGNAL(changedMirrorConfiguration(mirrorConfig::ConstPtr)),this,SLOT(changeMirrorConfiguration(mirrorConfig::ConstPtr)));
   connect(confserver,SIGNAL(changedLutConfiguration(visionHSVConfig::ConstPtr)),this,SLOT(changeLookUpTableConfiguration(visionHSVConfig::ConstPtr)));
   //#####################################
   
   
   //Test
   test_image = imread(QString(imgFolderPath+"1.png").toStdString());
   test = new QTimer();
   connect(test,SIGNAL(timeout()),this,SLOT(testfunc()));
   test->start(30);
}

Localization::~Localization()
{

}

void Localization::initVariables()
{
   qRegisterMetaType<uint8_t>("uint8_t");
   qRegisterMetaType<mirrorConfig::ConstPtr>("mirrorConfig::ConstPtr");
   qRegisterMetaType<visionHSVConfig::ConstPtr>("visionHSVConfig::ConstPtr");
   QString home = QString::fromStdString(getenv("HOME"));
   QString cfgDir = home+QString(configFolderPath);
   imgFolderPath = cfgDir+QString(imageFolderPath);
   
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

void Localization::changeLookUpTableConfiguration(visionHSVConfig::ConstPtr msg)
{
   ROS_INFO("New configuration of Look Up Table will be set");
}
void Localization::changeMirrorConfiguration(mirrorConfig::ConstPtr msg)
{
   ROS_INFO("New configuration of mirror will be set");
}
   
void Localization::hardwareCallback(const hardwareInfo::ConstPtr &msg)
{
   //Process hardware information
}

void Localization::testfunc()
{
   if(assigning_images){
      confserver->assignImage(&test_image);   
   }
}

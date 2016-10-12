#include "localization.h"

Localization::Localization(ros::NodeHandle *par , QObject *parent) : QObject(parent)
{
   initVariables();
   
   //#### Initialize major components ####
   //##################################### 
   processor = new ImageProcessor(false); 
   confserver = new ConfigServer(par); 
   confserver->setOmniVisionConf(processor->getMirrorConfAsMsg(),processor->getVisionConfAsMsg(),processor->getImageConfAsMsg());
   parentTimer = new QTimer();
   requiredTiming = 33;
   connect(parentTimer,SIGNAL(timeout()),this,SLOT(discoverWorldModel()));
   //##################################### 
   
   //## Setup connections to confserver ##
   //##################################### 
   connect(confserver,SIGNAL(stopImageAssigning()),this,SLOT(stopImageAssigning()));
   connect(confserver,SIGNAL(changedImageRequest(uint8_t)),this,SLOT(changeImageAssigning(uint8_t)));
   connect(confserver,SIGNAL(changedMirrorConfiguration(mirrorConfig::ConstPtr)),this,SLOT(changeMirrorConfiguration(mirrorConfig::ConstPtr)));
   connect(confserver,SIGNAL(changedLutConfiguration(visionHSVConfig::ConstPtr)),this,SLOT(changeLookUpTableConfiguration(visionHSVConfig::ConstPtr)));
   connect(confserver,SIGNAL(changedImageConfiguration(imageConfig::ConstPtr)),this,SLOT(changeImageConfiguration(imageConfig::ConstPtr)));
   //#####################################
   
   //TEST
   buffer = imread(QString(imgFolderPath+"1.png").toStdString());
   parentTimer->start(33);
}

Localization::~Localization()
{

}

void Localization::discoverWorldModel() // Main Funcition
{
   bool have_image = true;
   int timing = 0;
   parentTimer->stop();
   QTime measure;
   measure.start();
   
   // Localization code
   if(have_image){
      //Publish information   
      
   }  
   
   if(!have_image) parentTimer->start(1);
   else {
      //Send image to confserver if requested
      if(assigning_images){
         if(assigning_type==IMG_RAW) {
            confserver->assignImage(&buffer);  
         } else {
            processed = buffer.clone();
            processor->getSegmentedImage(&processed);  
            confserver->assignImage(&processed);     
         }
      }
   
      //Compensate time took in the function
      timing = requiredTiming-measure.elapsed();
      if(timing<0) timing = requiredTiming;
      parentTimer->start(timing);
   }
   
}
   
void Localization::initVariables()
{
   qRegisterMetaType<uint8_t>("uint8_t");
   qRegisterMetaType<mirrorConfig::ConstPtr>("mirrorConfig::ConstPtr");
   qRegisterMetaType<visionHSVConfig::ConstPtr>("visionHSVConfig::ConstPtr");
   qRegisterMetaType<imageConfig::ConstPtr>("imageConfig::ConstPtr");
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
   ROS_WARN("New configuration of Look Up Table will be set.");
   //Stop processing mechanism
   processor->updateLabelLutConf(FIELD,msg->field);
   processor->updateLabelLutConf(LINE,msg->line);
   processor->updateLabelLutConf(BALL,msg->ball);
   processor->updateLabelLutConf(OBSTACLE,msg->obstacle);
   processor->generateLookUpTable();
   if(processor->writeLookUpTable())ROS_INFO("New %s saved!",lutFileName);
   //Start processing mechanism
}
void Localization::changeMirrorConfiguration(mirrorConfig::ConstPtr msg)
{
   ROS_WARN("New configuration of mirror will be set.");
   //Stop processing mechanism
   processor->updateDists(msg->max_distance,msg->step,msg->pixel_distances);
   processor->generateMirrorConfiguration();
   if(processor->writeMirrorConfig())ROS_INFO("New %s saved!",mirrorFileName);
   //Start processing mechanism
}

void Localization::changeImageConfiguration(imageConfig::ConstPtr msg)
{
   ROS_WARN("New configuration of image will be set.");
   //Stop processing mechanism
   processor->setCenter(msg->center_x,msg->center_y,msg->tilt);
   processor->generateMirrorConfiguration();
   if(processor->writeImageConfig())ROS_INFO("New %s saved!",imageFileName);
   //Start processing mechanism
}
void Localization::hardwareCallback(const hardwareInfo::ConstPtr &msg)
{
   //Process hardware information
}

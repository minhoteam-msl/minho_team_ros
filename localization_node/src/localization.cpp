#include "localization.h"

Localization::Localization(ros::NodeHandle *par , QObject *parent) : QObject(parent)
{
   initVariables();
   
   //#### Initialize major components ####
   //##################################### 
   processor = new ImageProcessor(false); 
   confserver = new ConfigServer(par); 
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
   //Stop processing mechanism
   labelConfiguration label;
   label.lb_calib[H][MIN] = msg->field.H.min; label.lb_calib[H][MAX] = msg->field.H.max;
   label.lb_calib[S][MIN] = msg->field.S.min; label.lb_calib[S][MAX] = msg->field.S.max;
   label.lb_calib[V][MIN] = msg->field.V.min; label.lb_calib[V][MAX] = msg->field.V.max;
   processor->updateLabelLutConf(FIELD,label);
   label.lb_calib[H][MIN] = msg->line.H.min; label.lb_calib[H][MAX] = msg->line.H.max;
   label.lb_calib[S][MIN] = msg->line.S.min; label.lb_calib[S][MAX] = msg->line.S.max;
   label.lb_calib[V][MIN] = msg->line.V.min; label.lb_calib[V][MAX] = msg->line.V.max;
   processor->updateLabelLutConf(LINE,label);
   label.lb_calib[H][MIN] = msg->ball.H.min; label.lb_calib[H][MAX] = msg->ball.H.max;
   label.lb_calib[S][MIN] = msg->ball.S.min; label.lb_calib[S][MAX] = msg->ball.S.max;
   label.lb_calib[V][MIN] = msg->ball.V.min; label.lb_calib[V][MAX] = msg->ball.V.max;
   processor->updateLabelLutConf(BALL,label);
   label.lb_calib[H][MIN] = msg->obstacle.H.min; label.lb_calib[H][MAX] = msg->obstacle.H.max;
   label.lb_calib[S][MIN] = msg->obstacle.S.min; label.lb_calib[S][MAX] = msg->obstacle.S.max;
   label.lb_calib[V][MIN] = msg->obstacle.V.min; label.lb_calib[V][MAX] = msg->obstacle.V.max;
   processor->updateLabelLutConf(OBSTACLE,label);
   processor->generateLookUpTable();
   if(processor->writeLookUpTable())ROS_INFO("New %s saved!",lutFileName);
   //Start processing mechanism
}
void Localization::changeMirrorConfiguration(mirrorConfig::ConstPtr msg)
{
   ROS_INFO("New configuration of mirror will be set");
   //Stop processing mechanism
   processor->updateDists(msg->max_distance,msg->step,msg->pixel_distances);
   processor->generateMirrorConfiguration();
   if(processor->writeMirrorConfig())ROS_INFO("New %s saved!",mirrorFileName);
   //Start processing mechanism
}
   
void Localization::hardwareCallback(const hardwareInfo::ConstPtr &msg)
{
   //Process hardware information
}

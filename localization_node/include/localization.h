#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include <QObject>
#include "imageprocessor.h"
#include "configserver.h"
#include "minho_team_ros/hardwareInfo.h"
#include "minho_team_ros/robotInfo.h"
#include "localization.h"

using namespace ros;
using namespace cv;
using minho_team_ros::hardwareInfo; //Namespace for hardware information msg - SUBSCRIBING
using minho_team_ros::robotInfo; //Namespace for vision information msg - PUBLISHING

class Localization : public QObject
{
   Q_OBJECT
public:
   explicit Localization(ros::NodeHandle *par, QObject *parent = 0); // Constructor
   ~Localization();

private:
   //Major components
   ImageProcessor *processor;
   ConfigServer *confserver;
   QTimer *parentTimer;
   Mat buffer, processed;
   int requiredTiming;
   //Confserver variables
   bool assigning_images;
   uint8_t assigning_type;
   QString imgFolderPath;
private slots:
   void initVariables();
   void stopImageAssigning();
   void changeImageAssigning(uint8_t type);
   void changeLookUpTableConfiguration(visionHSVConfig::ConstPtr msg);
   void changeMirrorConfiguration(mirrorConfig::ConstPtr msg);
   void changeImageConfiguration(imageConfig::ConstPtr msg);
public slots:
   void hardwareCallback(const hardwareInfo::ConstPtr &msg);   
   void discoverWorldModel(); // Main Funcition
};

#endif // LOCALIZATION_H

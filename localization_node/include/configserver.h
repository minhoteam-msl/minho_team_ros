#ifndef CONFIGSERVER_H
#define CONFIGSERVER_H

#include <QObject>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "minho_team_ros/imgRequest.h"
#include "minho_team_ros/mirrorConfig.h"
#include "minho_team_ros/visionHSVConfig.h"
#include "minho_team_ros/imageConfig.h"
#include "minho_team_ros/requestOmniVisionConf.h"
#include "types.h"
#include "ros/topic_manager.h"
#include <QTime>

// Config messages
using minho_team_ros::imgRequest;
using minho_team_ros::mirrorConfig;
using minho_team_ros::visionHSVConfig;
using minho_team_ros::imageConfig;

using namespace cv;
using namespace std;
class ConfigServer : public QObject
{
   Q_OBJECT
public:
   explicit ConfigServer(ros::NodeHandle *par, QObject *parent = 0); // Constructor
   ~ConfigServer();
   void assignImage(Mat *source);
   void setOmniVisionConf(mirrorConfig mirrmsg,visionHSVConfig vismsg,imageConfig imgmsg);
private:
   // VARIABLES
   // ##############################################################
   // ##############################################################
   // Subscriptions and Requests monitor
   int n_subscriptions_;
   QTimer *watchdog_timer_;
   QTimer *image_timer_;
   int configured_frequency_;
   bool multiple_sends_;
   
   // ROS
   ros::NodeHandle *parent_; // ROS node parent pointer 
   image_transport::ImageTransport *it_; // To send images through ROS
   //SEND
   image_transport::Publisher image_pub_; // Publisher for image_transport
   //RECEIVE
   ros::Subscriber img_req_sub_; // Subscriber for image requests (imgRequest.msg)
   ros::Subscriber mirror_sub_; // Subscriber for mirror configuration (mirrorConfig.msg)
   ros::Subscriber image_sub_; // Subscriber for image configuration (imageConfig.msg)
   ros::Subscriber vision_sub_; // Subscriber for vision[lut] configuration (visionHSVConfig.msg)
   //SERVICES AND DATA
   ros::ServiceServer service_; // Service to relay the configuration
   mirrorConfig mirrorConfmsg; // Data to hold current mirrorConfig
   visionHSVConfig visionConfmsg; // Data to hold current visionHSVConfig
   imageConfig imageConfmsg; // Data to hold current imageConfig
   sensor_msgs::ImagePtr image_;
   cv::Mat mock_image;
private slots:
   void getSubscribers();
   void processImageRequest(const imgRequest::ConstPtr &msg);
   void processMirrorConfig(const mirrorConfig::ConstPtr &msg);
   void processVisionConfig(const visionHSVConfig::ConstPtr &msg);
   void processImageConfig(const imageConfig::ConstPtr &msg);
   bool omniVisionConfService(minho_team_ros::requestOmniVisionConf::Request &req,
                              minho_team_ros::requestOmniVisionConf::Response &res);
   void postRequestedImage();
   void init_mock_image();
signals:
   void stopImageAssigning();
   void changedImageRequest(uint8_t type);
   void changedMirrorConfiguration(mirrorConfig::ConstPtr msg);
   void changedLutConfiguration(visionHSVConfig::ConstPtr msg);
   void changedImageConfiguration(imageConfig::ConstPtr msg);
   
};

#endif // CONFIGSERVER_H

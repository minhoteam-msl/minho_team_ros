#ifndef CONFIGSERVER_H
#define CONFIGSERVER_H

#include <QObject>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "blackflydriver.h"
#include "minho_team_ros/imgRequest.h"
#include "types.h"
#include "ros/topic_manager.h"
#include <QTime>

// Config messages
using minho_team_ros::imgRequest;

using namespace cv;
using namespace std;
class configServer : public QObject
{
   Q_OBJECT
public:
   explicit configServer(ros::NodeHandle *par, QObject *parent = 0); // Constructor
   ~configServer();
   void assignImage(Mat *source);
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
   image_transport::ImageTransport *it_;
   image_transport::Publisher image_pub_;
   ros::NodeHandle *parent_;
   ros::Subscriber img_req_sub_;
   sensor_msgs::ImagePtr image_;
   cv::Mat mock_image;
private slots:
   void getSubscribers();
   void processImageRequest(const imgRequest::ConstPtr &msg);
   void postRequestedImage();
   void init_mock_image();
signals:
   void stopImageAssigning();
   void changedImageRequest(uint8_t type);
   
};

#endif // CONFIGSERVER_H

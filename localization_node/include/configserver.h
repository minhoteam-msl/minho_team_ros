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
   int configured_frequency_;
   // ROS 
   image_transport::ImageTransport *it_;
   image_transport::Publisher image_pub_;
   ros::NodeHandle *parent_;
   ros::Subscriber img_req_sub_;
   sensor_msgs::ImagePtr image_;
private slots:
   void getSubscribers();
   void processImageRequest(const imgRequest::ConstPtr &msg);
};

#endif // CONFIGSERVER_H

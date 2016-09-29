#include "configserver.h"

// Constructor of imaging TCP server
configServer::configServer(ros::NodeHandle *par , QObject *parent) : QObject(parent)
{
   n_subscriptions_ = 0;  
   configured_frequency_ = 0;
   parent_ = par;
   watchdog_timer_ = new QTimer(); 
   connect(watchdog_timer_,SIGNAL(timeout()),this,SLOT(getSubscribers()));
   // Setup ROS publishers and subscribers
   it_ = new image_transport::ImageTransport(*par);
   img_req_sub_ = par->subscribe("controlInfo", 
                                         1000, 
                                         &configServer::processImageRequest,
                                         this);
   image_pub_ = it_->advertise("camera", 1);
   
   
   watchdog_timer_->start(100);
}


configServer::~configServer()
{
   watchdog_timer_->stop();
   image_pub_.shutdown();
}

void configServer::assignImage(Mat *source)
{
   image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *source).toImageMsg();
}

void configServer::getSubscribers()
{
   n_subscriptions_ = image_pub_.getNumSubscribers();
}

void configServer::processImageRequest(const imgRequest::ConstPtr &msg)
{
   ROS_INFO("Received an image request");  
}

#include "configserver.h"

// Constructor of imaging TCP server
configServer::configServer(ros::NodeHandle *par , QObject *parent) : QObject(parent)
{
   n_subscriptions_ = 0;  
   multiple_sends_ = false;
   configured_frequency_ = 1;
   parent_ = par;
   watchdog_timer_ = new QTimer(); 
   image_timer_ = new QTimer();
   connect(watchdog_timer_,SIGNAL(timeout()),this,SLOT(getSubscribers()));
   connect(image_timer_,SIGNAL(timeout()),this,SLOT(postRequestedImage()));
   init_mock_image();
   // Setup ROS publishers and subscribers
   it_ = new image_transport::ImageTransport(*par);
   img_req_sub_ = par->subscribe("imgRequest", 
                                  1000, 
                                  &configServer::processImageRequest,
                                  this);
   image_pub_ = it_->advertise("camera", 1);
   watchdog_timer_->start(100);
}


configServer::~configServer()
{
   watchdog_timer_->stop();
   image_timer_->stop();
   image_pub_.shutdown();
}

void configServer::assignImage(Mat *source)
{
   image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *source).toImageMsg();
   image_timer_->start(1000.0/(float)configured_frequency_);
}

void configServer::getSubscribers()
{
   n_subscriptions_ = image_pub_.getNumSubscribers();
   if(n_subscriptions_<=0 && image_timer_->isActive()){
       emit stopImageAssigning();
       image_timer_->stop();
       ROS_INFO("Stopped image assigning");
   }
}

void configServer::processImageRequest(const imgRequest::ConstPtr &msg)
{
   emit changedImageRequest(msg->type); // Tells the upper class to assign Images
   configured_frequency_ = msg->frequency;
   multiple_sends_ = msg->is_multiple;
   ROS_INFO("Started image assigning");
}

void configServer::postRequestedImage()
{
   image_timer_->stop();
   image_pub_.publish(image_); 
   if(multiple_sends_)image_timer_->start(1000.0/(float)configured_frequency_);
   else { emit stopImageAssigning(); }
}

void configServer::init_mock_image()
{
   mock_image = cv::Mat(480,480,CV_8UC3,cv::Scalar(100,100,100));
   cv::putText(mock_image,"NO IMAGE",Point(10,240),FONT_HERSHEY_SIMPLEX,3.0,cv::Scalar(0,255,0),4);
   image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mock_image).toImageMsg();
}

#include "configserver.h"

ConfigServer::ConfigServer(ros::NodeHandle *par , QObject *parent) : QObject(parent)
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
                                  &ConfigServer::processImageRequest,
                                  this);
   image_pub_ = it_->advertise("camera", 1);
   mirror_sub_ = par->subscribe("mirrorConfig", 
                                  1000, 
                                  &ConfigServer::processMirrorConfig,
                                  this);
    vision_sub_ = par->subscribe("visionHSVConfig", 
                                  1000, 
                                  &ConfigServer::processVisionConfig,
                                  this);
   
   watchdog_timer_->start(100);
   ROS_WARN("ConfigServer running on ROS ...");
}


ConfigServer::~ConfigServer()
{
   watchdog_timer_->stop();
   image_timer_->stop();
   image_pub_.shutdown();
}

void ConfigServer::assignImage(Mat *source)
{
   image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *source).toImageMsg();
   if(!image_timer_->isActive())image_timer_->start(1000.0/(float)configured_frequency_);
}

void ConfigServer::getSubscribers()
{
   n_subscriptions_ = image_pub_.getNumSubscribers();
   if(n_subscriptions_<=0 && image_timer_->isActive()){
       emit stopImageAssigning();
       image_timer_->stop();
   }
}

void ConfigServer::processImageRequest(const imgRequest::ConstPtr &msg)
{
   emit changedImageRequest(msg->type); // Tells the upper class to assign Images
   configured_frequency_ = msg->frequency;
   if(configured_frequency_<=0) configured_frequency_ = 1;
   if(configured_frequency_>30) configured_frequency_ = 30;
   multiple_sends_ = msg->is_multiple;
   if(!multiple_sends_) configured_frequency_ = 30; // This is to send the single image as fast as possible
}

void ConfigServer::processMirrorConfig(const mirrorConfig::ConstPtr &msg)
{
   emit changedMirrorConfiguration(msg);
}

void ConfigServer::processVisionConfig(const visionHSVConfig::ConstPtr &msg)
{
   emit changedLutConfiguration(msg);
}
   
void ConfigServer::postRequestedImage()
{
   image_timer_->stop();
   image_pub_.publish(image_); 
   if(multiple_sends_)image_timer_->start(1000.0/(float)configured_frequency_);
   else { emit stopImageAssigning();}
}

void ConfigServer::init_mock_image()
{
   mock_image = cv::Mat(480,480,CV_8UC3,cv::Scalar(100,100,100));
   cv::putText(mock_image,"NO IMAGE",Point(10,240),FONT_HERSHEY_SIMPLEX,3.0,cv::Scalar(0,255,0),4);
   image_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mock_image).toImageMsg();
}

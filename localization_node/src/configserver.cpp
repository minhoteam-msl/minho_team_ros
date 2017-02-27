#include "configserver.h"

ConfigServer::ConfigServer(ros::NodeHandle *par , QObject *parent) : QObject(parent)
{
   n_subscriptions_ = 0;
   multiple_sends_ = false;
   configured_frequency_ = 1;
   parent_ = par;
   prop_used = 0;calib=false;

   watchdog_timer_ = new QTimer();
   image_timer_ = new QTimer();
   connect(watchdog_timer_,SIGNAL(timeout()),this,SLOT(getSubscribers()));
   connect(image_timer_,SIGNAL(timeout()),this,SLOT(postRequestedImage()));

   init_mock_image();
   // Setup ROS publishers and subscribers
   it_ = new image_transport::ImageTransport(*par);
   image_pub_ = it_->advertise("camera/image", 1);
   error_pub_ = par->advertise<ControlerError>("ControlerError",100);

   mirror_sub_ = par->subscribe("mirrorConfig", 10, &ConfigServer::processMirrorConfig,this);
   vision_sub_ = par->subscribe("visionHSVConfig", 10, &ConfigServer::processVisionConfig,this);
   image_sub_ = par->subscribe("imageConfig", 10, &ConfigServer::processImageConfig,this);
   property_sub_ = par->subscribe("cameraProperty", 10, &ConfigServer::processCamPropertyConfig,this);
   pid_sub_ = par->subscribe("PID", 10, &ConfigServer::processPIDConfig,this);
   roi_sub_ = par->subscribe("ROI", 10, &ConfigServer::processROIConfig,this);
   world_sub_ = par->subscribe("worldConfig", 2, &ConfigServer::processWorldConfig,this);

   service_omniconf = par->advertiseService("requestOmniVisionConf",&ConfigServer::omniVisionConfService,this);
   service_propConf = par->advertiseService("requestCamProperty",&ConfigServer::camPropertyConfService,this);
   service_pidConf = par->advertiseService("requestCamPID",&ConfigServer::camPIDConfService,this);
   service_roiConf = par->advertiseService("requestROI",&ConfigServer::camROIConfService,this);

   service_imgrequest = par->advertiseService("requestImage",&ConfigServer::processImageRequest,this);
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

void ConfigServer::setOmniVisionConf(mirrorConfig mirrmsg,visionHSVConfig vismsg, imageConfig imgmsg, worldConfig ballmsg, worldConfig obsmsg, worldConfig rlemsg)
{
   mirrorConfmsg = mirrmsg;
   visionConfmsg = vismsg;
   imageConfmsg = imgmsg;
   ballConfmsg = ballmsg;
   obsConfmsg = obsmsg;
   RLEConfmsg = rlemsg;
}


void ConfigServer::getSubscribers()
{
   n_subscriptions_ = image_pub_.getNumSubscribers();
   if(n_subscriptions_<=0 && image_timer_->isActive()){
       emit stopImageAssigning();
       image_timer_->stop();
   }
}

bool ConfigServer::processImageRequest(requestImage::Request &req, requestImage::Response &res)
{
   emit changedImageRequest(req.type); // Tells the upper class to assign Images
   configured_frequency_ = req.frequency;
   if(configured_frequency_<=0) configured_frequency_ = 1;
   if(configured_frequency_>30) configured_frequency_ = 30;
   multiple_sends_ = req.is_multiple;
   if(!multiple_sends_) configured_frequency_ = 30; // This is to send the single image as fast as possible

   res.success = true;
   return true;
}

void ConfigServer::processMirrorConfig(const mirrorConfig::ConstPtr &msg)
{
   emit changedMirrorConfiguration(msg);
   mirrorConfmsg = *msg;
}

void ConfigServer::processVisionConfig(const visionHSVConfig::ConstPtr &msg)
{
   emit changedLutConfiguration(msg);
   visionConfmsg = *msg;
}

void ConfigServer::processImageConfig(const imageConfig::ConstPtr &msg)
{
   emit changedImageConfiguration(msg);
   imageConfmsg = *msg;
}

void ConfigServer::processWorldConfig(const worldConfig::ConstPtr &msg)
{
   emit changedWorldConfiguration(msg);
   if(msg->ball_obs==true)ballConfmsg = *msg;
   else if(msg->ball_obs==false && msg->RLE==false)obsConfmsg = *msg;
   else if(msg->kalman==false)RLEConfmsg = *msg;
   else KalmanConfmsg = *msg;
}

bool ConfigServer::omniVisionConfService(minho_team_ros::requestOmniVisionConf::Request &req,minho_team_ros::requestOmniVisionConf::Response &res)
{
   ROS_INFO("requestOmniVisionConf by %s",req.request_node_name.c_str());
   //get data from
   res.mirrorConf = mirrorConfmsg;
   res.visionConf = visionConfmsg;
   res.imageConf = imageConfmsg;
   res.worldConfBall = ballConfmsg;
   res.worldConfObs = obsConfmsg;
   res.worldConfRLE = RLEConfmsg;
   return true;
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
bool ConfigServer::camPropertyConfService(requestCamProperty::Request &req,requestCamProperty::Response &res)
{
   if(req.property.property_id==5 && req.property.blue==true){prop_used=6; res.property=props_msg[6];}
   else if(!req.property.targets){res.property=props_msg[req.property.property_id]; prop_used = res.property.property_id;}
   else if(req.property.targets && req.property.blue){
     res.property.targets=true;
     res.property.val_a = targets.y;
   }
   else {
     res.property.targets=false;
     res.property.val_a = targets.x;
   }

}

bool ConfigServer::camPIDConfService(requestCamPID::Request &req,requestCamPID::Response &res)
{
	if(req.parameter.property_id==5 && req.parameter.blue==true && req.kalman==false){prop_used=6; res.parameterS=pid_msg[6];}
	else if(req.kalman==false){res.parameterS=pid_msg[req.parameter.property_id];prop_used = req.parameter.property_id;}
  else if(req.kalman == true){
    res.Kalman = KalmanConfmsg;
  }

}

void ConfigServer::processCamPropertyConfig(const cameraProperty::ConstPtr &msg)
{
	if(msg->reset!=true){
		if(msg->blue==true){
			props_msg[6]=*msg;
			prop_used = 6;
		}
		else {
			props_msg[msg->property_id]=*msg;
			prop_used = msg->property_id;
		}

	}
	emit changedCameraProperties(msg);
}

void ConfigServer::processPIDConfig(const PID::ConstPtr &msg)
{
	if(msg->calibrate!=true){
		if(msg->blue==true){
			pid_msg[6]=*msg;
			prop_used = 6;
		}
		else {
			pid_msg[msg->property_id]=*msg;
			prop_used = msg->property_id;
			}
		}
	if(msg->calibrate==true)calib=!calib;
	emit changedCamPID(msg);
}


// Save information about properties to vector
void ConfigServer::setProps(vector<float> prop_values)
{
	for(int i=0;i<7;i++){
		props_msg[i].property_id = i;
		props_msg[i].value_a = prop_values[i];
	}
	props_msg[6].blue = true;
	props_msg[6].property_id = 5;

}


void ConfigServer::setPIDValues(vector<float> pid_values)
{
	for(int i=0;i<7;i++){
		pid_msg[i].property_id = i;
		pid_msg[i].p = pid_values[i*3];
		pid_msg[i].i = pid_values[1+i*3];
		pid_msg[i].d = pid_values[2+i*3];
		}
		pid_msg[6].property_id = 5;
		pid_msg[6].blue = true;

    targets.x = int(pid_values[22]);
    targets.y = int(pid_values[23]);
}

void ConfigServer::sendError(Point2d error)
{
	ControlerError msg;
	if(prop_used==6){
		msg.blue==true;
		msg.property_id = 5;
	}
	else msg.property_id = prop_used;
	msg.erro = error.x;
	msg.value = error.y;
	error_pub_.publish(msg);
}

void ConfigServer::processROIConfig(const ROI::ConstPtr &msg)
{
  if(msg->white==true)whiteRoi = *msg;
  else blackRoi = *msg;
	emit changedROIConfiguration(msg);
}

void ConfigServer::setROIs(vector<ROI> rois)
{
	whiteRoi=rois[0];
	blackRoi=rois[1];
}

bool ConfigServer::camROIConfService(requestROI::Request &req,requestROI::Response &res)
{
	res.white=whiteRoi;
	res.black=blackRoi;
	return true;
}

void ConfigServer::setKalman(int Qx,int Qz,int Rx,int Rz)
{
  KalmanConfmsg.value_a = Qx;
  KalmanConfmsg.value_b = Qz;
  KalmanConfmsg.value_c = Rx;
  KalmanConfmsg.window = Rz;
  KalmanConfmsg.kalman = true;

}

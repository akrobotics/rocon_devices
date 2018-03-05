/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#include <rocon_rtsp_camera_relay/rocon_rtsp_camera_relay.hpp>

namespace rocon {

RoconRtspCameraRelay::RoconRtspCameraRelay(ros::NodeHandle& n) : nh_(n)
{
  image_transport::ImageTransport it(nh_);    
  pub_video_ = it.advertise("image", 1);
  pub_camera_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera_info", 1);
  pub_status_ = nh_.advertise<std_msgs::String>("status", 1);
  nh_.param<std::string>("frame_id", camera_frame_, "");
}

RoconRtspCameraRelay::~RoconRtspCameraRelay()
{
  vcap_.release();
}

bool RoconRtspCameraRelay::init(const std::string video_stream_url, const std::string camera_name , const std::string camera_info_url)
{
  video_stream_address_ = video_stream_url;
  camera_name_ = camera_name;
  camera_info_url_ = camera_info_url;
  cinfo_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));
  cinfo_->loadCameraInfo(camera_info_url_);

  if (!vcap_.open(video_stream_address_)) 
    return false; 
  else
    return true;
}

bool RoconRtspCameraRelay::reset(const std::string video_stream_url, const std::string camera_name , const std::string camera_info_url)
{
  vcap_.release();
  return init(video_stream_url, camera_name, camera_info_url);
}

/*
  Convert cv::Mat to sensor_msgs:Image and CameraInfo
 */
void RoconRtspCameraRelay::convertCvToRosImg(const cv::Mat& mat, sensor_msgs::Image& ros_img, sensor_msgs::CameraInfo& ci)
{
  cv_bridge::CvImage cv_img;

  cv_img.encoding = sensor_msgs::image_encodings::BGR8;
  cv_img.image = mat;
  cv_img.toImageMsg(ros_img);
  ros_img.header.stamp = ros::Time::now();
  ros_img.header.frame_id = camera_frame_;
  // check for default camera info
  if (!cinfo_->isCalibrated())
  {
    cinfo_->setCameraName(camera_name_);
    sensor_msgs::CameraInfo camera_info;
    camera_info.header = ros_img.header;
    camera_info.width = ros_img.width;
    camera_info.height = ros_img.height;
    cinfo_->setCameraInfo(camera_info);
  }
  ci = cinfo_->getCameraInfo();
  ci.header = ros_img.header;
  
  return;
}


void RoconRtspCameraRelay::spin()
{
  cv::Mat mat;
  sensor_msgs::CameraInfo ci;
  sensor_msgs::Image ros_img;
  std_msgs::String ros_str;

  while(ros::ok())
  {
    if(!vcap_.read(mat)) {
      status_ = "No frame from camera";
      cv::waitKey();
    }
    else {
      status_ = "live";
    }

    ros_str.data = status_;
    
    convertCvToRosImg(mat, ros_img, ci);
    pub_video_.publish(ros_img);
    pub_camera_info_.publish(ci);
    pub_status_.publish(ros_str);
    cv::waitKey(1);
  }
}
}

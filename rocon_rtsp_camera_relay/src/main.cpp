/*
 License: BSD
   https://raw.github.com/robotics-in-concert/rocon_devices/license/LICENSE
*/

#include <ros/ros.h>
#include <rocon_rtsp_camera_relay/rocon_rtsp_camera_relay.hpp>

int main (int argc, char** argv) 
{
  ros::init(argc, argv, "rtsp_camera_relay");
  ros::NodeHandle pnh("~");
  std::string video_stream_url, user, password;
  std::string camera_name, camera_info_url;

  pnh.getParam("video_stream_url", video_stream_url);
  pnh.getParam("camera_info_url", camera_info_url);
  pnh.getParam("camera_name", camera_name);

  rocon::RoconRtspCameraRelay rtsp(pnh);
  ROS_INFO("Rtsp Camera : Initialising..");
  if(!rtsp.init(video_stream_url, camera_name, camera_info_url))
  {
    ROS_ERROR("Rtsp Camera : Failed to initialise stream");
    return -1;
  }

  ROS_INFO("Rtsp Camera : Initialised");
  rtsp.spin();
  ROS_INFO("Rtsp Camera : Bye Bye");

  return 0;
}

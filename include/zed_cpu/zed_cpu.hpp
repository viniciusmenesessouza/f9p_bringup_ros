#pragma once

#include <memory>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <ros/ros.h>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

namespace zed_cpu
{

class ZedCameraNode
{
public:
  ZedCameraNode(
    const std::shared_ptr<ros::NodeHandle> & nh,
    const std::shared_ptr<image_transport::ImageTransport> & it);
  void runCamera();

private:
  void CameraInit();
  // void SensorInit();
  void PublishImages();
  // void PublishIMU();

  // ---- user-configurable params (set from ROS params) ----
  sl_oc::video::RESOLUTION desired_res_{sl_oc::video::RESOLUTION::HD720};
  sl_oc::video::FPS        desired_fps_{sl_oc::video::FPS::FPS_60};
  bool                     verbose_{false};

  // ---- ROS handles/publishers ----
  std::string node_name_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  ros::Publisher imu_pub_;
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;

  // ---- ZED Open-Capture handles ----
  std::unique_ptr<sl_oc::video::VideoCapture>    cap_;
  std::unique_ptr<sl_oc::sensors::SensorCapture> sens_;
};

}  // namespace zed_cpu


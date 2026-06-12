#include <zed_cpu.hpp>

#include <memory>
#include <vector>
#include <string>  // <-- added
#include <algorithm>  // for std::find

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

namespace zed_cpu
{

// ---- helpers ---------------------------------------------------------------

static sl_oc::video::RESOLUTION parseResolution(const std::string& s) {
  using sl_oc::video::RESOLUTION;
  if (s == "HD2K")   return RESOLUTION::HD2K;
  if (s == "HD1080") return RESOLUTION::HD1080;
  if (s == "HD720")  return RESOLUTION::HD720;
  if (s == "VGA")    return RESOLUTION::VGA;
  ROS_WARN("[ZedCameraNode] Unknown camera_resolution '%s', defaulting to HD720", s.c_str());
  return RESOLUTION::HD720;
}

static sl_oc::video::FPS chooseFps(sl_oc::video::RESOLUTION res, int requested_fps) {
  using sl_oc::video::FPS;
  // Allowed sets per res (typical for ZED/ZED2i Open-Capture):
  // HD2K:   {15}
  // HD1080: {15, 30}
  // HD720:  {15, 30, 60}
  // VGA:    {15, 30, 60, 100}
  std::vector<int> allowed;
  switch (res) {
    case sl_oc::video::RESOLUTION::HD2K:   allowed = {15}; break;
    case sl_oc::video::RESOLUTION::HD1080: allowed = {15, 30}; break;
    case sl_oc::video::RESOLUTION::HD720:  allowed = {15, 30, 60}; break;
    case sl_oc::video::RESOLUTION::VGA:    allowed = {15, 30, 60, 100}; break;
    default:                               allowed = {60}; break;
  }

  // pick the highest allowed <= requested; otherwise lowest allowed
  int pick = allowed.front();
  for (int v : allowed) {
    if (v <= requested_fps) pick = v;
  }
  if (std::find(allowed.begin(), allowed.end(), requested_fps) == allowed.end()) {
    ROS_WARN("[ZedCameraNode] Requested FPS %d not valid for this resolution; using %d",
             requested_fps, pick);
  }

  switch (pick) {
    case 15:  return FPS::FPS_15;
    case 30:  return FPS::FPS_30;
    case 60:  return FPS::FPS_60;
    case 100: return FPS::FPS_100;
    default:  return FPS::FPS_60; // fallback
  }
}

// add helper next to parseResolution/chooseFps
static int fpsEnumToInt(sl_oc::video::FPS fps) {
  using sl_oc::video::FPS;
  switch (fps) {
    case FPS::FPS_15:  return 15;
    case FPS::FPS_30:  return 30;
    case FPS::FPS_60:  return 60;
    case FPS::FPS_100: return 100;
    default:           return -1;
  }
}


ZedCameraNode::ZedCameraNode(
  const std::shared_ptr<ros::NodeHandle> & nh,
  const std::shared_ptr<image_transport::ImageTransport> & it)
: nh_(nh), it_(it)
{
  // ROS initialization
  node_name_ = ros::this_node::getName();
  left_image_pub_ = it_->advertise("rgb/left_image", 1);
  right_image_pub_ = it_->advertise("rgb/right_image", 1);
  imu_pub_ = nh_->advertise<sensor_msgs::Imu>("imu_data", 1);

  // --- read params (private namespace) ---
  std::string res_str = "HD720";
  int fps_req = 60;
  nh_->param<std::string>("camera_resolution", res_str, res_str);
  nh_->param<int>("camera_fps", fps_req, fps_req);
  nh_->param<bool>("camera_verbose", verbose_, false);

  desired_res_ = parseResolution(res_str);
  desired_fps_ = chooseFps(desired_res_, fps_req);

  CameraInit();
  // SensorInit();

  ROS_INFO("[%s] Node started (res=%s, fps=%d, verbose=%s)",
           node_name_.c_str(), res_str.c_str(), fps_req, verbose_ ? "true" : "false");
}

void ZedCameraNode::runCamera()
{
  PublishImages();
}

void ZedCameraNode::CameraInit()
{
  // Initialize ZED camera
  sl_oc::video::VideoParams params;
  params.res = desired_res_;
  params.fps = desired_fps_;
  params.verbose = verbose_ ? sl_oc::VERBOSITY::INFO : sl_oc::VERBOSITY::ERROR;

  // Create Video Capture
  cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
  // replace the logging block at the end of CameraInit()
  if (!cap_->initializeVideo()) {
    ROS_ERROR("[%s] Cannot open camera video capture", node_name_.c_str());
    ros::shutdown();
    return;
  }

  // Try to fetch a frame to discover actual width/height
  const sl_oc::video::Frame f = cap_->getLastFrame();
  const int fps_i = fpsEnumToInt(desired_fps_);

  if (f.data != nullptr) {
    ROS_INFO("[%s] Connected to camera sn: %d [%s] (frame: %dx%d @ %d fps)",
             node_name_.c_str(),
             cap_->getSerialNumber(),
             cap_->getDeviceName().c_str(),
             f.width, f.height, fps_i);
  } else {
    // No frame yet (e.g., right after init) — still report negotiated fps
    ROS_INFO("[%s] Connected to camera sn: %d [%s] (fps=%d)",
             node_name_.c_str(),
             cap_->getSerialNumber(),
             cap_->getDeviceName().c_str(),
             fps_i);
  }
}

// void ZedCameraNode::SensorInit()
// {
//   sens_ = std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::ERROR);

//   std::vector<int> devs = sens_->getDeviceList();

//   if (devs.size() == 0) {
//     ROS_ERROR("[%s] No available ZED 2, ZED 2i or ZED Mini cameras", node_name_.c_str());
//     ros::shutdown();
//     return;
//   }

//   uint16_t fw_maior;
//   uint16_t fw_minor;
//   sens_->getFirmwareVersion(fw_maior, fw_minor);
//   ROS_INFO("[%s] Connected to IMU firmware version: %d.%d", node_name_.c_str(), fw_maior, fw_minor);

//   // Initialize the sensors
//   if (!sens_->initializeSensors(devs[0])) {
//     ROS_ERROR("[%s] IMU initialize failed", node_name_.c_str());
//     ros::shutdown();
//     return;
//   }
// }

void ZedCameraNode::PublishImages()
{
  const sl_oc::video::Frame frame = cap_->getLastFrame();
  if (frame.data == nullptr) return;

  cv::Mat frame_yuv(frame.height, frame.width, CV_8UC2, frame.data);
  cv::Mat frame_bgr;
  cv::cvtColor(frame_yuv, frame_bgr, cv::COLOR_YUV2BGR_YUYV);

  cv::Mat left_img  = frame_bgr(cv::Rect(0, 0, frame_bgr.cols/2, frame_bgr.rows));
  cv::Mat right_img = frame_bgr(cv::Rect(frame_bgr.cols/2, 0, frame_bgr.cols/2, frame_bgr.rows));

  // --- timestamp: prefer camera timestamp if the SDK provides it; otherwise use ROS time ---
  ros::Time stamp;
  // If the SDK exposes a nanosecond/µs timestamp, use it like this:
  // stamp.fromNSec(frame.timestamp_ns);
  // Otherwise:
  stamp = ros::Time::now();
  if (stamp.isZero()) {
    // Fallback when use_sim_time is true but no /clock is being published
    stamp.fromNSec(ros::WallTime::now().toNSec());
  }

  std_msgs::Header lh, rh;
  lh.stamp = stamp; lh.frame_id = "zed_left_optical_frame";
  rh.stamp = stamp; rh.frame_id = "zed_right_optical_frame";

  sensor_msgs::ImagePtr left_msg  = cv_bridge::CvImage(lh, "bgr8", left_img).toImageMsg();
  sensor_msgs::ImagePtr right_msg = cv_bridge::CvImage(rh, "bgr8", right_img).toImageMsg();

  left_image_pub_.publish(left_msg);
  right_image_pub_.publish(right_msg);
}

// void ZedCameraNode::PublishIMU()
// {
//   // Get IMU data with a timeout of 5 milliseconds
//   const sl_oc::sensors::data::Imu imu_data = sens_->getLastIMUData(5000);

//   if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
//     // Create a sensor_msgs/Imu message
//     sensor_msgs::Imu imu_msg;
//     imu_msg.header.stamp = ros::Time::now();
//     imu_msg.header.frame_id = "imu_frame";

//     // Convert the IMU data to the sensor_msgs/Imu message fields
//     imu_msg.linear_acceleration.x = -imu_data.aX;
//     imu_msg.linear_acceleration.y =  imu_data.aY;
//     imu_msg.linear_acceleration.z =  imu_data.aZ;

//     imu_msg.angular_velocity.x = -imu_data.gX;
//     imu_msg.angular_velocity.y =  imu_data.gY;
//     imu_msg.angular_velocity.z =  imu_data.gZ;

//     // Publish the sensor_msgs/Imu message
//     imu_pub_.publish(imu_msg);
//   }
// }

}  // namespace zed_cpu


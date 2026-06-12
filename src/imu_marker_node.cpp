#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <deque>
#include <string>

struct Vec3 { double x,y,z; };
static double norm(const Vec3& v){ return std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z); }

class ImuMarker {
public:
  ImuMarker(ros::NodeHandle& nh, ros::NodeHandle& pnh): nh_(nh), pnh_(pnh) {
    pnh_.param<std::string>("imu_topic", imu_topic_, std::string("imu_data"));
    pnh_.param<std::string>("frame_id", frame_id_, std::string("imu_frame"));
    pnh_.param<std::string>("ns", ns_, std::string("imu_viz"));
    pnh_.param<int>("trail_len", trail_len_, 50);
    pnh_.param<double>("arrow_scale", arrow_scale_, 0.1);   // meters per (m/s^2)
    pnh_.param<double>("shaft_d", shaft_d_, 0.01);
    pnh_.param<double>("head_d", head_d_, 0.02);
    pnh_.param<bool>("use_gyro", use_gyro_, false);         // visualize gyro instead of accel
    pnh_.param<double>("gyro_scale", gyro_scale_, 0.05);    // meters per (rad/s)

    sub_ = nh_.subscribe(imu_topic_, 100, &ImuMarker::imuCb, this);
    pub_arrow_ = nh_.advertise<visualization_msgs::Marker>("imu_viz/arrow", 1);
    pub_trail_ = nh_.advertise<visualization_msgs::Marker>("imu_viz/trail", 1);
  }

private:
  void imuCb(const sensor_msgs::ImuConstPtr& msg) {
    // choose vector
    Vec3 v{};
    if (use_gyro_) {
      v = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    } else {
      v = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    }
    const double scale = use_gyro_ ? gyro_scale_ : arrow_scale_;
    // tip position = scaled vector in imu_frame
    geometry_msgs::Point p0, p1;
    p0.x = p0.y = p0.z = 0.0;
    p1.x = v.x * scale;
    p1.y = v.y * scale;
    p1.z = v.z * scale;

    // Arrow marker
    visualization_msgs::Marker m;
    m.header.stamp = msg->header.stamp;
    m.header.frame_id = frame_id_;
    m.ns = ns_;
    m.id = 0;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.points.clear();
    m.points.push_back(p0);
    m.points.push_back(p1);
    m.scale.x = shaft_d_;  // shaft diameter
    m.scale.y = head_d_;   // head diameter
    m.scale.z = head_d_*1.5; // head length (rviz interprets differently; this is fine)
    // color: accel blue, gyro orange
    if (use_gyro_) { m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0; }
    else           { m.color.r = 0.0; m.color.g = 0.5; m.color.b = 1.0; }
    m.color.a = 0.9;
    m.pose.orientation.w = 1.0;
    pub_arrow_.publish(m);

    // Trail (last N tips)
    if (trail_len_ > 0) {
      if ((int)trail_.size() >= trail_len_) trail_.pop_front();
      trail_.push_back(p1);

      visualization_msgs::Marker t;
      t.header = m.header;
      t.ns = ns_;
      t.id = 1;
      t.type = visualization_msgs::Marker::LINE_STRIP;
      t.action = visualization_msgs::Marker::ADD;
      t.scale.x = shaft_d_*0.5; // line width
      t.color.r = 0.2; t.color.g = 1.0; t.color.b = 0.2; t.color.a = 0.8;
      t.pose.orientation.w = 1.0;
      t.points.assign(trail_.begin(), trail_.end());
      pub_trail_.publish(t);
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher pub_arrow_, pub_trail_;

  std::string imu_topic_{"imu_data"};
  std::string frame_id_{"imu_frame"};
  std::string ns_{"imu_viz"};
  int trail_len_{50};
  double arrow_scale_{0.1};
  double gyro_scale_{0.05};
  double shaft_d_{0.01}, head_d_{0.02};
  bool use_gyro_{false};

  std::deque<geometry_msgs::Point> trail_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_marker_node");
  ros::NodeHandle nh, pnh("~");
  ImuMarker node(nh, pnh);
  ros::spin();
  return 0;
}

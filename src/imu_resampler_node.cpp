#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <deque>
#include <string>
#include <algorithm>

struct ImuSample {
  ros::Time t;
  double ax, ay, az;
  double gx, gy, gz;
};


class ImuResampler {
public:
  ImuResampler(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    // params
    pnh_.param<std::string>("input_topic",  input_topic_,  std::string("imu"));
    pnh_.param<std::string>("output_topic", output_topic_, std::string("imu_data_interpolated"));
    pnh_.param<double>("imu_output_hz",        imu_output_hz_,        300.0);
    pnh_.param<bool>  ("imu_warn_if_below",    imu_warn_if_below_,    true);
    pnh_.param<double>("imu_warn_margin",      imu_warn_margin_,      0.05);
    pnh_.param<double>("imu_freq_ewma_alpha",  imu_freq_ewma_alpha_,  0.1);
    pnh_.param<int>   ("buffer_size",          buffer_size_,          512);
    pnh_.param<int>   ("watchdog_ms",          watchdog_ms_,          200);
    pnh_.param<std::string>("frame_id",        frame_id_,             std::string("imu_frame"));

    // Compute integer-nanosecond period to avoid FP accumulation
    if (imu_output_hz_ <= 0.0) {
      ROS_WARN("[imu_resampler] imu_output_hz <= 0; resampling disabled");
      period_ns_ = 0;
      tick_dt_ = ros::Duration(0.0);
    } else {
      const double period_s = 1.0 / imu_output_hz_;
      period_ns_ = static_cast<uint64_t>(llround(period_s * 1e9));
      tick_dt_ = ros::Duration(0, static_cast<int32_t>(period_ns_ % 1000000000ULL));
      tick_dt_ += ros::Duration(static_cast<int32_t>(period_ns_ / 1000000000ULL));
    }

    sub_ = nh_.subscribe(input_topic_, 2000, &ImuResampler::imuCb, this);
    pub_ = nh_.advertise<sensor_msgs::Imu>(output_topic_, 50);

    if (period_ns_ > 0) {
      // Timer just triggers our loop; timestamps come from our own tick counter.
      timer_ = nh_.createTimer(tick_dt_, &ImuResampler::timerCb, this);
    }
  }

private:
  struct ImuSample {
    ros::Time t;
    double ax, ay, az;
    double gx, gy, gz;
  };

  void imuCb(const sensor_msgs::ImuConstPtr& msg) {
    const ros::Time t = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    // stall watchdog
    if (!last_raw_.isZero()) {
      const double ms_since = (t - last_raw_).toSec() * 1000.0;
      if (ms_since > watchdog_ms_) {
        ROS_WARN_THROTTLE(2.0, "[imu_resampler] stall: no IMU for %.0f ms", ms_since);
      }
    }

    // EWMA input rate
    if (!last_raw_.isZero()) {
      const double dt = (t - last_raw_).toSec();
      if (dt > 1e-6) {
        const double inst = 1.0 / dt;
        est_hz_ = (est_hz_ <= 0.0) ? inst
                                   : imu_freq_ewma_alpha_ * inst + (1.0 - imu_freq_ewma_alpha_) * est_hz_;
      }
    }
    last_raw_ = t;

    // push into buffer (bounded)
    ImuSample s;
    s.t  = t;
    s.ax = msg->linear_acceleration.x;
    s.ay = msg->linear_acceleration.y;
    s.az = msg->linear_acceleration.z;
    s.gx = msg->angular_velocity.x;
    s.gy = msg->angular_velocity.y;
    s.gz = msg->angular_velocity.z;

    buf_.push_back(s);
    if ((int)buf_.size() > buffer_size_) buf_.pop_front();

    // Initialize resampling grid on first sample
    if (!started_ && period_ns_ > 0) {
      start_time_ = t;
      next_tick_  = start_time_;
      started_ = true;
    }
  }

  void timerCb(const ros::TimerEvent& /*ev*/) {
    if (!started_) return;           // wait until we have an anchor sample
    if (buf_.empty()) return;

    // Advance by exactly one period on each callback, irrespective of ROS timer jitter.
    // Use integer-ns increments to preserve exact spacing.
    next_tick_ = addNSec(next_tick_, period_ns_);

    // Rate guard (advisory)
    if (imu_warn_if_below_ && est_hz_ > 0.0) {
      const double desired = imu_output_hz_;
      if (desired > est_hz_ * (1.0 + imu_warn_margin_)) {
        ROS_WARN_THROTTLE(2.0,
          "[imu_resampler] desired %.1f Hz > estimated input %.1f Hz (margin %.0f%%) → interpolation will repeat spectrum",
          desired, est_hz_, imu_warn_margin_ * 100.0);
      }
    }

    // Find bracketing samples a.t <= next_tick_ <= b.t
    ImuSample a, b;
    bool have = false;

    if (buf_.size() == 1) {
      a = b = buf_.back();
      have = true;
    } else {
      // Since buffer is small, linear scan is fine.
      // Optionally, you can keep an index to speed this up.
      for (size_t i = 1; i < buf_.size(); ++i) {
        const ImuSample& A = buf_[i-1];
        const ImuSample& B = buf_[i];
        if ((A.t <= next_tick_) && (next_tick_ <= B.t)) {
          a = A; b = B; have = true; break;
        }
      }
      if (!have) {
        if (next_tick_ < buf_.front().t) {
          // too early: hold earliest (rare right after start if period_ns_ is tiny)
          a = b = buf_.front();
          have = true;
        } else if (next_tick_ > buf_.back().t) {
          // too new: hold latest until new data arrives
          a = b = buf_.back();
          have = true;
        }
      }
    }

    if (!have) return;

    // Build output at exact tick time
    sensor_msgs::Imu out;
    out.header.stamp = next_tick_;    // EXACT evenly spaced stamp
    out.header.frame_id = frame_id_;
    out.orientation_covariance[0] = -1.0; // unknown orientation

    if (b.t > a.t) {
      const double alpha = (next_tick_ - a.t).toSec() / (b.t - a.t).toSec();
      auto lerp = [alpha](double A, double B){ return (1.0 - alpha)*A + alpha*B; };

      out.linear_acceleration.x = lerp(a.ax, b.ax);
      out.linear_acceleration.y = lerp(a.ay, b.ay);
      out.linear_acceleration.z = lerp(a.az, b.az);

      out.angular_velocity.x    = lerp(a.gx, b.gx);
      out.angular_velocity.y    = lerp(a.gy, b.gy);
      out.angular_velocity.z    = lerp(a.gz, b.gz);
    } else {
      // equal/invalid timestamps → pass-through latest
      out.linear_acceleration.x = b.ax; out.linear_acceleration.y = b.ay; out.linear_acceleration.z = b.az;
      out.angular_velocity.x    = b.gx; out.angular_velocity.y    = b.gy; out.angular_velocity.z    = b.gz;
    }

    pub_.publish(out);

    // Keep buffer lean by discarding samples strictly older than 'a'
    // (optional but helps keep search small)
    while (buf_.size() > 2 && buf_.front().t < a.t) {
      buf_.pop_front();
    }
  }

  // Helpers
  static ros::Time addNSec(const ros::Time& t, uint64_t ns) {
    const uint64_t t_ns = static_cast<uint64_t>(t.sec) * 1000000000ULL + static_cast<uint64_t>(t.nsec);
    const uint64_t sum  = t_ns + ns;
    const uint32_t nsec = static_cast<uint32_t>(sum % 1000000000ULL);
    const uint32_t sec  = static_cast<uint32_t>(sum / 1000000000ULL);
    return ros::Time(sec, nsec);
  }

  // ROS
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_;
  ros::Publisher  pub_;
  ros::Timer timer_;

  // Buffer & timing
  std::deque<ImuSample> buf_;
  int buffer_size_{512};
  int watchdog_ms_{200};

  // Params
  std::string input_topic_{"imu"};
  std::string output_topic_{"imu_data_interpolated"};
  std::string frame_id_{"imu_frame"};
  double imu_output_hz_{200.0};
  bool   imu_warn_if_below_{true};
  double imu_warn_margin_{0.05};
  double imu_freq_ewma_alpha_{0.1};

  // Input stats
  double est_hz_{0.0};
  ros::Time last_raw_;

  // Resampling grid
  bool started_{false};
  ros::Time start_time_;
  ros::Time next_tick_;
  uint64_t   period_ns_{0};
  ros::Duration tick_dt_;
};


int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_resampler");
  ros::NodeHandle nh, pnh("~");
  ImuResampler node(nh, pnh);
  ros::spin();
  return 0;
}

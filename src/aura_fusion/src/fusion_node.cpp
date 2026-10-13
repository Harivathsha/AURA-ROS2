#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "aura_fusion/planar_ekf.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace af = aura_fusion;
using SteadyClock = std::chrono::steady_clock;

class FusionNode : public rclcpp::Node
{
public:
  FusionNode() : Node("aura_fusion")
  {
    radius_ = parameter("wheel_radius", 0.0325);
    separation_ = parameter("wheel_separation", 0.21);
    const double hz = parameter("wheel_sample_hz", 50.0);
    if (hz > 1000.0) { throw std::invalid_argument("wheel_sample_hz must be <= 1000"); }
    wheel_period_ns_ = static_cast<std::int64_t>(std::llround(1e9 / hz));
    wheel_R_ = af::wheelCovariance(radius_, separation_,
      parameter("left_wheel_std", 0.1), parameter("right_wheel_std", 0.1));
    const double gyro_std = parameter("gyro_std", 0.02);
    fallback_gyro_variance_ = gyro_std * gyro_std;
    noise_.speed_variance_per_second = parameter("speed_variance_per_second", 0.04, true);
    noise_.turn_variance_per_second = parameter("turn_variance_per_second", 0.25, true);
    noise_.position_variance_per_second = parameter("position_variance_per_second", 1e-6, true);
    filter_ = std::make_unique<af::PlanarEKF>(noise_);
    lag_ns_ = static_cast<std::int64_t>(parameter("reorder_delay", 0.03, true) * 1e9);
    max_gap_ = parameter("max_gap", 0.5);
    if (lag_ns_ >= max_gap_ * 1e9) {
      throw std::invalid_argument("reorder_delay must be less than max_gap");
    }

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(100);
    wheel_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", sensor_qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) { receiveWheel(*msg); });
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/imu", sensor_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) { receiveImu(*msg); });
    clock_sub_ = create_subscription<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS(),
      [this](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
        const auto now = stampNs(msg->clock);
        if (have_clock_ && now < clock_ns_) { restart("simulation clock reset"); }
        clock_ns_ = now; have_clock_ = true;
      });
    wheel_pub_ = create_publisher<nav_msgs::msg::Odometry>("/aura/odom_wheel", 10);
    fused_pub_ = create_publisher<nav_msgs::msg::Odometry>("/aura/odom_fused", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() { drain(); });
    RCLCPP_INFO(get_logger(),
      "Wheel + gyro EKF ready. Wheel updates %.1f Hz; timestamp buffer %.3f s. No TF output.",
      hz, static_cast<double>(lag_ns_) * 1e-9);
  }

private:
  struct Event
  {
    bool wheel{false};
    af::Twist2 twist = af::Twist2::Zero();
    double gyro{0.0};
    double gyro_variance{0.0};
  };

  double parameter(const std::string& name, double value, bool allow_zero = false)
  {
    const double result = declare_parameter<double>(name, value);
    if (!std::isfinite(result) || (allow_zero ? result < 0.0 : result <= 0.0)) {
      throw std::invalid_argument("Invalid parameter: " + name);
    }
    return result;
  }

  static std::int64_t stampNs(const builtin_interfaces::msg::Time& stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
  }

  bool acceptableStamp(std::int64_t stamp)
  {
    if (stamp < 0 || (have_clock_ && stamp > clock_ns_ + 500000000LL)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "Ignoring invalid/far-future sensor stamp; check /clock and use_sim_time.");
      return false;
    }
    return true;
  }

  void receiveWheel(const sensor_msgs::msg::JointState& msg)
  {
    const auto find = [&msg](const std::string& name) {
      return static_cast<std::size_t>(
        std::find(msg.name.begin(), msg.name.end(), name) - msg.name.begin());
    };
    const auto left = find("joint_lw"), right = find("joint_rw");
    if (left == msg.name.size() || right == msg.name.size() ||
        msg.velocity.size() != msg.name.size()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Missing named wheel velocities.");
      return;
    }
    if (!std::isfinite(msg.velocity[left]) || !std::isfinite(msg.velocity[right])) { return; }
    const auto stamp = stampNs(msg.header.stamp);
    if (!acceptableStamp(stamp)) { return; }
    // Gazebo emits joints at about 1 kHz here. Select fresh measurements at 50 Hz.
    // Feeding the same effectively ideal signal 1000 times/s can overstate confidence.
    if (last_selected_wheel_ns_ >= 0 && stamp - last_selected_wheel_ns_ < wheel_period_ns_) {
      return;
    }
    last_selected_wheel_ns_ = stamp;
    Event event; event.wheel = true;
    event.twist = af::wheelTwist(msg.velocity[left], msg.velocity[right], radius_, separation_);
    enqueue(stamp, event);
  }

  void receiveImu(const sensor_msgs::msg::Imu& msg)
  {
    // Verified URDF mounting is an identity rotation: IMU Z equals base Z.
    // Never silently reuse this assumption on another robot or mounting.
    if (msg.header.frame_id != "imu_link" || msg.angular_velocity_covariance[0] < 0.0 ||
        !std::isfinite(msg.angular_velocity.z)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "IMU frame/value unavailable.");
      return;
    }
    const auto stamp = stampNs(msg.header.stamp);
    if (!acceptableStamp(stamp) || stamp <= last_received_gyro_ns_) { return; }
    last_received_gyro_ns_ = stamp;
    Event event;
    event.gyro = msg.angular_velocity.z;
    const double supplied = msg.angular_velocity_covariance[8];
    // An all-zero covariance is unknown. Do not use zero as a perfect measurement.
    event.gyro_variance = std::isfinite(supplied) && supplied > 0.0 ?
      std::max(supplied, 1e-8) : fallback_gyro_variance_;
    enqueue(stamp, event);
  }

  void enqueue(std::int64_t stamp, const Event& event)
  {
    events_[stamp].push_back(event);
    newest_ns_ = std::max(newest_ns_, stamp);
    last_arrival_ = SteadyClock::now(); have_arrival_ = true;
    if (events_.size() > 1000) { restart("sensor queue overflow"); }
  }

  void restart(const char* reason)
  {
    ready_ = false; events_.clear(); newest_ns_ = -1;
    last_selected_wheel_ns_ = last_received_gyro_ns_ = -1;
    have_arrival_ = have_used_gyro_ = false;
    ++segment_;
    RCLCPP_WARN(get_logger(), "%s: new local frame aura_odom_segment_%u", reason, segment_);
  }

  void start(std::int64_t stamp, const Event& wheel)
  {
    filter_->reset(wheel.twist, wheel_R_);
    wheel_pose_.setZero(); wheel_P_.setZero();
    previous_wheel_ = wheel.twist;
    wheel_stamp_ns_ = filter_stamp_ns_ = stamp;
    have_used_gyro_ = false; ready_ = true;
    RCLCPP_INFO(get_logger(), "Starting relative odometry at (0,0,0), segment %u.", segment_);
  }

  void integrateWheel(std::int64_t stamp, const af::Twist2& measured)
  {
    const double dt = static_cast<double>(stamp - wheel_stamp_ns_) * 1e-9;
    if (dt > 0.0) {
      const af::Twist2 average = 0.5 * (previous_wheel_ + measured);
      af::State state; state << wheel_pose_, average;
      const auto F = af::motionJacobian(state, dt);
      const Eigen::Matrix3d F_pose = F.topLeftCorner<3, 3>();
      const Eigen::Matrix<double, 3, 2> J = F.block<3, 2>(0, 3);
      wheel_pose_ = af::motionModel(state, dt).head<3>();
      // Approximate baseline covariance: fixed wheel noise, short intervals.
      // R is conservative for endpoint averaging; inter-interval correlations
      // and systematic wheel-radius/slip errors are not explicitly modelled.
      wheel_P_ = (F_pose * wheel_P_ * F_pose.transpose() + J * wheel_R_ * J.transpose()).eval();
      wheel_P_(0, 0) += noise_.position_variance_per_second * dt;
      wheel_P_(1, 1) += noise_.position_variance_per_second * dt;
      wheel_P_ = (0.5 * (wheel_P_ + wheel_P_.transpose())).eval();
    }
    previous_wheel_ = measured; wheel_stamp_ns_ = stamp;
  }

  void drain()
  {
    if (events_.empty()) { return; }
    // Release the last buffered samples after a pause, without extrapolating
    // on wall time. A running stream normally stays 30 ms behind its stamps.
    const bool quiet = have_arrival_ &&
      std::chrono::duration<double>(SteadyClock::now() - last_arrival_).count() > 0.1;
    const auto limit = quiet ? newest_ns_ : newest_ns_ - lag_ns_;
    while (!events_.empty() && events_.begin()->first <= limit) {
      const auto stamp = events_.begin()->first;
      auto group = std::move(events_.begin()->second);
      events_.erase(events_.begin());
      if (ready_ && stamp < filter_stamp_ns_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
          "Late sensor sample outside timestamp buffer: discarded.");
        continue;
      }
      const bool contains_wheel = std::any_of(group.begin(), group.end(),
        [](const Event& event) { return event.wheel; });
      if (ready_ && (static_cast<double>(stamp - filter_stamp_ns_) * 1e-9 > max_gap_ ||
          (contains_wheel && static_cast<double>(stamp - wheel_stamp_ns_) * 1e-9 > max_gap_))) {
        restart("measurement time gap");
      }
      // Wheel first within a timestamp group; gyro correction then uses the
      // same timestamp. Publish only after every measurement in the group.
      std::stable_sort(group.begin(), group.end(),
        [](const Event& a, const Event& b) { return a.wheel > b.wheel; });
      bool wheel_in_group = false;
      try {
        for (const auto& event : group) {
          if (!ready_) {
            if (!event.wheel) { continue; }
            start(stamp, event); wheel_in_group = true;
            continue; // Initial wheel is used once, not repeated as an update.
          }
          filter_->predict(static_cast<double>(stamp - filter_stamp_ns_) * 1e-9);
          filter_stamp_ns_ = stamp;
          if (event.wheel) {
            integrateWheel(stamp, event.twist);
            filter_->updateWheel(event.twist, wheel_R_);
            wheel_in_group = true;
          } else {
            filter_->updateGyro(event.gyro, event.gyro_variance);
            last_used_gyro_ns_ = stamp; have_used_gyro_ = true;
          }
        }
        if (ready_ && wheel_in_group) { publish(stamp); }
      } catch (const std::exception& error) {
        RCLCPP_ERROR(get_logger(), "Estimator rejected input: %s", error.what());
        restart("numerical/input error");
      }
    }
  }

  nav_msgs::msg::Odometry odometry(
    std::int64_t stamp, const Eigen::Vector3d& pose, const Eigen::Matrix3d& pose_P,
    const af::Twist2& twist, const af::TwistCovariance& twist_P) const
  {
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = rclcpp::Time(stamp, RCL_ROS_TIME);
    msg.header.frame_id = segment_ == 0 ? "aura_odom" :
      "aura_odom_segment_" + std::to_string(segment_);
    msg.child_frame_id = "base_footprint";
    msg.pose.pose.position.x = pose(0); msg.pose.pose.position.y = pose(1);
    msg.pose.pose.orientation.z = std::sin(0.5 * pose(2));
    msg.pose.pose.orientation.w = std::cos(0.5 * pose(2));
    msg.twist.twist.linear.x = twist(0); msg.twist.twist.angular.z = twist(1);
    // Pose is in the local odometry frame; twist is in the robot body frame.
    const std::array<int, 3> planar{0, 1, 5};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        msg.pose.covariance[6 * planar[i] + planar[j]] = pose_P(i, j);
      }
    }
    for (const int index : {2, 3, 4}) { msg.pose.covariance[6 * index + index] = 1e6; }
    msg.twist.covariance[0] = twist_P(0, 0);
    msg.twist.covariance[5] = twist_P(0, 1);
    msg.twist.covariance[30] = twist_P(1, 0);
    msg.twist.covariance[35] = twist_P(1, 1);
    for (const int index : {1, 2, 3, 4}) { msg.twist.covariance[6 * index + index] = 1e6; }
    return msg;
  }

  void publish(std::int64_t stamp)
  {
    const auto& state = filter_->state(); const auto& P = filter_->covariance();
    wheel_pub_->publish(odometry(stamp, wheel_pose_, wheel_P_, previous_wheel_, wheel_R_));
    fused_pub_->publish(odometry(stamp, state.head<3>(), P.topLeftCorner<3, 3>(),
      state.tail<2>(), P.bottomRightCorner<2, 2>()));
    const double gyro_age = have_used_gyro_ ?
      static_cast<double>(stamp - last_used_gyro_ns_) * 1e-9 : std::numeric_limits<double>::quiet_NaN();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "wheel[x=%.3f y=%.3f yaw=%.3f] fused[x=%.3f y=%.3f yaw=%.3f v=%.3f w=%.3f] "
      "gyro_age=%.3f s sigma_yaw=%.4f",
      wheel_pose_(0), wheel_pose_(1), wheel_pose_(2), state(0), state(1), state(2),
      state(3), state(4), gyro_age, std::sqrt(std::max(0.0, P(2, 2))));
    if (!have_used_gyro_ || gyro_age > 0.2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
        "No recent gyro correction: estimate currently relies on wheels and model.");
    }
  }

  double radius_{0.0325}, separation_{0.21}, fallback_gyro_variance_{0.0004}, max_gap_{0.5};
  af::ModelNoise noise_;
  std::unique_ptr<af::PlanarEKF> filter_;
  af::TwistCovariance wheel_R_ = af::TwistCovariance::Identity();
  Eigen::Vector3d wheel_pose_ = Eigen::Vector3d::Zero();
  Eigen::Matrix3d wheel_P_ = Eigen::Matrix3d::Zero();
  af::Twist2 previous_wheel_ = af::Twist2::Zero();
  bool ready_{false}, have_clock_{false}, have_arrival_{false}, have_used_gyro_{false};
  std::int64_t clock_ns_{0}, newest_ns_{-1}, filter_stamp_ns_{0}, wheel_stamp_ns_{0};
  std::int64_t last_selected_wheel_ns_{-1}, last_received_gyro_ns_{-1}, last_used_gyro_ns_{0};
  std::int64_t wheel_period_ns_{20000000}, lag_ns_{30000000};
  unsigned segment_{0};
  SteadyClock::time_point last_arrival_;
  std::map<std::int64_t, std::vector<Event>> events_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr wheel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr wheel_pub_, fused_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<FusionNode>()); }
  catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("aura_fusion"), "%s", error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace motor_control {

namespace {

double yawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double clampAbs(double value, double limit)
{
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}

double normalizeAngle(double angle)
{
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

struct TargetPoint {
  double x {0.0};
  double y {0.0};
  const char* source {"none"};
};

}  // namespace

class VisionGoalTrackerNode : public rclcpp::Node {
public:
  VisionGoalTrackerNode() : Node("vision_goal_tracker_node")
  {
    loadParameters();

    auto_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisionGoalTrackerNode::autoGoalCallback, this, std::placeholders::_1));
    pre_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pre_goal_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisionGoalTrackerNode::preGoalCallback, this, std::placeholders::_1));
    realtime_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      realtime_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisionGoalTrackerNode::realtimeCallback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      std::bind(&VisionGoalTrackerNode::odomCallback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic_, 10);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VisionGoalTrackerNode::publishCommand, this));

    last_goal_time_ = now();
    last_pre_goal_time_ = now();
    last_realtime_time_ = now();
    last_odom_time_ = now();
    last_log_time_ = now();

    RCLCPP_INFO(
      get_logger(),
      "vision_goal_tracker_node 启动: source=%s realtime=%s pre_goal=%s auto_goal=%s odom=%s cmd=%s timeout=%.2fs max_auto_goal=%.2fm",
      target_source_.c_str(), realtime_topic_.c_str(), pre_goal_topic_.c_str(),
      goal_topic_.c_str(), odom_topic_.c_str(), cmd_topic_.c_str(), goal_timeout_,
      max_goal_distance_);
  }

private:
  void loadParameters()
  {
    target_source_ = declare_parameter<std::string>("target_source", target_source_);
    realtime_topic_ = declare_parameter<std::string>("realtime_topic", realtime_topic_);
    pre_goal_topic_ = declare_parameter<std::string>("pre_goal_topic", pre_goal_topic_);
    goal_topic_ = declare_parameter<std::string>("goal_topic", goal_topic_);
    odom_topic_ = declare_parameter<std::string>("odom_topic", odom_topic_);
    cmd_topic_ = declare_parameter<std::string>("cmd_topic", cmd_topic_);
    publish_rate_ = declare_parameter<double>("publish_rate", publish_rate_);
    goal_timeout_ = declare_parameter<double>("goal_timeout", goal_timeout_);
    odom_timeout_ = declare_parameter<double>("odom_timeout", odom_timeout_);
    require_odom_ = declare_parameter<bool>("require_odom", require_odom_);
    max_goal_distance_ = declare_parameter<double>("max_goal_distance", max_goal_distance_);
    max_realtime_distance_ =
      declare_parameter<double>("max_realtime_distance", max_realtime_distance_);
    max_realtime_abs_y_ =
      declare_parameter<double>("max_realtime_abs_y", max_realtime_abs_y_);
    min_realtime_x_ = declare_parameter<double>("min_realtime_x", min_realtime_x_);
    auto_goal_realtime_lockout_ =
      declare_parameter<double>("auto_goal_realtime_lockout", auto_goal_realtime_lockout_);
    auto_goal_hold_time_ =
      declare_parameter<double>("auto_goal_hold_time", auto_goal_hold_time_);
    pre_goal_hold_time_ = declare_parameter<double>("pre_goal_hold_time", pre_goal_hold_time_);
    goal_tolerance_ = declare_parameter<double>("goal_tolerance", goal_tolerance_);
    stop_forward_distance_ =
      declare_parameter<double>("stop_forward_distance", stop_forward_distance_);
    kp_forward_ = declare_parameter<double>("kp_forward", kp_forward_);
    kp_lateral_ = declare_parameter<double>("kp_lateral", kp_lateral_);
    max_forward_velocity_ =
      declare_parameter<double>("max_forward_velocity", max_forward_velocity_);
    max_lateral_velocity_ =
      declare_parameter<double>("max_lateral_velocity", max_lateral_velocity_);
    max_far_lateral_velocity_ =
      declare_parameter<double>("max_far_lateral_velocity", max_far_lateral_velocity_);
    far_lateral_distance_ =
      declare_parameter<double>("far_lateral_distance", far_lateral_distance_);
    max_backward_velocity_ =
      declare_parameter<double>("max_backward_velocity", max_backward_velocity_);
    max_linear_accel_ = declare_parameter<double>("max_linear_accel", max_linear_accel_);
    approach_slow_distance_ =
      declare_parameter<double>("approach_slow_distance", approach_slow_distance_);
    max_near_forward_velocity_ =
      declare_parameter<double>("max_near_forward_velocity", max_near_forward_velocity_);
    reverse_deadband_ = declare_parameter<double>("reverse_deadband", reverse_deadband_);
    forward_sign_ = declare_parameter<double>("forward_sign", forward_sign_);
    lateral_sign_ = declare_parameter<double>("lateral_sign", lateral_sign_);
    use_odom_yaw_ = declare_parameter<bool>("use_odom_yaw", use_odom_yaw_);
    yaw_hold_enabled_ = declare_parameter<bool>("yaw_hold_enabled", yaw_hold_enabled_);
    yaw_hold_kp_ = declare_parameter<double>("yaw_hold_kp", yaw_hold_kp_);
    max_yaw_velocity_ = declare_parameter<double>("max_yaw_velocity", max_yaw_velocity_);
    yaw_deadband_ = declare_parameter<double>("yaw_deadband", yaw_deadband_);
    yaw_sign_ = declare_parameter<double>("yaw_sign", yaw_sign_);
    realtime_lateral_only_when_slow_ = declare_parameter<bool>(
      "realtime_lateral_only_when_slow", realtime_lateral_only_when_slow_);
    realtime_chase_min_speed_ =
      declare_parameter<double>("realtime_chase_min_speed", realtime_chase_min_speed_);
    realtime_speed_alpha_ =
      declare_parameter<double>("realtime_speed_alpha", realtime_speed_alpha_);

    publish_rate_ = std::max(1.0, publish_rate_);
    goal_timeout_ = std::max(0.05, goal_timeout_);
    odom_timeout_ = std::max(0.05, odom_timeout_);
    max_realtime_distance_ = std::max(0.0, max_realtime_distance_);
    max_realtime_abs_y_ = std::max(0.0, max_realtime_abs_y_);
    auto_goal_realtime_lockout_ = std::max(0.0, auto_goal_realtime_lockout_);
    auto_goal_hold_time_ = std::max(goal_timeout_, auto_goal_hold_time_);
    pre_goal_hold_time_ = std::max(goal_timeout_, pre_goal_hold_time_);
    goal_tolerance_ = std::max(0.0, goal_tolerance_);
    stop_forward_distance_ = std::max(0.0, stop_forward_distance_);
    max_forward_velocity_ = std::abs(max_forward_velocity_);
    max_lateral_velocity_ = std::abs(max_lateral_velocity_);
    max_far_lateral_velocity_ = std::abs(max_far_lateral_velocity_);
    far_lateral_distance_ = std::max(0.0, far_lateral_distance_);
    max_backward_velocity_ = std::abs(max_backward_velocity_);
    max_linear_accel_ = std::abs(max_linear_accel_);
    approach_slow_distance_ = std::max(0.0, approach_slow_distance_);
    max_near_forward_velocity_ = std::abs(max_near_forward_velocity_);
    reverse_deadband_ = std::max(0.0, reverse_deadband_);
    yaw_hold_kp_ = std::abs(yaw_hold_kp_);
    max_yaw_velocity_ = std::abs(max_yaw_velocity_);
    yaw_deadband_ = std::max(0.0, yaw_deadband_);
    realtime_chase_min_speed_ = std::max(0.0, realtime_chase_min_speed_);
    realtime_speed_alpha_ = std::clamp(realtime_speed_alpha_, 0.01, 1.0);
    if (target_source_ != "realtime" && target_source_ != "auto_goal" &&
        target_source_ != "auto_or_realtime") {
      RCLCPP_WARN(
        get_logger(), "target_source=%s 无效，回退到 realtime", target_source_.c_str());
      target_source_ = "realtime";
    }
  }

  void autoGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (!isGoalTargetValid(msg->pose.position.x, msg->pose.position.y, "auto_goal")) {
      return;
    }

    latest_goal_ = *msg;
    last_goal_time_ = now();
    has_goal_ = true;
  }

  void preGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (!isGoalTargetValid(msg->pose.position.x, msg->pose.position.y, "pre_goal")) {
      return;
    }

    latest_pre_goal_ = *msg;
    last_pre_goal_time_ = now();
    has_pre_goal_ = true;
  }

  bool isGoalTargetValid(double x, double y, const char* source)
  {
    const double distance_from_origin =
      std::hypot(x, y);
    if (max_goal_distance_ > 0.0 && distance_from_origin > max_goal_distance_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "丢弃超距视觉目标: source=%s distance=%.2f max=%.2f x=%.2f y=%.2f",
        source, distance_from_origin, max_goal_distance_, x, y);
      return false;
    }
    return true;
  }

  void realtimeCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (!isRealtimeTargetValid(msg->point.x, msg->point.y)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "丢弃异常实时球点: x=%.2f y=%.2f max_range=%.2f max_abs_y=%.2f min_x=%.2f",
        msg->point.x, msg->point.y, max_realtime_distance_, max_realtime_abs_y_,
        min_realtime_x_);
      return;
    }

    const auto stamp = now();
    if (has_realtime_) {
      const double dt = (stamp - last_realtime_time_).seconds();
      if (dt > 0.01 && dt < 0.5) {
        const double dx = msg->point.x - latest_realtime_.point.x;
        const double dy = msg->point.y - latest_realtime_.point.y;
        const double speed = std::hypot(dx, dy) / dt;
        realtime_speed_ = has_realtime_speed_
          ? (1.0 - realtime_speed_alpha_) * realtime_speed_ + realtime_speed_alpha_ * speed
          : speed;
        has_realtime_speed_ = true;
      }
    }

    latest_realtime_ = *msg;
    last_realtime_time_ = stamp;
    has_realtime_ = true;
  }

  bool isRealtimeTargetValid(double x, double y) const
  {
    if (!std::isfinite(x) || !std::isfinite(y)) {
      return false;
    }
    if (max_realtime_distance_ > 0.0 && std::hypot(x, y) > max_realtime_distance_) {
      return false;
    }
    if (max_realtime_abs_y_ > 0.0 && std::abs(y) > max_realtime_abs_y_) {
      return false;
    }
    if (x < min_realtime_x_) {
      return false;
    }
    return true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }
    latest_odom_ = *msg;
    latest_yaw_ = yawFromQuaternion(msg->pose.pose.orientation);
    last_odom_time_ = now();
    has_odom_ = true;
  }

  void publishCommand()
  {
    auto cmd = geometry_msgs::msg::Twist();
    const auto now_time = now();

    TargetPoint target;
    const bool fresh_goal = selectTarget(now_time, target);
    const bool fresh_odom =
      has_odom_ && ((now_time - last_odom_time_).seconds() <= odom_timeout_);

    if (!fresh_goal || (require_odom_ && !fresh_odom)) {
      updateYawReferenceWhenIdle(fresh_goal, fresh_odom);
      cmd_pub_->publish(cmd);
      rememberPublishedCommand(now_time, cmd);
      logState(now_time, fresh_goal, fresh_odom, target, cmd, false);
      return;
    }

    const double current_x = fresh_odom ? latest_odom_.pose.pose.position.x : 0.0;
    const double current_y = fresh_odom ? latest_odom_.pose.pose.position.y : 0.0;
    const double yaw = (fresh_odom && use_odom_yaw_) ? latest_yaw_ : 0.0;

    const double error_x_world = target.x - current_x;
    const double error_y_world = target.y - current_y;
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw = std::sin(yaw);
    const double error_forward =
      cos_yaw * error_x_world + sin_yaw * error_y_world - stop_forward_distance_;
    const double error_left =
      -sin_yaw * error_x_world + cos_yaw * error_y_world;

    const bool lateral_only =
      std::string(target.source) == "realtime" &&
      realtime_lateral_only_when_slow_ &&
      (!has_realtime_speed_ || realtime_speed_ < realtime_chase_min_speed_);

    if (std::hypot(error_forward, error_left) > goal_tolerance_) {
      cmd.linear.y = clampAbs(forward_sign_ * kp_forward_ * error_forward, max_forward_velocity_);
      if (cmd.linear.y > 0.0 && approach_slow_distance_ > 1e-6) {
        const double scale = std::clamp(error_forward / approach_slow_distance_, 0.0, 1.0);
        const double forward_limit =
          max_near_forward_velocity_ +
          (max_forward_velocity_ - max_near_forward_velocity_) * scale;
        cmd.linear.y = std::min(cmd.linear.y, forward_limit);
      } else if (error_forward > -reverse_deadband_) {
        cmd.linear.y = std::max(0.0, cmd.linear.y);
      }
      const double lateral_limit = std::abs(error_forward) > far_lateral_distance_
        ? std::min(max_lateral_velocity_, max_far_lateral_velocity_)
        : max_lateral_velocity_;
      cmd.linear.x = clampAbs(lateral_sign_ * kp_lateral_ * error_left, lateral_limit);
      if (cmd.linear.y < -max_backward_velocity_) {
        cmd.linear.y = -max_backward_velocity_;
      }
      if (lateral_only) {
        cmd.linear.y = 0.0;
      }
    }
    cmd.angular.z = computeYawHoldCommand(fresh_odom);

    cmd = limitCommandRate(now_time, cmd);
    cmd_pub_->publish(cmd);
    logState(now_time, fresh_goal, fresh_odom, target, cmd, lateral_only);
  }

  geometry_msgs::msg::Twist limitCommandRate(
    const rclcpp::Time& now_time, const geometry_msgs::msg::Twist& desired)
  {
    if (max_linear_accel_ <= 0.0) {
      rememberPublishedCommand(now_time, desired);
      return desired;
    }

    double dt = has_published_cmd_
      ? (now_time - last_cmd_publish_time_).seconds()
      : (1.0 / publish_rate_);
    if (dt <= 0.0 || dt > 0.5) {
      dt = 1.0 / publish_rate_;
    }

    auto limited = desired;
    const double dx = desired.linear.x - last_published_cmd_.linear.x;
    const double dy = desired.linear.y - last_published_cmd_.linear.y;
    const double delta_norm = std::hypot(dx, dy);
    const double max_delta = max_linear_accel_ * dt;
    if (delta_norm > max_delta && delta_norm > 1e-9) {
      const double scale = max_delta / delta_norm;
      limited.linear.x = last_published_cmd_.linear.x + dx * scale;
      limited.linear.y = last_published_cmd_.linear.y + dy * scale;
    }

    rememberPublishedCommand(now_time, limited);
    return limited;
  }

  void updateYawReferenceWhenIdle(bool fresh_goal, bool fresh_odom)
  {
    if (!yaw_hold_enabled_ || !fresh_odom || fresh_goal) {
      return;
    }
    yaw_reference_ = latest_yaw_;
    last_yaw_error_ = 0.0;
    has_yaw_reference_ = true;
  }

  double computeYawHoldCommand(bool fresh_odom)
  {
    if (!yaw_hold_enabled_ || !fresh_odom) {
      last_yaw_error_ = 0.0;
      return 0.0;
    }
    if (!has_yaw_reference_) {
      yaw_reference_ = latest_yaw_;
      has_yaw_reference_ = true;
    }

    const double yaw_error = normalizeAngle(yaw_reference_ - latest_yaw_);
    last_yaw_error_ = yaw_error;
    if (std::abs(yaw_error) <= yaw_deadband_) {
      return 0.0;
    }
    return clampAbs(yaw_sign_ * yaw_hold_kp_ * yaw_error, max_yaw_velocity_);
  }

  void rememberPublishedCommand(
    const rclcpp::Time& now_time, const geometry_msgs::msg::Twist& cmd)
  {
    last_published_cmd_ = cmd;
    last_cmd_publish_time_ = now_time;
    has_published_cmd_ = true;
  }

  bool selectTarget(const rclcpp::Time& now_time, TargetPoint& target) const
  {
    const double auto_goal_age = has_goal_ ? (now_time - last_goal_time_).seconds() : 0.0;
    const double pre_goal_age = has_pre_goal_ ? (now_time - last_pre_goal_time_).seconds() : 0.0;
    const bool fresh_auto = has_goal_ && (auto_goal_age <= auto_goal_hold_time_);
    const bool fresh_pre_goal = has_pre_goal_ && (pre_goal_age <= pre_goal_hold_time_);
    const bool fresh_realtime =
      has_realtime_ && ((now_time - last_realtime_time_).seconds() <= goal_timeout_);

    if ((target_source_ == "auto_goal" || target_source_ == "auto_or_realtime") && fresh_auto) {
      target.x = latest_goal_.pose.position.x;
      target.y = latest_goal_.pose.position.y;
      target.source = "auto_goal";
      return true;
    }
    if ((target_source_ == "auto_goal" || target_source_ == "auto_or_realtime") &&
        fresh_pre_goal) {
      target.x = latest_pre_goal_.pose.position.x;
      target.y = latest_pre_goal_.pose.position.y;
      target.source = "pre_goal";
      return true;
    }
    const bool auto_goal_lockout =
      target_source_ == "auto_or_realtime" &&
      ((has_goal_ && auto_goal_age <= auto_goal_realtime_lockout_) ||
       (has_pre_goal_ && pre_goal_age <= auto_goal_realtime_lockout_));
    if ((target_source_ == "realtime" || target_source_ == "auto_or_realtime") &&
        fresh_realtime && !auto_goal_lockout) {
      target.x = latest_realtime_.point.x;
      target.y = latest_realtime_.point.y;
      target.source = "realtime";
      return true;
    }
    return false;
  }

  void logState(
    const rclcpp::Time& now_time, bool fresh_goal, bool fresh_odom,
    const TargetPoint& target, const geometry_msgs::msg::Twist& cmd, bool lateral_only)
  {
    if ((now_time - last_log_time_).seconds() < 1.0) {
      return;
    }
    last_log_time_ = now_time;

    if (!fresh_goal) {
      RCLCPP_WARN(get_logger(), "视觉目标超时，输出零速");
      return;
    }
    if (require_odom_ && !fresh_odom) {
      RCLCPP_WARN(get_logger(), "里程计超时，输出零速");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "视觉追点: source=%s target=(%.2f, %.2f) odom=(%.2f, %.2f) speed=%.2f mode=%s yaw_err=%.3f cmd=(right %.2f, forward %.2f, yaw %.2f)",
      target.source, target.x, target.y,
      has_odom_ ? latest_odom_.pose.pose.position.x : 0.0,
      has_odom_ ? latest_odom_.pose.pose.position.y : 0.0,
      has_realtime_speed_ ? realtime_speed_ : 0.0,
      lateral_only ? "lateral_only" : "chase",
      last_yaw_error_, cmd.linear.x, cmd.linear.y, cmd.angular.z);
  }

  std::string target_source_ {"realtime"};
  std::string realtime_topic_ {"/frontvehicle/ball/realtime"};
  std::string pre_goal_topic_ {"/frontvehicle/auto/pre_goal_pose"};
  std::string goal_topic_ {"/frontvehicle/auto/goal_pose"};
  std::string odom_topic_ {"/odom"};
  std::string cmd_topic_ {"/vision/cmd_vel"};

  double publish_rate_ {50.0};
  double goal_timeout_ {0.3};
  double odom_timeout_ {0.3};
  bool require_odom_ {true};
  double max_goal_distance_ {7.0};
  double max_realtime_distance_ {10.0};
  double max_realtime_abs_y_ {5.0};
  double min_realtime_x_ {0.0};
  double auto_goal_realtime_lockout_ {2.0};
  double auto_goal_hold_time_ {3.0};
  double pre_goal_hold_time_ {2.0};
  double goal_tolerance_ {0.10};
  double stop_forward_distance_ {0.0};
  double kp_forward_ {0.45};
  double kp_lateral_ {0.45};
  double max_forward_velocity_ {0.8};
  double max_lateral_velocity_ {0.6};
  double max_far_lateral_velocity_ {0.8};
  double far_lateral_distance_ {1.5};
  double max_backward_velocity_ {0.4};
  double max_linear_accel_ {2.0};
  double approach_slow_distance_ {2.0};
  double max_near_forward_velocity_ {0.6};
  double reverse_deadband_ {0.3};
  double forward_sign_ {1.0};
  double lateral_sign_ {-1.0};
  bool use_odom_yaw_ {true};
  bool yaw_hold_enabled_ {false};
  double yaw_hold_kp_ {1.2};
  double max_yaw_velocity_ {0.6};
  double yaw_deadband_ {0.04};
  double yaw_sign_ {1.0};
  bool realtime_lateral_only_when_slow_ {true};
  double realtime_chase_min_speed_ {0.4};
  double realtime_speed_alpha_ {0.25};

  geometry_msgs::msg::PoseStamped latest_goal_;
  geometry_msgs::msg::PoseStamped latest_pre_goal_;
  geometry_msgs::msg::PointStamped latest_realtime_;
  nav_msgs::msg::Odometry latest_odom_;
  double latest_yaw_ {0.0};
  double yaw_reference_ {0.0};
  double last_yaw_error_ {0.0};
  double realtime_speed_ {0.0};
  bool has_goal_ {false};
  bool has_pre_goal_ {false};
  bool has_realtime_ {false};
  bool has_odom_ {false};
  bool has_yaw_reference_ {false};
  bool has_realtime_speed_ {false};
  bool has_published_cmd_ {false};

  rclcpp::Time last_goal_time_;
  rclcpp::Time last_pre_goal_time_;
  rclcpp::Time last_realtime_time_;
  rclcpp::Time last_odom_time_;
  rclcpp::Time last_log_time_;
  rclcpp::Time last_cmd_publish_time_;
  geometry_msgs::msg::Twist last_published_cmd_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr auto_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pre_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr realtime_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace motor_control

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<motor_control::VisionGoalTrackerNode>());
  rclcpp::shutdown();
  return 0;
}

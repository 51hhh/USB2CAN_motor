#include <rclcpp/rclcpp.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#define COLOR_RESET   "\033[0m"
#define COLOR_RED     "\033[31m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_BOLD    "\033[1m"
#define COLOR_DIM     "\033[2m"

#define CLEAR_SCREEN  "\033[2J"
#define CURSOR_HOME   "\033[H"
#define CURSOR_HIDE   "\033[?25l"
#define CURSOR_SHOW   "\033[?25h"

namespace {

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

double yawFromOdom(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

std::string valueOr(const std::map<std::string, std::string> & values,
                    const std::string & key,
                    const std::string & fallback = "-")
{
  const auto it = values.find(key);
  return it == values.end() ? fallback : it->second;
}

double doubleValueOr(const std::map<std::string, std::string> & values,
                     const std::string & key,
                     double fallback = 0.0)
{
  const auto it = values.find(key);
  if (it == values.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (...) {
    return fallback;
  }
}

std::string formatDouble(double value, int precision)
{
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(precision) << value;
  return oss.str();
}

}  // namespace

class PositioningMonitorNode : public rclcpp::Node {
public:
  PositioningMonitorNode() : Node("positioning_monitor_node")
  {
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/odom");
    wheel_odom_topic_ = this->declare_parameter<std::string>("wheel_odom_topic", "/odom_wheels");
    diagnostics_topic_ = this->declare_parameter<std::string>("diagnostics_topic", "/diagnostics");
    display_rate_hz_ = this->declare_parameter<double>("display_rate_hz", 5.0);
    stale_timeout_sec_ = this->declare_parameter<double>("stale_timeout_sec", 0.5);
    jump_warn_m_ = this->declare_parameter<double>("jump_warn_m", 0.05);
    yaw_jump_warn_rad_ = this->declare_parameter<double>("yaw_jump_warn_rad", 0.10);

    if (display_rate_hz_ <= 0.0) {
      display_rate_hz_ = 5.0;
    }
    if (stale_timeout_sec_ <= 0.0) {
      stale_timeout_sec_ = 0.5;
    }

    std::cout << CLEAR_SCREEN << CURSOR_HIDE << std::flush;

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&PositioningMonitorNode::odomCallback, this, std::placeholders::_1));

    wheel_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      wheel_odom_topic_, 20,
      std::bind(&PositioningMonitorNode::wheelOdomCallback, this, std::placeholders::_1));

    diagnostics_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      diagnostics_topic_, 10,
      std::bind(&PositioningMonitorNode::diagnosticsCallback, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / display_rate_hz_);
    display_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PositioningMonitorNode::updateDisplay, this));
  }

  ~PositioningMonitorNode() override
  {
    std::cout << CURSOR_SHOW << COLOR_RESET << std::endl;
  }

private:
  struct TopicStats {
    rclcpp::Time last_update;
    rclcpp::Time last_stat_time;
    int msg_count {0};
    double hz {0.0};
  };

  void updateStats(TopicStats & stats)
  {
    const auto now = this->now();
    if (stats.last_update.nanoseconds() == 0) {
      stats.last_stat_time = now;
    }
    stats.last_update = now;
    stats.msg_count++;

    const double dt = (now - stats.last_stat_time).seconds();
    if (dt >= 1.0) {
      stats.hz = static_cast<double>(stats.msg_count) / dt;
      stats.msg_count = 0;
      stats.last_stat_time = now;
    }
  }

  bool isFresh(const TopicStats & stats) const
  {
    if (stats.last_update.nanoseconds() == 0) {
      return false;
    }
    return (this->now() - stats.last_update).seconds() <= stale_timeout_sec_;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double yaw = yawFromOdom(*msg);
    if (has_odom_) {
      const auto & prev_p = latest_odom_.pose.pose.position;
      const auto & curr_p = msg->pose.pose.position;
      last_step_xy_m_ = std::hypot(curr_p.x - prev_p.x, curr_p.y - prev_p.y);
      last_step_yaw_rad_ = std::abs(normalizeAngle(yaw - latest_odom_yaw_));
      max_step_xy_m_ = std::max(max_step_xy_m_, last_step_xy_m_);
      max_step_yaw_rad_ = std::max(max_step_yaw_rad_, last_step_yaw_rad_);
      if (last_step_xy_m_ >= jump_warn_m_ || last_step_yaw_rad_ >= yaw_jump_warn_rad_) {
        jump_count_++;
        last_jump_time_ = this->now();
      }
    }

    latest_odom_ = *msg;
    latest_odom_yaw_ = yaw;
    has_odom_ = true;
    updateStats(odom_stats_);
  }

  void wheelOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_wheel_odom_ = *msg;
    has_wheel_odom_ = true;
    updateStats(wheel_stats_);
  }

  void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    diagnostics_values_.clear();
    diagnostics_level_ = 0;
    diagnostics_message_ = "-";

    for (const auto & status : msg->status) {
      if (status.name != "positioning_bridge") {
        continue;
      }
      diagnostics_level_ = static_cast<int>(status.level);
      diagnostics_message_ = status.message;
      for (const auto & kv : status.values) {
        diagnostics_values_[kv.key] = kv.value;
      }
      break;
    }
    has_diagnostics_ = true;
    updateStats(diagnostics_stats_);
  }

  std::string freshText(const TopicStats & stats) const
  {
    if (isFresh(stats)) {
      return "ONLINE";
    }
    if (stats.last_update.nanoseconds() == 0) {
      return "NO_DATA";
    }
    return "STALE";
  }

  std::string ageText(const TopicStats & stats) const
  {
    if (stats.last_update.nanoseconds() == 0) {
      return "-";
    }
    return formatDouble((this->now() - stats.last_update).seconds(), 2);
  }

  std::string diagLevelText() const
  {
    if (!has_diagnostics_) {
      return "NO_DIAG";
    }
    if (diagnostics_level_ == 0) {
      return "OK";
    }
    if (diagnostics_level_ == 1) {
      return "WARN";
    }
    return "ERROR";
  }

  std::string boolColorText(const std::string & value) const
  {
    return value;
  }

  std::string stepColor(double value, double threshold) const
  {
    (void)threshold;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(6) << value;
    return oss.str();
  }

  void updateDisplay()
  {
    std::ostringstream oss;
    oss << CLEAR_SCREEN << CURSOR_HOME;

    oss << COLOR_BOLD << COLOR_CYAN
      << "╔════════════════════════════════════════════════════════════════════════════════════════╗\n"
      << "║                             码盘定位 / ODOM 实时监控                                  ║\n"
      << "╚════════════════════════════════════════════════════════════════════════════════════════╝"
        << COLOR_RESET << "\n\n";

    auto topicRow = [&oss](const std::string & topic, const std::string & state,
                 double hz, const std::string & age) {
      oss << "│ " << std::left << std::setw(16) << topic
        << " │ " << std::setw(9) << state
        << " │ " << std::right << std::setw(8) << formatDouble(hz, 1)
        << " │ " << std::setw(8) << age << " │\n";
    };

    oss << COLOR_BOLD << "Topic 状态" << COLOR_RESET << "\n";
    oss << "┌──────────────────┬───────────┬──────────┬──────────┐\n";
    oss << "│ Topic            │ State     │ Hz       │ Age(s)   │\n";
    oss << "├──────────────────┼───────────┼──────────┼──────────┤\n";
    topicRow(odom_topic_, freshText(odom_stats_), odom_stats_.hz, ageText(odom_stats_));
    topicRow(wheel_odom_topic_, freshText(wheel_stats_), wheel_stats_.hz, ageText(wheel_stats_));
    topicRow(diagnostics_topic_, freshText(diagnostics_stats_), diagnostics_stats_.hz, ageText(diagnostics_stats_));
    oss << "└──────────────────┴───────────┴──────────┴──────────┘\n\n";

    auto valueRow = [&oss](const std::string & name, const std::string & value,
                 const std::string & unit, const std::string & note = "") {
      oss << "│ " << std::left << std::setw(14) << name
        << " │ " << std::right << std::setw(16) << value
        << " │ " << std::left << std::setw(8) << unit
        << " │ " << std::setw(26) << note << " │\n";
    };

    oss << COLOR_BOLD << "/odom 位姿与跳变" << COLOR_RESET << "\n";
    oss << "┌────────────────┬──────────────────┬──────────┬────────────────────────────┐\n";
    oss << "│ Field          │ Value            │ Unit     │ Note                       │\n";
    oss << "├────────────────┼──────────────────┼──────────┼────────────────────────────┤\n";
    if (has_odom_) {
      const auto & p = latest_odom_.pose.pose.position;
      const auto & t = latest_odom_.twist.twist;
      valueRow("x", formatDouble(p.x, 6), "m");
      valueRow("y", formatDouble(p.y, 6), "m");
      valueRow("yaw", formatDouble(latest_odom_yaw_, 6), "rad", formatDouble(latest_odom_yaw_ * 180.0 / M_PI, 2) + " deg");
      valueRow("vx", formatDouble(t.linear.x, 6), "m/s");
      valueRow("vy", formatDouble(t.linear.y, 6), "m/s");
      valueRow("wz", formatDouble(t.angular.z, 6), "rad/s");
      valueRow("step_xy", stepColor(last_step_xy_m_, jump_warn_m_), "m", last_step_xy_m_ >= jump_warn_m_ ? "JUMP" : "ok");
      valueRow("max_step_xy", stepColor(max_step_xy_m_, jump_warn_m_), "m");
      valueRow("step_yaw", stepColor(last_step_yaw_rad_, yaw_jump_warn_rad_), "rad", last_step_yaw_rad_ >= yaw_jump_warn_rad_ ? "JUMP" : "ok");
      valueRow("max_step_yaw", stepColor(max_step_yaw_rad_, yaw_jump_warn_rad_), "rad");
      valueRow("jumps", std::to_string(jump_count_), "count", last_jump_time_.nanoseconds() == 0 ? "-" : "last " + formatDouble((this->now() - last_jump_time_).seconds(), 2) + "s ago");
    } else {
      valueRow("status", "waiting", "-", "no /odom data");
    }
    oss << "└────────────────┴──────────────────┴──────────┴────────────────────────────┘\n\n";

    oss << COLOR_BOLD << "/odom_wheels 轮速里程计" << COLOR_RESET << "\n";
    oss << "┌────────────────┬──────────────────┬──────────┬────────────────────────────┐\n";
    oss << "│ Field          │ Value            │ Unit     │ Note                       │\n";
    oss << "├────────────────┼──────────────────┼──────────┼────────────────────────────┤\n";
    if (has_wheel_odom_) {
      const auto & t = latest_wheel_odom_.twist.twist;
      const auto & p = latest_wheel_odom_.pose.pose.position;
      valueRow("wheel_x", formatDouble(p.x, 6), "m");
      valueRow("wheel_y", formatDouble(p.y, 6), "m");
      valueRow("wheel_vx", formatDouble(t.linear.x, 6), "m/s");
      valueRow("wheel_vy", formatDouble(t.linear.y, 6), "m/s");
      valueRow("wheel_wz", formatDouble(t.angular.z, 6), "rad/s");
    } else {
      valueRow("status", "waiting", "-", "pure encoder fusion");
    }
    oss << "└────────────────┴──────────────────┴──────────┴────────────────────────────┘\n\n";

    auto diagRow = [&oss](const std::string & name, const std::string & value,
                const std::string & name2, const std::string & value2) {
      oss << "│ " << std::left << std::setw(20) << name
        << " │ " << std::setw(18) << value
        << " │ " << std::setw(20) << name2
        << " │ " << std::setw(18) << value2 << " │\n";
    };

    oss << COLOR_BOLD << "positioning_bridge 诊断" << COLOR_RESET << "\n";
    oss << "┌──────────────────────┬────────────────────┬──────────────────────┬────────────────────┐\n";
    oss << "│ Key                  │ Value              │ Key                  │ Value              │\n";
    oss << "├──────────────────────┼────────────────────┼──────────────────────┼────────────────────┤\n";
    diagRow("level", diagLevelText(), "message", diagnostics_message_);
    diagRow("serial_open", boolColorText(valueOr(diagnostics_values_, "serial_open")), "protocol", valueOr(diagnostics_values_, "protocol_mode"));
    diagRow("frames_ok", valueOr(diagnostics_values_, "frames_ok"), "crc_error", valueOr(diagnostics_values_, "frames_crc_error"));
    diagRow("parse_error", valueOr(diagnostics_values_, "frames_parse_error"), "last_error", valueOr(diagnostics_values_, "last_serial_error", ""));
    diagRow("time_sync_locked", boolColorText(valueOr(diagnostics_values_, "time_sync_locked")), "samples", valueOr(diagnostics_values_, "time_sync_samples"));
    diagRow("rtt_us", valueOr(diagnostics_values_, "time_sync_rtt_us"), "fusion", valueOr(diagnostics_values_, "fusion_state"));
    diagRow("alpha", valueOr(diagnostics_values_, "fusion_alpha"), "motor_twist_age", valueOr(diagnostics_values_, "motor_twist_age_sec"));
    diagRow("residual_xy", valueOr(diagnostics_values_, "fusion_residual_xy_m"), "residual_yaw", valueOr(diagnostics_values_, "fusion_residual_yaw_rad"));
    diagRow("state_changes", valueOr(diagnostics_values_, "fusion_state_changes"), "", "");
    oss << "└──────────────────────┴────────────────────┴──────────────────────┴────────────────────┘\n\n";

    const double time_sync_samples = doubleValueOr(diagnostics_values_, "time_sync_samples", 0.0);
    const std::string serial_open = valueOr(diagnostics_values_, "serial_open");
    if (serial_open == "true" && has_diagnostics_ && time_sync_samples <= 0.0) {
      oss << COLOR_YELLOW
          << "提示: frames_ok 增加但 time_sync_samples=0 时，说明 MCU->Host 通，Host->MCU 上行可能不通。\n"
          << COLOR_RESET;
    }
    oss << COLOR_DIM
        << "提示: 按 Ctrl+C 退出  |  jump_warn_m=" << std::fixed << std::setprecision(3)
        << jump_warn_m_ << " m  yaw_warn=" << yaw_jump_warn_rad_ << " rad"
        << COLOR_RESET << "\n";

    std::cout << oss.str() << std::flush;
  }

  std::string odom_topic_;
  std::string wheel_odom_topic_;
  std::string diagnostics_topic_;
  double display_rate_hz_ {5.0};
  double stale_timeout_sec_ {0.5};
  double jump_warn_m_ {0.05};
  double yaw_jump_warn_rad_ {0.10};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr wheel_odom_sub_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
  rclcpp::TimerBase::SharedPtr display_timer_;

  TopicStats odom_stats_;
  TopicStats wheel_stats_;
  TopicStats diagnostics_stats_;

  bool has_odom_ {false};
  bool has_wheel_odom_ {false};
  bool has_diagnostics_ {false};
  nav_msgs::msg::Odometry latest_odom_;
  nav_msgs::msg::Odometry latest_wheel_odom_;
  double latest_odom_yaw_ {0.0};

  double last_step_xy_m_ {0.0};
  double last_step_yaw_rad_ {0.0};
  double max_step_xy_m_ {0.0};
  double max_step_yaw_rad_ {0.0};
  int jump_count_ {0};
  rclcpp::Time last_jump_time_;

  int diagnostics_level_ {0};
  std::string diagnostics_message_ {"-"};
  std::map<std::string, std::string> diagnostics_values_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositioningMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

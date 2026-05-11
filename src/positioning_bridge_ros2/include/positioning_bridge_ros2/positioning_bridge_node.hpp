#ifndef POSITIONING_BRIDGE_ROS2__POSITIONING_BRIDGE_NODE_HPP_
#define POSITIONING_BRIDGE_ROS2__POSITIONING_BRIDGE_NODE_HPP_

#include <optional>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "positioning_bridge_ros2/protocol_decoder.hpp"
#include "positioning_bridge_ros2/serial_transport.hpp"
#include "positioning_bridge_ros2/time_sync_estimator.hpp"

namespace positioning_bridge_ros2 {

class PositioningBridgeNode : public rclcpp::Node {
public:
  PositioningBridgeNode();

private:
  struct PoseHistory {
    double x {0.0};
    double y {0.0};
    double yaw {0.0};
    rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
    bool valid {false};
  };

  enum class FusionMode {
    NONE,
    COMPLEMENTARY,
  };

  enum class FusionState {
    NORMAL,
    ALERT,
    DISTURBED,
  };

  struct MotorTwistSample {
    double vx {0.0};
    double vy {0.0};
    double wz {0.0};
    rclcpp::Time stamp {0, 0, RCL_ROS_TIME};
    bool valid {false};
  };

  void declareParameters();
  std::string resolveConfigFile();
  void loadParameters();
  void tryOpenSerial();
  void readSerialTick();
  void timeSyncTick();
  void diagnosticsTick();
  void handleResetLocalOrigin(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);
  bool sendSetLocalOrigin(float x, float y, float yaw, std::string & err);
  void maybeAutoResetOrigin();

  void handleFrame(const DecodedFrame & frame);
  void publishPose(const PoseSample & pose_sample, uint16_t sequence);
  void publishDiagnostics();
  void publishTransform(const nav_msgs::msg::Odometry & odom_msg);
  nav_msgs::msg::Odometry buildOdometry(const PoseSample & pose_sample, const rclcpp::Time & stamp) const;
  rclcpp::Time resolveStamp(const PoseSample & pose_sample) const;
  void maybeUpdateDerivedVelocity(PoseSample & pose_sample, const rclcpp::Time & stamp);
  static double unwrapAngle(double previous, double current);

  void onMotorTwist(const nav_msgs::msg::Odometry::SharedPtr msg);
  // 在 publishPose 内部对 pose_sample 做就地修改 (xy/yaw 替换为融合结果)
  // 返回 true 表示融合发生 (任意模式), false 表示完全跳过
  bool fusePose(PoseSample & pose_sample, const rclcpp::Time & stamp);
  void resetFusion();

  SerialConfig serial_config_;
  ProtocolMode protocol_mode_ {ProtocolMode::LEGACY_ASCII};
  SerialTransport serial_;
  std::unique_ptr<ProtocolDecoder> decoder_;
  TimeSyncEstimator time_sync_estimator_;

  std::string frame_id_ {"odom"};
  std::string child_frame_id_ {"base_link"};
  bool publish_tf_ {true};
  bool derive_velocity_from_pose_ {true};
  double reopen_interval_sec_ {1.0};
  double diagnostic_period_sec_ {1.0};
  double time_sync_period_sec_ {0.5};
  double pose_timeout_sec_ {0.5};
  bool auto_reset_origin_on_start_ {true};
  double auto_reset_origin_timeout_sec_ {3.0};

  // ── 融合参数 ──
  FusionMode fusion_mode_ {FusionMode::NONE};
  std::string motor_twist_topic_ {"/odom_wheels"};
  double motor_twist_timeout_sec_ {0.3};
  double fusion_threshold_pos_normal_ {0.05};
  double fusion_threshold_pos_alert_  {0.20};
  double fusion_threshold_yaw_normal_ {0.05};
  double fusion_threshold_yaw_alert_  {0.30};
  double fusion_alpha_normal_    {0.30};
  double fusion_alpha_alert_     {0.15};
  double fusion_alpha_disturbed_ {0.02};

  // ── 融合运行时状态 ──
  MotorTwistSample latest_motor_twist_;
  bool fusion_initialized_ {false};
  double fused_x_   {0.0};
  double fused_y_   {0.0};
  double fused_yaw_ {0.0};
  rclcpp::Time fused_stamp_ {0, 0, RCL_ROS_TIME};
  FusionState fusion_state_ {FusionState::NORMAL};
  uint64_t fusion_state_change_count_ {0};
  double last_residual_xy_ {0.0};
  double last_residual_yaw_ {0.0};
  double last_alpha_ {0.0};

  uint16_t next_tx_sequence_ {1};
  rclcpp::Time last_pose_stamp_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time node_start_time_ {0, 0, RCL_ROS_TIME};
  bool origin_reset_done_ {false};
  PoseHistory previous_pose_;
  std::optional<StatusSample> latest_status_;
  std::string last_serial_error_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_local_origin_srv_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr motor_twist_sub_;

  rclcpp::TimerBase::SharedPtr read_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr time_sync_timer_;
};

}  // namespace positioning_bridge_ros2

#endif  // POSITIONING_BRIDGE_ROS2__POSITIONING_BRIDGE_NODE_HPP_

#include "positioning_bridge_ros2/positioning_bridge_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace positioning_bridge_ros2 {

PositioningBridgeNode::PositioningBridgeNode()
: Node("positioning_bridge_node")
{
  declareParameters();
  loadParameters();

  node_start_time_ = this->now();
  decoder_ = std::make_unique<ProtocolDecoder>(protocol_mode_);//结构化帧
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 50);
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);
  ir_trigger_pub_ = this->create_publisher<std_msgs::msg::Bool>("/ir_trigger", 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  reset_local_origin_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "/positioning/reset_local_origin",
    std::bind(&PositioningBridgeNode::handleResetLocalOrigin, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "输出纯码盘 pose 到 /odom");

  read_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(10), std::bind(&PositioningBridgeNode::readSerialTick, this));
  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(diagnostic_period_sec_)),
    std::bind(&PositioningBridgeNode::diagnosticsTick, this));

  if (protocol_mode_ == ProtocolMode::BINARY_V1) {
    time_sync_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(time_sync_period_sec_)),
      std::bind(&PositioningBridgeNode::timeSyncTick, this));
  }

  RCLCPP_INFO(this->get_logger(), "positioning_bridge_node 启动，协议模式: %s",
    protocol_mode_ == ProtocolMode::LEGACY_ASCII ? "legacy_ascii" : "binary_v1");
}

void PositioningBridgeNode::declareParameters()
{
  // 唯一 ROS2 参数：外部 yaml 文件路径（参考 omni_chassis_control_node 模式）
  // 为空时默认到 share 目录下的 positioning_bridge_params.yaml
  this->declare_parameter<std::string>("config_file", "");
}

std::string PositioningBridgeNode::resolveConfigFile()
{
  std::string config_file = this->get_parameter("config_file").as_string();
  if (!config_file.empty()) {
    return config_file;
  }
  return ament_index_cpp::get_package_share_directory("positioning_bridge_ros2")
    + "/config/positioning_bridge_params.yaml";
}

void PositioningBridgeNode::loadParameters()
{
  const std::string config_file = resolveConfigFile();
  RCLCPP_INFO(this->get_logger(), "正在加载 positioning_bridge 参数: %s", config_file.c_str());

  YAML::Node root = YAML::LoadFile(config_file);
  YAML::Node p = root;
  if (root["positioning_bridge_node"] && root["positioning_bridge_node"]["ros__parameters"]) {
    p = root["positioning_bridge_node"]["ros__parameters"];
  }

  auto str = [&](const char* k, const std::string& def) {
    return p[k] ? p[k].as<std::string>() : def;
  };
  auto dbl = [&](const char* k, double def) {
    return p[k] ? p[k].as<double>() : def;
  };
  auto bl = [&](const char* k, bool def) {
    return p[k] ? p[k].as<bool>() : def;
  };

  // serial
  if (p["serial"]) {
    auto s = p["serial"];
    serial_config_.device = s["device"] ? s["device"].as<std::string>() : "/dev/ttyUSB0";
    serial_config_.baudrate = s["baudrate"] ? s["baudrate"].as<int>() : 115200;
    serial_config_.data_bits = s["data_bits"] ? s["data_bits"].as<int>() : 8;
    serial_config_.stop_bits = s["stop_bits"] ? s["stop_bits"].as<int>() : 1;
    serial_config_.parity = s["parity"] ? s["parity"].as<std::string>() : "none";
    serial_config_.read_timeout_ms = s["read_timeout_ms"] ? s["read_timeout_ms"].as<int>() : 20;
  } else {
    serial_config_.device = "/dev/ttyUSB0";
    serial_config_.baudrate = 115200;
    serial_config_.data_bits = 8;
    serial_config_.stop_bits = 1;
    serial_config_.parity = "none";
    serial_config_.read_timeout_ms = 20;
  }

  const auto protocol_mode_string = str("protocol_mode", "binary_v1");
  protocol_mode_ = (protocol_mode_string == "binary_v1") ? ProtocolMode::BINARY_V1 : ProtocolMode::LEGACY_ASCII;
  frame_id_ = str("frame_id", "odom");
  child_frame_id_ = str("child_frame_id", "base_link");
  publish_tf_ = bl("publish_tf", true);
  derive_velocity_from_pose_ = bl("derive_velocity_from_pose", true);
  reopen_interval_sec_ = dbl("reopen_interval_sec", 1.0);
  diagnostic_period_sec_ = dbl("diagnostic_period_sec", 1.0);
  time_sync_period_sec_ = dbl("time_sync_period_sec", 0.5);
  pose_timeout_sec_ = dbl("pose_timeout_sec", 0.5);
  auto_reset_origin_on_start_ = bl("auto_reset_origin_on_start", false);
  auto_reset_origin_timeout_sec_ = dbl("auto_reset_origin_timeout_sec", 3.0);
}

void PositioningBridgeNode::tryOpenSerial()
{
  if (serial_.isOpen()) {
    return;
  }

  std::string error_message;
  if (!serial_.open(serial_config_, error_message)) {
    last_serial_error_ = error_message;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
      static_cast<int64_t>(reopen_interval_sec_ * 1000.0),
      "串口打开失败: %s", error_message.c_str());
    return;
  }

  last_serial_error_.clear();
  decoder_->reset();
  time_sync_estimator_.reset();
  RCLCPP_INFO(this->get_logger(), "已连接定位串口: %s @ %d", serial_config_.device.c_str(), serial_config_.baudrate);
}

void PositioningBridgeNode::readSerialTick()
{
  tryOpenSerial();
  if (!serial_.isOpen()) {
    return;
  }

  std::array<uint8_t, 512> rx_buffer{};
  std::string error_message;
  const auto bytes_read = serial_.readSome(rx_buffer.data(), rx_buffer.size(), serial_config_.read_timeout_ms, error_message);
  if (bytes_read < 0) {
    last_serial_error_ = error_message;
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "串口读取失败: %s", error_message.c_str());
    serial_.close();
    return;
  }
  if (bytes_read == 0) {
    return;
  }

  std::vector<DecodedFrame> frames;
  decoder_->feed(rx_buffer.data(), static_cast<std::size_t>(bytes_read), frames);
  if (!frames.empty()) {
    RCLCPP_INFO_ONCE(this->get_logger(), "首次解析到 %zu 帧 (bytes_read=%zd)", frames.size(), bytes_read);
  }
  for (const auto & frame : frames) {
    handleFrame(frame);
  }
}

void PositioningBridgeNode::timeSyncTick()
{
  if (protocol_mode_ != ProtocolMode::BINARY_V1 || !serial_.isOpen()) {
    return;
  }

  const uint64_t host_time_us = static_cast<uint64_t>(this->now().nanoseconds() / 1000LL);
  const auto frame = buildTimeSyncRequestFrame(next_tx_sequence_++, host_time_us);
  std::string error_message;
  if (!serial_.writeAll(frame, error_message)) {
    last_serial_error_ = error_message;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "时间同步请求发送失败: %s", error_message.c_str());
    serial_.close();
  }
}

void PositioningBridgeNode::diagnosticsTick()
{
  maybeAutoResetOrigin();
  publishDiagnostics();
}

bool PositioningBridgeNode::sendSetLocalOrigin(float x, float y, float yaw, std::string & err)
{
  if (protocol_mode_ != ProtocolMode::BINARY_V1) {
    err = "当前协议模式不是 binary_v1";
    return false;
  }
  if (!serial_.isOpen()) {
    err = "串口未连接";
    return false;
  }
  const auto frame = buildSetLocalOriginFrame(next_tx_sequence_++, x, y, yaw);
  if (!serial_.writeAll(frame, err)) {
    last_serial_error_ = err;
    serial_.close();
    return false;
  }
  return true;
}

void PositioningBridgeNode::maybeAutoResetOrigin()
{
  if (origin_reset_done_ || !auto_reset_origin_on_start_) {
    return;
  }
  if (protocol_mode_ != ProtocolMode::BINARY_V1 || !serial_.isOpen()) {
    return;
  }
  const bool sync_locked = time_sync_estimator_.locked();
  const double elapsed = (this->now() - node_start_time_).seconds();
  const bool timeout = elapsed >= auto_reset_origin_timeout_sec_;
  if (!sync_locked && !timeout) {
    return;
  }
  std::string err;
  if (sendSetLocalOrigin(0.0F, 0.0F, 0.0F, err)) {
    origin_reset_done_ = true;
    RCLCPP_INFO(this->get_logger(),
      "启动自动归零完成 (sync_locked=%s, elapsed=%.2fs)",
      sync_locked ? "true" : "false", elapsed);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "启动自动归零发送失败: %s（将重试）", err.c_str());
  }
}

void PositioningBridgeNode::handleResetLocalOrigin(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  std::string err;
  if (sendSetLocalOrigin(0.0F, 0.0F, 0.0F, err)) {
    response->success = true;
    response->message = "已发送 SET_LOCAL_ORIGIN(0,0,0) 请求，等待 STM32 ACK";
  } else {
    response->success = false;
    response->message = "发送重定位命令失败: " + err;
  }
}

void PositioningBridgeNode::handleFrame(const DecodedFrame & frame)
{
  switch (frame.kind) {
    case FrameKind::ODOM_POSE:
    case FrameKind::ODOM_STATE:
      if (frame.pose.has_value()) {
        publishPose(*frame.pose, frame.sequence);
      }
      break;
    case FrameKind::STATUS:
    case FrameKind::HEARTBEAT:
      latest_status_ = frame.status;
      break;
    case FrameKind::TIME_SYNC_RESPONSE:
      if (frame.time_sync.has_value()) {
        const uint64_t recv_time_us = static_cast<uint64_t>(this->now().nanoseconds() / 1000LL);
        time_sync_estimator_.update(frame.time_sync->echoed_host_time_us, recv_time_us, frame.time_sync->mcu_time_us);
      }
      break;
    case FrameKind::SET_LOCAL_ORIGIN_ACK:
      if (frame.ack.has_value()) {
        RCLCPP_INFO(this->get_logger(), "收到重定位 ACK: seq=%u result=%u event=%u",
          frame.ack->acked_seq, frame.ack->result_code, frame.ack->event_counter);
      }
      break;
    case FrameKind::IR_TRIGGER: {
      // 红外传感器检测到球经过，发布触发事件
      std_msgs::msg::Bool ir_msg;
      ir_msg.data = true;
      ir_trigger_pub_->publish(ir_msg);
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
        "红外触发事件 seq=%u", frame.sequence);
      break;
    }
    default:
      break;
  }
}

void PositioningBridgeNode::publishPose(const PoseSample & incoming_pose, uint16_t sequence)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "首次发布 odom: x=%.3f y=%.3f yaw=%.3f",
    incoming_pose.x, incoming_pose.y, incoming_pose.yaw);
  PoseSample pose_sample = incoming_pose;
  const auto stamp = resolveStamp(pose_sample);
  maybeUpdateDerivedVelocity(pose_sample, stamp);
  auto odom_msg = buildOdometry(pose_sample, stamp);
  odom_pub_->publish(odom_msg);
  last_pose_stamp_ = stamp;

  if (publish_tf_) {
    publishTransform(odom_msg);
  }

  RCLCPP_DEBUG(this->get_logger(),
    "发布 odom seq=%u x=%.3f y=%.3f yaw=%.3f vx=%.3f vy=%.3f wz=%.3f",
    sequence, pose_sample.x, pose_sample.y, pose_sample.yaw, pose_sample.vx, pose_sample.vy, pose_sample.wz);
}

void PositioningBridgeNode::publishDiagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray array_msg;
  array_msg.header.stamp = this->now();

  diagnostic_msgs::msg::DiagnosticStatus status_msg;
  status_msg.name = "positioning_bridge";
  status_msg.hardware_id = serial_config_.device;

  const bool serial_ok = serial_.isOpen();
  const bool pose_recent = last_pose_stamp_.nanoseconds() > 0 &&
    (this->now() - last_pose_stamp_).seconds() <= pose_timeout_sec_;

  if (!serial_ok) {
    status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status_msg.message = "serial_disconnected";
  } else if (!pose_recent) {
    status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status_msg.message = "pose_timeout";
  } else if (protocol_mode_ == ProtocolMode::BINARY_V1 && !time_sync_estimator_.locked()) {
    status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status_msg.message = "time_sync_not_locked";
  } else {
    status_msg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status_msg.message = "ok";
  }

  auto push_key_value = [&status_msg](const std::string & key, const std::string & value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status_msg.values.push_back(kv);
  };

  push_key_value("protocol_mode", protocol_mode_ == ProtocolMode::LEGACY_ASCII ? "legacy_ascii" : "binary_v1");
  push_key_value("serial_open", serial_ok ? "true" : "false");
  push_key_value("time_sync_locked", time_sync_estimator_.locked() ? "true" : "false");
  push_key_value("time_sync_samples", std::to_string(time_sync_estimator_.sampleCount()));
  push_key_value("time_sync_offset_us", std::to_string(time_sync_estimator_.offsetUs()));
  push_key_value("time_sync_rtt_us", std::to_string(time_sync_estimator_.rttUs()));
  push_key_value("last_serial_error", last_serial_error_);
  push_key_value("frames_ok", std::to_string(decoder_->stats().frames_ok));
  push_key_value("frames_crc_error", std::to_string(decoder_->stats().frames_crc_error));
  push_key_value("frames_parse_error", std::to_string(decoder_->stats().frames_parse_error));
  push_key_value("bytes_rx", std::to_string(decoder_->stats().bytes_rx));

  if (latest_status_.has_value()) {
    push_key_value("status_bits", std::to_string(latest_status_->status_bits));
    push_key_value("quality", std::to_string(latest_status_->quality));
    push_key_value("link_state", std::to_string(latest_status_->link_state));
    push_key_value("uptime_ms", std::to_string(latest_status_->uptime_ms));
    push_key_value("error_code", std::to_string(latest_status_->error_code));
  }

  array_msg.status.push_back(status_msg);
  diagnostics_pub_->publish(array_msg);
}

void PositioningBridgeNode::publishTransform(const nav_msgs::msg::Odometry & odom_msg)
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header = odom_msg.header;
  tf_msg.child_frame_id = odom_msg.child_frame_id;
  tf_msg.transform.translation.x = odom_msg.pose.pose.position.x;
  tf_msg.transform.translation.y = odom_msg.pose.pose.position.y;
  tf_msg.transform.translation.z = odom_msg.pose.pose.position.z;
  tf_msg.transform.rotation = odom_msg.pose.pose.orientation;
  tf_broadcaster_->sendTransform(tf_msg);
}

nav_msgs::msg::Odometry PositioningBridgeNode::buildOdometry(const PoseSample & pose_sample, const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.stamp = stamp;
  odom_msg.header.frame_id = frame_id_;
  odom_msg.child_frame_id = child_frame_id_;

  odom_msg.pose.pose.position.x = pose_sample.x;
  odom_msg.pose.pose.position.y = pose_sample.y;
  odom_msg.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, pose_sample.yaw);
  odom_msg.pose.pose.orientation.x = q.x();
  odom_msg.pose.pose.orientation.y = q.y();
  odom_msg.pose.pose.orientation.z = q.z();
  odom_msg.pose.pose.orientation.w = q.w();

  odom_msg.twist.twist.linear.x = pose_sample.vx;
  odom_msg.twist.twist.linear.y = pose_sample.vy;
  odom_msg.twist.twist.linear.z = 0.0;
  odom_msg.twist.twist.angular.x = 0.0;
  odom_msg.twist.twist.angular.y = 0.0;
  odom_msg.twist.twist.angular.z = pose_sample.wz;

  const bool good_quality = pose_sample.quality > 0 || pose_sample.status_bits != 0;
  const double pose_cov = good_quality ? 0.02 : 0.1;
  const double twist_cov = pose_sample.has_velocity ? 0.05 : 0.2;
  odom_msg.pose.covariance[0] = pose_cov;
  odom_msg.pose.covariance[7] = pose_cov;
  odom_msg.pose.covariance[35] = pose_cov * 2.0;
  odom_msg.twist.covariance[0] = twist_cov;
  odom_msg.twist.covariance[7] = twist_cov;
  odom_msg.twist.covariance[35] = twist_cov * 2.0;

  return odom_msg;
}

rclcpp::Time PositioningBridgeNode::resolveStamp(const PoseSample & pose_sample) const
{
  if (pose_sample.source_time_valid && protocol_mode_ == ProtocolMode::BINARY_V1) {
    auto mapped = time_sync_estimator_.toRosTime(pose_sample.source_time_us);
    if (mapped.has_value()) {
      return *mapped;
    }
  }
  return this->now();
}

void PositioningBridgeNode::maybeUpdateDerivedVelocity(PoseSample & pose_sample, const rclcpp::Time & stamp)
{
  if (pose_sample.has_velocity || !derive_velocity_from_pose_) {
    previous_pose_ = PoseHistory{pose_sample.x, pose_sample.y, pose_sample.yaw, stamp, true};
    return;
  }

  if (previous_pose_.valid) {
    const double dt = (stamp - previous_pose_.stamp).seconds();
    if (dt > 1e-4) {
      const double dx_world = pose_sample.x - previous_pose_.x;
      const double dy_world = pose_sample.y - previous_pose_.y;
      const double dyaw = unwrapAngle(previous_pose_.yaw, pose_sample.yaw);

      const double vx_world = dx_world / dt;
      const double vy_world = dy_world / dt;
      const double cos_yaw = std::cos(pose_sample.yaw);
      const double sin_yaw = std::sin(pose_sample.yaw);

      pose_sample.vx = cos_yaw * vx_world + sin_yaw * vy_world;
      pose_sample.vy = -sin_yaw * vx_world + cos_yaw * vy_world;
      pose_sample.wz = dyaw / dt;
      pose_sample.has_velocity = true;
    }
  }

  previous_pose_ = PoseHistory{pose_sample.x, pose_sample.y, pose_sample.yaw, stamp, true};
}

double PositioningBridgeNode::unwrapAngle(double previous, double current)
{
  double diff = current - previous;
  while (diff > M_PI) {
    diff -= 2.0 * M_PI;
  }
  while (diff < -M_PI) {
    diff += 2.0 * M_PI;
  }
  return diff;
}

}  // namespace positioning_bridge_ros2

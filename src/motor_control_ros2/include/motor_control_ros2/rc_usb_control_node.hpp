#ifndef MOTOR_CONTROL_ROS2__RC_USB_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_ROS2__RC_USB_CONTROL_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/u_int8.hpp>

namespace motor_control {

class RCUsbControlNode : public rclcpp::Node {
public:
  RCUsbControlNode();
  ~RCUsbControlNode() override;

private:
  struct RcForwardFrame {
    uint16_t magic = 0;
    uint8_t version = 0;
    uint8_t payload_len = 0;
    uint16_t seq = 0;
    uint32_t tick_ms = 0;
    int16_t lx = 0;        // 左摇杆左右，右推为正
    int16_t ly = 0;        // 左摇杆前后，前推为正
    int16_t rx = 0;        // 右摇杆左右，右推为正
    int16_t ry = 0;        // 右摇杆前后，当前未用于底盘控制
    uint8_t sw_left = 0;   // 左三档拨杆: UP=1, MID=3, DOWN=2
    uint8_t sw_right = 0;  // 右三档拨杆: UP=1, MID=3, DOWN=2
    uint16_t reserved = 0;
    uint16_t crc16 = 0;
  };

  enum class ControlMode {
    ESTOP,
    MANUAL,
    VISION
  };

  void configureParameters();
  void startSerialLoop();
  void stopSerialLoop();
  void serialReadLoop();
  int openSerialPort();
  void closeSerialPort();
  bool setupSerialPort(int fd);

  void onSerialData(const uint8_t* data, std::size_t len);
  std::vector<RcForwardFrame> parseBufferedFrames();
  bool validateFrame(const uint8_t* data, const RcForwardFrame& frame);
  void updateCommand(const RcForwardFrame& frame);
  void publishCommand();
  void visionCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void logDiagnostics(const rclcpp::Time& now);

  ControlMode modeFromSwitch(uint8_t sw) const;
  uint8_t modeSwitchValue(const RcForwardFrame& frame) const;
  geometry_msgs::msg::Twist commandFromFrame(const RcForwardFrame& frame) const;
  geometry_msgs::msg::Twist zeroTwist() const;
  double normalizeAxis(int16_t raw) const;
  double clamp(double value, double min_value, double max_value) const;

  static RcForwardFrame parseFrame(const uint8_t* data);
  static uint16_t readLe16(const uint8_t* data);
  static uint32_t readLe32(const uint8_t* data);
  static int16_t readLeI16(const uint8_t* data);
  static uint16_t crc16Modbus(const uint8_t* data, std::size_t len);
  static int baudRateToTermios(int baud_rate);

  std::string serial_port_{"/dev/ttyACM1"};
  std::string cmd_vel_topic_{"/cmd_vel"};
  std::string estop_topic_{"/chassis/estop"};
  std::string vision_cmd_vel_topic_{"/vision/cmd_vel"};
  std::string strike_switch_topic_{"/rc/strike_switch"};
  std::string strike_switch_field_{"right"};
  std::string mode_switch_field_{"left"};
  int baud_rate_{115200};
  int serial_read_chunk_{256};
  int serial_reconnect_interval_ms_{1000};

  double max_linear_velocity_{0.5};
  double max_angular_velocity_{1.0};
  double axis_max_{660.0};
  double deadzone_{0.05};
  double publish_rate_{50.0};
  double command_timeout_{0.3};
  double diagnostics_interval_sec_{1.0};
  double vision_timeout_{0.3};
  double serial_stale_reconnect_sec_{2.0};

  int manual_switch_{1};
  int estop_switch_{3};
  int vision_switch_{2};
  bool invert_linear_x_{false};
  bool invert_linear_y_{false};
  bool invert_angular_z_{false};
  bool publish_zero_before_first_frame_{true};
  bool enable_vision_passthrough_{true};
  bool enable_strike_switch_output_{true};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr strike_switch_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vision_cmd_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::thread serial_thread_;
  std::atomic<bool> running_{false};
  std::atomic<int> serial_fd_{-1};

  std::vector<uint8_t> rx_buffer_;
  mutable std::mutex buffer_mutex_;
  mutable std::mutex command_mutex_;

  geometry_msgs::msg::Twist last_manual_cmd_;
  geometry_msgs::msg::Twist last_vision_cmd_;
  rclcpp::Time last_frame_time_;
  rclcpp::Time last_vision_cmd_time_;
  rclcpp::Time last_diagnostics_time_;
  ControlMode control_mode_{ControlMode::ESTOP};
  bool has_valid_frame_{false};
  bool has_vision_cmd_{false};

  uint64_t rx_bytes_{0};
  uint64_t valid_frames_{0};
  uint64_t invalid_frames_{0};
  uint64_t dropped_bytes_{0};
  uint16_t last_seq_{0};
  uint8_t last_sw_left_{0};
  uint8_t last_sw_right_{0};
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_ROS2__RC_USB_CONTROL_NODE_HPP_

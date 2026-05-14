#ifndef MOTOR_CONTROL_ROS2__TEACH_PLAYBACK_CONTROL_NODE_HPP_
#define MOTOR_CONTROL_ROS2__TEACH_PLAYBACK_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"

#include <string>
#include <vector>
#include <map>
#include <cmath>
#include <array>

namespace motor_control {

constexpr double GEAR_RATIO = 6.33;
constexpr int NUM_MOTORS = 4;
constexpr std::array<int, 4> ALL_IDS = {0, 1, 2, 3};
constexpr double TEMP_CRITICAL = 85.0;

enum class State {
    IDLE,
    GOTO_START,
    PLAYBACK,
    HOLD,
    RETURN_ZERO
};

// 1000Hz 预插值轨迹
struct PlaybackTrajectory {
    double duration;
    int n_frames;       // 1000Hz 帧数
    // 预插值到 1000Hz 的三通道数据 [motor][frame]
    std::array<std::vector<double>, 4> pos;
    std::array<std::vector<double>, 4> vel;
    std::array<std::vector<double>, 4> torque;
};

class TeachPlaybackControlNode : public rclcpp::Node {
public:
    TeachPlaybackControlNode();
    void start();
    void brake_all();

private:
    void load_trajectory();
    void control_loop();

    void state_callback(
        const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg);
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void ir_callback(const std_msgs::msg::Bool::SharedPtr msg);

    void send_cmd(int id, double torque_ff, double kp, double kd,
                  double pos_target, double vel_target = 0.0);
    double smoothstep(double t);
    double gravity_torque(int mid, double theta1, double theta2);
    std::pair<double, double> get_theta_pair();
    bool check_safety();
    bool motors_online();

    // ── ROS 接口 ──
    rclcpp::Publisher<motor_control_ros2::msg::UnitreeGO8010Command>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
    rclcpp::Subscription<motor_control_ros2::msg::UnitreeGO8010State>::SharedPtr motor_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ir_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // ── 参数 ──
    std::string teach_file_;
    int rec_index_;
    double kp_, kd_;
    double kp_hold_, kd_hold_;
    double speed_;
    double filter_tau_;
    double tau_inner_, tau_outer_;
    double gravity_offset_inner_, gravity_offset_outer_;
    double goto_time_, hold_time_;
    std::string trigger_source_;
    int joy_button_;

    // ── 电机实时状态 ──
    std::map<int, double> positions_;
    std::map<int, double> velocities_;
    std::map<int, double> temperatures_;
    std::map<int, int> errors_;

    // ── 状态机 ──
    State state_;
    bool trigger_pending_;
    double phase_start_;
    std::array<double, 4> initial_pos_;
    int loop_count_;

    // ── 轨迹 ──
    PlaybackTrajectory traj_;
    std::array<double, 4> start_pos_, end_pos_;
    std::array<double, 4> start_vel_, end_vel_;
    std::array<double, 4> start_torque_, end_torque_;

    static bool is_inner(int mid) { return mid == 0 || mid == 2; }
};

}  // namespace motor_control

#endif  // MOTOR_CONTROL_ROS2__TEACH_PLAYBACK_CONTROL_NODE_HPP_

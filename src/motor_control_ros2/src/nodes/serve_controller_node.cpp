/**
 * @file serve_controller_node.cpp
 * @brief 发球协调控制节点
 *
 * 整合 Delta 机械臂（垫球盘）与 DJI 3508 发球装置的连贯发球流程：
 *
 * 状态机:
 *   IDLE          等待触发
 *   CHARGE_ARM    垫球盘下降到接球位（→ delta_arm_manager 执行）
 *   CHARGE_STRIKE 发球臂顺时针蓄力到指定角度
 *   WAIT_DELAY    垫球完成，等待可配置的延迟后触发击球（关键时序）
 *   STRIKING      发球臂直接电流击球，累积转角达标后停止
 *   RETURNING     发球臂慢速回零；垫球盘复位
 *
 * 输入:
 *   /joy                          手柄输入（sensor_msgs/Joy）
 *   /delta_arm/ready              Delta 臂就绪通知（std_msgs/String）
 *   /dji_motor_states             DJI 电机状态（motor_control_ros2/DJIMotorState）
 *
 * 输出:
 *   /delta_arm/target             Delta 臂目标（motor_control_ros2/ArmTarget）
 *   /dji_motor_command_advanced   DJI 电机指令（motor_control_ros2/DJIMotorCommandAdvanced）
 *
 * 手柄 / 键盘操作:
 *   手柄 LB（默认索引4）或 键盘 Enter → 启动完整发球序列
 *   手柄 RB（默认索引5）或 键盘 'q'   → 立即停止/复位
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "motor_control_ros2/msg/arm_target.hpp"
#include "motor_control_ros2/msg/dji_motor_command_advanced.hpp"
#include "motor_control_ros2/msg/dji_motor_state.hpp"

#include <yaml-cpp/yaml.h>
#include <atomic>
#include <cmath>
#include <chrono>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using ArmTarget         = motor_control_ros2::msg::ArmTarget;
using DJICmd            = motor_control_ros2::msg::DJIMotorCommandAdvanced;
using DJIState          = motor_control_ros2::msg::DJIMotorState;

static constexpr uint8_t MODE_DIRECT   = 0;
static constexpr uint8_t MODE_VELOCITY = 1;
static constexpr uint8_t MODE_POSITION = 2;

// ============================================================
// 状态机
// ============================================================
enum class ServeState {
    IDLE,           // 等待触发
    CHARGE_ARM,     // 垫球盘下降，等待 delta_arm/ready
    CHARGE_STRIKE,  // 发球臂顺时针蓄力到 charge_angle
    WAIT_DELAY,     // 垫球完成，等待间隔时间
    STRIKING,       // 发球臂直接电流击打
    RETURNING,      // 发球臂回零 + 垫球盘复位
};

[[maybe_unused]] static const char* stateStr(ServeState s) {
    switch (s) {
        case ServeState::IDLE:           return "IDLE";
        case ServeState::CHARGE_ARM:     return "CHARGE_ARM";
        case ServeState::CHARGE_STRIKE:  return "CHARGE_STRIKE";
        case ServeState::WAIT_DELAY:     return "WAIT_DELAY";
        case ServeState::STRIKING:       return "STRIKING";
        case ServeState::RETURNING:      return "RETURNING";
    }
    return "UNKNOWN";
}

// ============================================================
// 配置参数
// ============================================================
struct ServeConfig {
    // --- 手柄按钮 ---
    int button_trigger  = 4;   // LB：启动完整发球序列
    int button_abort    = 5;   // RB：立即停止

    // --- Delta 臂 ---
    double arm_descend_rad = -0.8; // 垫球盘下降量（弧度，负值=向下）
    double arm_wait_timeout = 5.0; // 等待 delta_arm/ready 超时（秒）

    // --- DJI 发球电机 ---
    std::string motor1_name   = "DJI3508_1";  // 主电机
    std::string motor2_name   = "DJI3508_2";  // 副电机（对称，反向）
    bool        use_motor2    = true;

    double charge_angle_deg   = 240.0;  // 蓄力目标角度（度）
    double charge_speed_rpm   = 1000.0; // 蓄力速度（RPM）
    double charge_threshold_deg = 5.0;  // 蓄力到位阈值（度）

    int    strike_current     = -16384; // 击球电流（负=逆时针）
    double strike_rotation_deg = 450.0; // 击球累积转角（度）
    double strike_timeout_s   = 2.0;    // 击球超时保护（秒）

    double return_speed_rpm   = 1500.0; // 回零速度（RPM）
    double return_threshold_deg = 5.0;  // 回零到位阈值（度）

    // --- 关键时序 ---
    double delay_arm_to_strike_s = 0.3; // 垫球完成 → 发球间隔（秒）

    // --- 控制频率 ---
    double control_hz = 200.0;
};

// ============================================================
// 节点
// ============================================================
class ServeControllerNode : public rclcpp::Node {
public:
    ServeControllerNode() : Node("serve_controller_node") {
        loadConfig();

        // 订阅
        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) { joyCallback(msg); });

        arm_ready_sub_ = create_subscription<std_msgs::msg::String>(
            "/delta_arm/ready", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) { armReadyCallback(msg); });

        dji_state_sub_ = create_subscription<DJIState>(
            "/dji_motor_states", 10,
            [this](const DJIState::SharedPtr msg) { djiStateCallback(msg); });

        // 发布
        arm_target_pub_ = create_publisher<ArmTarget>("/delta_arm/target", 10);
        dji_cmd_pub_    = create_publisher<DJICmd>("/dji_motor_command_advanced", 10);

        // 控制定时器
        auto period = std::chrono::duration<double>(1.0 / cfg_.control_hz);
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { controlLoop(); });

        // 键盘监听线程（非阻塞读取 stdin）
        keyboard_thread_ = std::thread([this]() { keyboardThread(); });

        RCLCPP_INFO(get_logger(),
            "发球协调控制节点启动\n"
            "  手柄 LB(btn%d) / 键盘 Enter → 启动发球序列\n"
            "  手柄 RB(btn%d) / 键盘 'q'   → 立即停止\n"
            "  关键延迟: %.2f s",
            cfg_.button_trigger, cfg_.button_abort,
            cfg_.delay_arm_to_strike_s);
    }

    ~ServeControllerNode() {
        keyboard_running_ = false;
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
    }

private:
    // ── 配置 ────────────────────────────────────────────────
    ServeConfig cfg_;

    // ── ROS 接口 ─────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr      joy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr      arm_ready_sub_;
    rclcpp::Subscription<DJIState>::SharedPtr                   dji_state_sub_;
    rclcpp::Publisher<ArmTarget>::SharedPtr                     arm_target_pub_;
    rclcpp::Publisher<DJICmd>::SharedPtr                        dji_cmd_pub_;
    rclcpp::TimerBase::SharedPtr                                control_timer_;

    // ── 状态机 ──────────────────────────────────────────────
    ServeState state_ = ServeState::IDLE;
    rclcpp::Time phase_start_;

    // ── 电机状态（主电机）────────────────────────────────────
    double  dji_angle_deg_    = 0.0;  // 当前角度（度）
    double  dji_last_angle_   = 0.0;  // 上一帧（用于累积计算）
    double  strike_rotated_   = 0.0;  // 累积已转角度（击打阶段）
    bool    dji_state_valid_  = false;

    // ── 原子信号 ─────────────────────────────────────────────
    std::atomic<bool> trigger_pressed_ {false}; // 启动信号
    std::atomic<bool> abort_pressed_   {false}; // 中止信号
    std::atomic<bool> arm_ready_       {false}; // Delta臂就绪

    // ── 手柄按键上一帧 ────────────────────────────────────────
    int joy_trigger_prev_ = 0;
    int joy_abort_prev_   = 0;

    // ── 键盘线程 ─────────────────────────────────────────────
    std::thread keyboard_thread_;
    std::atomic<bool> keyboard_running_ {true};

    // ────────────────────────────────────────────────────────
    // 配置加载
    // ────────────────────────────────────────────────────────
    void loadConfig() {
        std::string pkg_share;
        try {
            pkg_share = ament_index_cpp::get_package_share_directory("motor_control_ros2");
        } catch (...) {
            RCLCPP_WARN(get_logger(), "无法获取 package share 目录，使用默认配置");
            return;
        }

        std::string config_file = pkg_share + "/config/serve_controller_params.yaml";
        try {
            YAML::Node root = YAML::LoadFile(config_file);
            auto p = root["serve_controller_node"]["ros__parameters"];
            if (!p) { RCLCPP_WARN(get_logger(), "配置文件格式异常，使用默认值"); return; }

            auto get_d = [&](const char* k, double def) -> double {
                return p[k] ? p[k].as<double>() : def;
            };
            auto get_i = [&](const char* k, int def) -> int {
                return p[k] ? p[k].as<int>() : def;
            };
            auto get_s = [&](const char* k, const std::string& def) -> std::string {
                return p[k] ? p[k].as<std::string>() : def;
            };
            auto get_b = [&](const char* k, bool def) -> bool {
                return p[k] ? p[k].as<bool>() : def;
            };

            cfg_.button_trigger         = get_i("button_trigger",        cfg_.button_trigger);
            cfg_.button_abort           = get_i("button_abort",           cfg_.button_abort);
            cfg_.arm_descend_rad        = get_d("arm_descend_rad",        cfg_.arm_descend_rad);
            cfg_.arm_wait_timeout       = get_d("arm_wait_timeout",       cfg_.arm_wait_timeout);
            cfg_.motor1_name            = get_s("motor1_name",            cfg_.motor1_name);
            cfg_.motor2_name            = get_s("motor2_name",            cfg_.motor2_name);
            cfg_.use_motor2             = get_b("use_motor2",             cfg_.use_motor2);
            cfg_.charge_angle_deg       = get_d("charge_angle_deg",       cfg_.charge_angle_deg);
            cfg_.charge_speed_rpm       = get_d("charge_speed_rpm",       cfg_.charge_speed_rpm);
            cfg_.charge_threshold_deg   = get_d("charge_threshold_deg",   cfg_.charge_threshold_deg);
            cfg_.strike_current         = get_i("strike_current",         cfg_.strike_current);
            cfg_.strike_rotation_deg    = get_d("strike_rotation_deg",    cfg_.strike_rotation_deg);
            cfg_.strike_timeout_s       = get_d("strike_timeout_s",       cfg_.strike_timeout_s);
            cfg_.return_speed_rpm       = get_d("return_speed_rpm",       cfg_.return_speed_rpm);
            cfg_.return_threshold_deg   = get_d("return_threshold_deg",   cfg_.return_threshold_deg);
            cfg_.delay_arm_to_strike_s  = get_d("delay_arm_to_strike_s",  cfg_.delay_arm_to_strike_s);
            cfg_.control_hz             = get_d("control_hz",             cfg_.control_hz);

            RCLCPP_INFO(get_logger(), "配置加载成功: %s", config_file.c_str());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "配置加载失败（%s），使用默认值", e.what());
        }
    }

    // ────────────────────────────────────────────────────────
    // 回调
    // ────────────────────────────────────────────────────────
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        const auto& b = msg->buttons;

        // 触发键上升沿
        if (cfg_.button_trigger < static_cast<int>(b.size())) {
            if (b[cfg_.button_trigger] && !joy_trigger_prev_) {
                trigger_pressed_.store(true);
            }
            joy_trigger_prev_ = b[cfg_.button_trigger];
        }
        // 中止键上升沿
        if (cfg_.button_abort < static_cast<int>(b.size())) {
            if (b[cfg_.button_abort] && !joy_abort_prev_) {
                abort_pressed_.store(true);
            }
            joy_abort_prev_ = b[cfg_.button_abort];
        }
    }

    void armReadyCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "READY") {
            arm_ready_.store(true);
        }
    }

    void djiStateCallback(const DJIState::SharedPtr msg) {
        if (msg->joint_name == cfg_.motor1_name) {
            dji_last_angle_ = dji_angle_deg_;
            dji_angle_deg_  = msg->angle;
            dji_state_valid_ = true;
        }
    }

    // ────────────────────────────────────────────────────────
    // 键盘线程（非阻塞 stdin）
    // ────────────────────────────────────────────────────────
    void keyboardThread() {
        // 设置 stdin 为非阻塞 raw 模式
        struct termios orig, raw;
        tcgetattr(STDIN_FILENO, &orig);
        raw = orig;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN]  = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        while (keyboard_running_) {
            char c = 0;
            if (read(STDIN_FILENO, &c, 1) == 1) {
                if (c == '\n' || c == '\r') {
                    trigger_pressed_.store(true);
                    RCLCPP_INFO(get_logger(), "[键盘] Enter → 触发发球序列");
                } else if (c == 'q' || c == 'Q') {
                    abort_pressed_.store(true);
                    RCLCPP_INFO(get_logger(), "[键盘] q → 停止");
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        tcsetattr(STDIN_FILENO, TCSANOW, &orig);
    }

    // ────────────────────────────────────────────────────────
    // 主控制循环（200 Hz 定时触发）
    // ────────────────────────────────────────────────────────
    void controlLoop() {
        // 中止信号：任何状态下立即回零
        if (abort_pressed_.exchange(false) && state_ != ServeState::IDLE) {
            RCLCPP_WARN(get_logger(), "接收到中止信号，强制回零");
            enterReturning();
            return;
        }

        switch (state_) {
            case ServeState::IDLE:          updateIdle();         break;
            case ServeState::CHARGE_ARM:    updateChargeArm();    break;
            case ServeState::CHARGE_STRIKE: updateChargeStrike(); break;
            case ServeState::WAIT_DELAY:    updateWaitDelay();    break;
            case ServeState::STRIKING:      updateStriking();     break;
            case ServeState::RETURNING:     updateReturning();    break;
        }
    }

    // ────────────────────────────────────────────────────────
    // 状态更新函数
    // ────────────────────────────────────────────────────────

    // IDLE：等待触发
    void updateIdle() {
        if (trigger_pressed_.exchange(false)) {
            RCLCPP_INFO(get_logger(), "触发发球序列 → CHARGE_ARM");
            enterChargeArm();
        }
    }

    // CHARGE_ARM：发送垫球盘下降指令，等待 delta_arm/ready
    void enterChargeArm() {
        arm_ready_.store(false);
        state_ = ServeState::CHARGE_ARM;
        phase_start_ = now();

        ArmTarget msg;
        msg.header.stamp = now();
        msg.target_angles[0] = cfg_.arm_descend_rad;
        msg.target_angles[1] = cfg_.arm_descend_rad;
        msg.target_angles[2] = cfg_.arm_descend_rad;
        msg.execute = true;
        arm_target_pub_->publish(msg);

        RCLCPP_INFO(get_logger(), "垫球盘下降 %.3f rad，等待就绪信号...", cfg_.arm_descend_rad);
    }

    void updateChargeArm() {
        double elapsed = (now() - phase_start_).seconds();

        if (arm_ready_.exchange(false)) {
            RCLCPP_INFO(get_logger(), "垫球盘就绪 → CHARGE_STRIKE");
            enterChargeStrike();
            return;
        }
        if (elapsed > cfg_.arm_wait_timeout) {
            RCLCPP_WARN(get_logger(), "等待垫球臂超时 (%.1fs)，跳过直接蓄力", cfg_.arm_wait_timeout);
            enterChargeStrike();
        }
    }

    // CHARGE_STRIKE：DJI 电机顺时针蓄力到 charge_angle
    void enterChargeStrike() {
        state_ = ServeState::CHARGE_STRIKE;
        phase_start_ = now();
        RCLCPP_INFO(get_logger(), "发球臂蓄力 → %.1f°", cfg_.charge_angle_deg);
    }

    void updateChargeStrike() {
        if (!dji_state_valid_) return;

        double remaining = angleDiffCW(dji_angle_deg_, cfg_.charge_angle_deg);

        if (remaining < cfg_.charge_threshold_deg) {
            sendDjiPosition(cfg_.charge_angle_deg);
            RCLCPP_INFO(get_logger(), "蓄力完成（%.1f°）→ WAIT_DELAY %.2fs",
                dji_angle_deg_, cfg_.delay_arm_to_strike_s);
            state_ = ServeState::WAIT_DELAY;
            phase_start_ = now();
        } else {
            sendDjiVelocity(cfg_.charge_speed_rpm);
        }
    }

    // WAIT_DELAY：等待间隔后击球（关键时序参数）
    void updateWaitDelay() {
        // 蓄力阶段持续发送位置保持
        sendDjiPosition(cfg_.charge_angle_deg);

        double elapsed = (now() - phase_start_).seconds();
        if (elapsed >= cfg_.delay_arm_to_strike_s) {
            RCLCPP_INFO(get_logger(), "延迟 %.2fs 结束 → STRIKING", cfg_.delay_arm_to_strike_s);
            enterStriking();
        }
    }

    // STRIKING：直接电流击打，累积转角达标停止
    void enterStriking() {
        state_ = ServeState::STRIKING;
        phase_start_ = now();
        strike_rotated_ = 0.0;
        dji_last_angle_ = dji_angle_deg_;
        RCLCPP_INFO(get_logger(), "发球击打开始，目标转角 %.1f°", cfg_.strike_rotation_deg);
    }

    void updateStriking() {
        if (!dji_state_valid_) return;

        // 累积逆时针转角
        double delta = std::fmod(dji_last_angle_ - dji_angle_deg_ + 360.0, 360.0);
        if (delta < 180.0) {
            strike_rotated_ += delta;
        }
        dji_last_angle_ = dji_angle_deg_;

        double elapsed = (now() - phase_start_).seconds();

        if (strike_rotated_ >= cfg_.strike_rotation_deg || elapsed >= cfg_.strike_timeout_s) {
            if (elapsed >= cfg_.strike_timeout_s) {
                RCLCPP_WARN(get_logger(), "击球超时（转角 %.1f°）", strike_rotated_);
            } else {
                RCLCPP_INFO(get_logger(), "击球完成（转角 %.1f°）→ RETURNING", strike_rotated_);
            }
            enterReturning();
        } else {
            sendDjiDirect(cfg_.strike_current);
        }
    }

    // RETURNING：发球臂回零 + 垫球盘复位
    void enterReturning() {
        state_ = ServeState::RETURNING;
        phase_start_ = now();
        dji_last_angle_ = dji_angle_deg_;

        // 同步发送垫球盘复位（返回 0 偏移）
        ArmTarget msg;
        msg.header.stamp = now();
        msg.target_angles[0] = -cfg_.arm_descend_rad; // 回零偏移
        msg.target_angles[1] = -cfg_.arm_descend_rad;
        msg.target_angles[2] = -cfg_.arm_descend_rad;
        msg.execute = true;
        arm_target_pub_->publish(msg);

        RCLCPP_INFO(get_logger(), "回零中（发球臂 + 垫球盘同步复位）...");
    }

    void updateReturning() {
        if (!dji_state_valid_) return;

        double remaining = angleDiffCCW(dji_angle_deg_, 0.0);

        if (remaining < cfg_.return_threshold_deg) {
            sendDjiPosition(0.0);
            RCLCPP_INFO(get_logger(), "回零完成 → IDLE");
            state_ = ServeState::IDLE;
        } else {
            sendDjiVelocity(-cfg_.return_speed_rpm);
        }
    }

    // ────────────────────────────────────────────────────────
    // DJI 电机指令辅助
    // ────────────────────────────────────────────────────────
    void publishDjiCmd(DJICmd& msg) {
        dji_cmd_pub_->publish(msg);
        if (cfg_.use_motor2) {
            DJICmd msg2 = msg;
            msg2.joint_name = cfg_.motor2_name;
            if (msg.mode == MODE_DIRECT)   { msg2.direct_output   = -msg.direct_output; }
            if (msg.mode == MODE_VELOCITY) { msg2.velocity_target = -msg.velocity_target; }
            // 位置模式副电机由 motor_control_node 内部镜像处理
            dji_cmd_pub_->publish(msg2);
        }
    }

    void sendDjiVelocity(double rpm) {
        DJICmd msg;
        msg.joint_name     = cfg_.motor1_name;
        msg.mode           = MODE_VELOCITY;
        msg.velocity_target = rpm * 2.0 * M_PI / 60.0; // RPM → rad/s
        publishDjiCmd(msg);
    }

    void sendDjiPosition(double angle_deg) {
        DJICmd msg;
        msg.joint_name    = cfg_.motor1_name;
        msg.mode          = MODE_POSITION;
        msg.position_target = angle_deg * M_PI / 180.0; // deg → rad
        publishDjiCmd(msg);
    }

    void sendDjiDirect(int current) {
        DJICmd msg;
        msg.joint_name   = cfg_.motor1_name;
        msg.mode         = MODE_DIRECT;
        msg.direct_output = static_cast<int16_t>(
            std::max(-16384, std::min(16384, current)));
        publishDjiCmd(msg);
    }

    void sendDjiStop() {
        DJICmd msg;
        msg.joint_name   = cfg_.motor1_name;
        msg.mode         = MODE_DIRECT;
        msg.direct_output = 0;
        publishDjiCmd(msg);
    }

    // ────────────────────────────────────────────────────────
    // 角度辅助函数
    // ────────────────────────────────────────────────────────

    // 顺时针（角度增加方向）从 current 到 target 的距离
    static double angleDiffCW(double current, double target) {
        return std::fmod(target - current + 360.0, 360.0);
    }
    // 逆时针（角度减小方向）从 current 到 target 的距离
    static double angleDiffCCW(double current, double target) {
        return std::fmod(current - target + 360.0, 360.0);
    }

    rclcpp::Time now() { return get_clock()->now(); }
};

// ============================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServeControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

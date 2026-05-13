/**
 * strike_node.cpp
 *
 * 两阶段击球控制节点（C++ 版）
 *
 * 功能：
 *   1. 订阅 /joy (sensor_msgs/Joy)
 *      — 4个按键（A/B/X/Y）选择击球方案
 *      — LB 触发状态机（蓄力/击打）
 *   2. 订阅视觉落点话题 /strike/landing_point (geometry_msgs/PointStamped)
 *      — 仅用于自动选方案，不触发状态机
 *   3. 订阅红外触发话题 /ir_trigger (std_msgs/Bool)
 *      — 触发状态机（与 LB 等价）
 *   4. 预置多套击球方案，每套包含独立的蓄力/击打/刹停参数
 *   5. 发布 unitree_go8010_command 控制 4 个 GO-M8010-6 电机
 *
 * 决策（选方案）：
 *   手柄 A/B/X/Y 四键 / 键盘 1/2/3/4 键 / 视觉落点 → 更新 active_profile
 *
 * 触发（状态转换）：
 *   手柄 LB / 红外 /ir_trigger → IDLE→蓄力, READY→击打
 *
 * 默认方案映射（可通过 yaml 修改）：
 *   A(0) / 键盘1 → "center"
 *   B(1) / 键盘2 → "left"
 *   X(2) / 键盘3 → "right"
 *   Y(3) / 键盘4 → "strong"（如有）
 *
 * 参数（YAML 加载）：
 *   见 strike_node_params.yaml
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include "motor_control_ros2/msg/unitree_go8010_command.hpp"
#include "motor_control_ros2/msg/unitree_go8010_state.hpp"

#include <cmath>
#include <string>
#include <map>
#include <array>
#include <atomic>
#include <mutex>
#include <chrono>

namespace motor_control {

using UnitreeCmd = motor_control_ros2::msg::UnitreeGO8010Command;
using UnitreeState = motor_control_ros2::msg::UnitreeGO8010State;
using PointStamped = geometry_msgs::msg::PointStamped;

// ── 电机索引 ────────────────────────────────────────────────
static constexpr int MOTOR_L1_ID = 0;   // 左大臂
static constexpr int MOTOR_L2_ID = 1;   // 左小臂
static constexpr int MOTOR_R1_ID = 2;   // 右大臂
static constexpr int MOTOR_R2_ID = 3;   // 右小臂

struct MotorConfig {
    int id;
    std::string device;
};

// 单个关节的 FOC 参数组
struct FocParams {
    double pos        {0.0};
    double vel        {0.0};
    double torque_ff  {0.0};
    double kp         {0.0};
    double kd         {0.0};
};

// 一个完整的击球阶段参数（大臂 + 小臂）
struct PhaseCmd {
    FocParams main;  // 大臂 (ID 0/2)
    FocParams sub;   // 小臂 (ID 1/3)
};

// 一套完整击球方案
struct StrikeProfile {
    std::string name;
    PhaseCmd    ready;          // 蓄力阶段
    PhaseCmd    strike;         // 击打加速阶段
    PhaseCmd    brake;          // 刹停阶段
    double      trigger_rad     {1.92};  // 大臂触发位置 (rad, ≈110°)
    double      strike_timeout  {3.0};   // 击打超时保护 (s)
    double      brake_duration  {1.0};   // 刹停持续时间 (s)
    double      return_time     {1.5};   // 回蓄力插值时间 (s)
};

// ── 状态机 ───────────────────────────────────────────────────
enum class StrikeState {
    IDLE,
    MOVING_TO_READY,
    READY,
    STRIKE_ACCEL,
    BRAKING,
    RETURNING,
};

// ── 节点 ─────────────────────────────────────────────────────
class StrikeNode : public rclcpp::Node {
public:
    StrikeNode() : Node("strike_node") {
        loadParams();
        setupMotors();

        cmd_pub_ = create_publisher<UnitreeCmd>("unitree_go8010_command", 10);

        state_sub_ = create_subscription<UnitreeState>(
            "unitree_go8010_states", 20,
            [this](const UnitreeState::SharedPtr msg) { stateCallback(msg); });

        joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) { joyCallback(msg); });

        vision_sub_ = create_subscription<PointStamped>(
            "/strike/landing_point", 10,
            [this](const PointStamped::SharedPtr msg) { visionCallback(msg); });

        // 红外触发接口（下位机通过桥接节点透传）
        ir_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
            ir_trigger_topic_, 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    ir_triggered_.store(true);
                    RCLCPP_INFO_ONCE(get_logger(), "红外触发接口已激活，话题: %s", ir_trigger_topic_.c_str());
                }
            });

        // 控制定时器 200 Hz
        using namespace std::chrono_literals;
        control_timer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / control_hz_)),
            [this]() { controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "strike_node 启动 | %.0f Hz | %zu 套击球方案 | 默认方案: %s",
            control_hz_, profiles_.size(), active_profile_name_.c_str());
        printProfiles();
    }

private:
    // ── 参数加载 ──────────────────────────────────────────────
    void loadParams() {
        std::string cfg_path;
        try {
            cfg_path = ament_index_cpp::get_package_share_directory("motor_control_ros2")
                       + "/config/strike_node_params.yaml";
        } catch (...) {
            RCLCPP_WARN(get_logger(), "找不到 package share 目录，使用内置默认值");
            loadDefaults();
            return;
        }

        YAML::Node root;
        try {
            root = YAML::LoadFile(cfg_path);
        } catch (const std::exception & e) {
            RCLCPP_WARN(get_logger(), "加载参数文件失败: %s，使用内置默认值", e.what());
            loadDefaults();
            return;
        }

        auto p = root["strike_node"]["ros__parameters"];

        // 基础参数
        if (p["control_hz"])             control_hz_           = p["control_hz"].as<double>();
        if (p["joy_button_lb"])          joy_button_lb_        = p["joy_button_lb"].as<int>();
        if (p["gravity_tau_inner"])      gravity_tau_inner_    = p["gravity_tau_inner"].as<double>();
        if (p["gravity_tau_outer"])      gravity_tau_outer_    = p["gravity_tau_outer"].as<double>();
        if (p["gear_ratio"])             gear_ratio_           = p["gear_ratio"].as<double>();
        if (p["gravity_offset_rad"])     gravity_offset_       = p["gravity_offset_rad"].as<double>();
        if (p["vision_threshold_right"]) vision_thresh_right_  = p["vision_threshold_right"].as<double>();
        if (p["vision_threshold_left"])  vision_thresh_left_   = p["vision_threshold_left"].as<double>();
        if (p["vision_timeout_sec"])     vision_timeout_sec_   = p["vision_timeout_sec"].as<double>();
        if (p["default_profile"])        active_profile_name_  = p["default_profile"].as<std::string>();
        if (p["ir_trigger_topic"])       ir_trigger_topic_     = p["ir_trigger_topic"].as<std::string>();
        if (p["ir_trigger_enabled"])     ir_trigger_enabled_   = p["ir_trigger_enabled"].as<bool>();

        // 手柄选方案按键映射（button index → profile name）
        if (p["joy_profile_buttons"]) {
            auto bmap = p["joy_profile_buttons"];
            for (int i = 0; i < 4; ++i) {
                std::string key = "button_" + std::to_string(i);
                if (bmap[key]) {
                    joy_profile_map_[i] = bmap[key].as<std::string>();
                }
            }
        }

        // 电机设备
        if (p["motors"]) {
            auto m = p["motors"];
            motor_l1_ = {m["l1_id"].as<int>(), m["l1_device"].as<std::string>()};
            motor_l2_ = {m["l2_id"].as<int>(), m["l2_device"].as<std::string>()};
            motor_r1_ = {m["r1_id"].as<int>(), m["r1_device"].as<std::string>()};
            motor_r2_ = {m["r2_id"].as<int>(), m["r2_device"].as<std::string>()};
        }

        // 击球方案
        if (p["profiles"]) {
            for (const auto& prof_node : p["profiles"]) {
                StrikeProfile prof;
                prof.name = prof_node["name"].as<std::string>();
                if (prof_node["trigger_deg"])    prof.trigger_rad    = deg2rad(prof_node["trigger_deg"].as<double>());
                if (prof_node["strike_timeout"]) prof.strike_timeout = prof_node["strike_timeout"].as<double>();
                if (prof_node["brake_duration"]) prof.brake_duration = prof_node["brake_duration"].as<double>();
                if (prof_node["return_time"])    prof.return_time    = prof_node["return_time"].as<double>();
                loadPhase(prof_node["ready"],  prof.ready);
                loadPhase(prof_node["strike"], prof.strike);
                loadPhase(prof_node["brake"],  prof.brake);
                profiles_[prof.name] = prof;
            }
        }

        if (profiles_.empty()) {
            RCLCPP_WARN(get_logger(), "参数文件中没有 profiles，使用内置默认值");
            loadDefaults();
        }

        if (profiles_.find(active_profile_name_) == profiles_.end()) {
            active_profile_name_ = profiles_.begin()->first;
        }
    }

    void loadPhase(const YAML::Node& n, PhaseCmd& phase) {
        if (!n) return;
        loadFoc(n["main"], phase.main);
        loadFoc(n["sub"],  phase.sub);
    }

    void loadFoc(const YAML::Node& n, FocParams& f) {
        if (!n) return;
        if (n["pos_deg"])    f.pos       = deg2rad(n["pos_deg"].as<double>());
        if (n["vel"])        f.vel       = n["vel"].as<double>();
        if (n["torque_ff"])  f.torque_ff = n["torque_ff"].as<double>();
        if (n["kp"])         f.kp        = n["kp"].as<double>();
        if (n["kd"])         f.kd        = n["kd"].as<double>();
    }

    void loadDefaults() {
        // 默认方案 "center"
        StrikeProfile center;
        center.name = "center";
        center.trigger_rad    = deg2rad(110.0);
        center.strike_timeout = 3.0;
        center.brake_duration = 1.0;
        center.return_time    = 1.5;

        center.ready.main = { deg2rad(30.0), 0.0, 0.0, 0.5, 0.1 };
        center.ready.sub  = { deg2rad(0.0),  0.0, 0.0, 0.5, 0.1 };

        center.strike.main = { 0.0, 9.0, 0.5, 0.0, 0.1 };
        center.strike.sub  = { 0.0, 7.5, 0.5, 0.0, 0.1 };

        center.brake.main = { 0.0, 0.0, 0.0, 0.0, 0.1 };
        center.brake.sub  = { 0.0, 0.0, 0.0, 0.0, 0.1 };

        profiles_["center"] = center;

        // 偏左方案
        StrikeProfile left = center;
        left.name = "left";
        left.ready.main.pos  = deg2rad(25.0);
        left.strike.main.vel = 9.5;
        left.strike.sub.vel  = 8.0;
        profiles_["left"] = left;

        // 偏右方案
        StrikeProfile right = center;
        right.name = "right";
        right.ready.main.pos  = deg2rad(35.0);
        right.strike.main.vel = 8.5;
        right.strike.sub.vel  = 7.0;
        profiles_["right"] = right;

        // 强力方案（Y 键 / 键盘4）
        StrikeProfile strong = center;
        strong.name = "strong";
        strong.strike.main.vel = 10.5;
        strong.strike.main.torque_ff = 0.8;
        strong.strike.sub.vel  = 9.0;
        strong.strike.sub.torque_ff  = 0.8;
        profiles_["strong"] = strong;

        active_profile_name_ = "center";

        // 默认按键映射: A=center, B=left, X=right, Y=strong
        joy_profile_map_ = {{0, "center"}, {1, "left"}, {2, "right"}, {3, "strong"}};
    }

    void setupMotors() {
        motor_l1_ = {MOTOR_L1_ID, "/dev/ttyUSB0"};
        motor_l2_ = {MOTOR_L2_ID, "/dev/ttyUSB0"};
        motor_r1_ = {MOTOR_R1_ID, "/dev/ttyUSB1"};
        motor_r2_ = {MOTOR_R2_ID, "/dev/ttyUSB1"};
    }

    // ── 回调 ──────────────────────────────────────────────────
    void stateCallback(const UnitreeState::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        MotorStateData s;
        s.position = msg->position;
        s.velocity = msg->velocity;
        s.torque   = msg->torque;
        s.online   = msg->online;
        motor_states_[msg->motor_id] = s;
    }

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        const auto& btns = msg->buttons;

        // ── LB（触发键）上升沿 ──────────────────────────────
        if (static_cast<int>(btns.size()) > joy_button_lb_) {
            int cur_lb = btns[joy_button_lb_];
            if (cur_lb && !lb_prev_) {
                lb_pressed_.store(true);
            }
            lb_prev_ = cur_lb;
        }

        // ── A/B/X/Y（选方案键）上升沿 ──────────────────────
        for (auto& [btn_idx, profile_name] : joy_profile_map_) {
            if (btn_idx >= static_cast<int>(btns.size())) continue;
            int cur = btns[btn_idx];
            if (cur && !joy_select_prev_[btn_idx]) {
                selectProfile(profile_name, "手柄按键");
            }
            joy_select_prev_[btn_idx] = cur;
        }
    }

    // 选方案（仅在 IDLE/READY/RETURNING 时生效，击打过程中锁定）
    void selectProfile(const std::string& name, const char* source) {
        if (profiles_.find(name) == profiles_.end()) {
            RCLCPP_WARN(get_logger(), "%s: 未知方案 '%s'", source, name.c_str());
            return;
        }
        if (state_ == StrikeState::STRIKE_ACCEL || state_ == StrikeState::BRAKING) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500,
                "%s: 击打中，方案切换将在下次蓄力生效", source);
            pending_profile_ = name;
            return;
        }
        if (active_profile_name_ != name) {
            active_profile_name_ = name;
            pending_profile_.clear();
            RCLCPP_INFO(get_logger(), "%s → 方案: [%s]", source, name.c_str());
        }
    }

    void visionCallback(const PointStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(vision_mutex_);
        landing_x_    = msg->point.x;
        vision_stamp_ = now_sec();

        // 视觉仅做决策，不触发状态机
        std::string new_profile;
        if (landing_x_ >= vision_thresh_right_) {
            new_profile = "right";
        } else if (landing_x_ <= vision_thresh_left_) {
            new_profile = "left";
        } else {
            new_profile = "center";
        }
        selectProfile(new_profile, "视觉落点");
    }

    // ── 主控制循环 (200 Hz) ───────────────────────────────────
    void controlLoop() {
        // 视觉超时降级
        checkVisionTimeout();

        switch (state_) {
            case StrikeState::IDLE:
                if (lb_pressed_.exchange(false) || checkIrTrigger()) {
                    RCLCPP_INFO(get_logger(), "触发 → 蓄力 [方案: %s]",
                                active_profile_name_.c_str());
                    startMoveToReady();
                }
                break;

            case StrikeState::MOVING_TO_READY:
                updateMoveToReady();
                break;

            case StrikeState::READY:
                holdPhase(activeProfile().ready);
                if (lb_pressed_.exchange(false) || checkIrTrigger()) {
                    RCLCPP_INFO(get_logger(), "触发 → 击打! [方案: %s]",
                                active_profile_name_.c_str());
                    startStrike();
                }
                break;

            case StrikeState::STRIKE_ACCEL:
                holdPhase(activeProfile().strike);
                checkStrikeTrigger();
                break;

            case StrikeState::BRAKING:
                holdPhase(activeProfile().brake);
                if (elapsed() >= activeProfile().brake_duration) {
                    RCLCPP_INFO(get_logger(), "刹停完成 → 回蓄力");
                    startReturn();
                }
                break;

            case StrikeState::RETURNING:
                updateReturn();
                break;
        }
    }

    void applyPendingProfile() {
        if (!pending_profile_.empty() && profiles_.count(pending_profile_)) {
            active_profile_name_ = pending_profile_;
            RCLCPP_INFO(get_logger(), "应用待定方案: [%s]", active_profile_name_.c_str());
            pending_profile_.clear();
        }
    }

    // ── 状态动作 ──────────────────────────────────────────────
    void startMoveToReady() {
        const auto& prof = activeProfile();
        move_start_main_ = avgPos(MOTOR_L1_ID, MOTOR_R1_ID);
        move_start_sub_  = avgPos(MOTOR_L2_ID, MOTOR_R2_ID);
        move_target_main_ = prof.ready.main.pos;
        move_target_sub_  = prof.ready.sub.pos;
        move_kp_ = prof.ready.main.kp;
        move_kd_ = prof.ready.main.kd;
        move_time_ = prof.return_time;
        state_ = StrikeState::MOVING_TO_READY;
        state_start_ = now_sec();
    }

    void updateMoveToReady() {
        double t = std::min(1.0, elapsed() / move_time_);
        double frac = t * t * (3.0 - 2.0 * t);  // smoothstep
        double pos_m = move_start_main_ + (move_target_main_ - move_start_main_) * frac;
        double pos_s = move_start_sub_  + (move_target_sub_  - move_start_sub_)  * frac;

        for (const auto* motor : {&motor_l1_, &motor_r1_}) {
            sendFoc(*motor, pos_m, 0.0, 0.0, move_kp_, move_kd_);
        }
        for (const auto* motor : {&motor_l2_, &motor_r2_}) {
            sendFoc(*motor, pos_s, 0.0, 0.0, move_kp_, move_kd_);
        }

        if (elapsed() >= move_time_) {
            RCLCPP_INFO(get_logger(), "蓄力位到达");
            state_ = StrikeState::READY;
            state_start_ = now_sec();
        }
    }

    void startStrike() {
        state_ = StrikeState::STRIKE_ACCEL;
        state_start_ = now_sec();
        sendPhase(activeProfile().strike);
    }

    void checkStrikeTrigger() {
        double main_pos = avgPos(MOTOR_L1_ID, MOTOR_R1_ID);
        double main_vel = avgVel(MOTOR_L1_ID, MOTOR_R1_ID);
        double sub_pos  = avgPos(MOTOR_L2_ID, MOTOR_R2_ID);

        if (main_pos >= activeProfile().trigger_rad) {
            RCLCPP_INFO(get_logger(),
                "到达击球位: 大臂=%.1f° 小臂=%.1f° vel=%.1f rad/s (%.0f°/s) t=%.3fs",
                rad2deg(main_pos), rad2deg(sub_pos), main_vel, rad2deg(main_vel), elapsed());
            state_ = StrikeState::BRAKING;
            state_start_ = now_sec();
            sendPhase(activeProfile().brake);
            return;
        }
        if (elapsed() >= activeProfile().strike_timeout) {
            RCLCPP_WARN(get_logger(), "击打超时 %.1fs", activeProfile().strike_timeout);
            state_ = StrikeState::BRAKING;
            state_start_ = now_sec();
            sendPhase(activeProfile().brake);
        }
    }

    void startReturn() {
        const auto& prof = activeProfile();
        move_start_main_ = avgPos(MOTOR_L1_ID, MOTOR_R1_ID);
        move_start_sub_  = avgPos(MOTOR_L2_ID, MOTOR_R2_ID);
        move_target_main_ = prof.ready.main.pos;
        move_target_sub_  = prof.ready.sub.pos;
        move_kp_ = prof.ready.main.kp;
        move_kd_ = prof.ready.main.kd;
        move_time_ = prof.return_time;
        state_ = StrikeState::RETURNING;
        state_start_ = now_sec();
    }

    void updateReturn() {
        double t = std::min(1.0, elapsed() / move_time_);
        double frac = t * t * (3.0 - 2.0 * t);
        double pos_m = move_start_main_ + (move_target_main_ - move_start_main_) * frac;
        double pos_s = move_start_sub_  + (move_target_sub_  - move_start_sub_)  * frac;

        for (const auto* motor : {&motor_l1_, &motor_r1_}) {
            sendFoc(*motor, pos_m, 0.0, 0.0, move_kp_, move_kd_);
        }
        for (const auto* motor : {&motor_l2_, &motor_r2_}) {
            sendFoc(*motor, pos_s, 0.0, 0.0, move_kp_, move_kd_);
        }

        if (elapsed() >= move_time_) {
            RCLCPP_INFO(get_logger(), "回蓄力完成 → IDLE，等待下次触发");
            applyPendingProfile();
            state_ = StrikeState::IDLE;
        }
    }

    // ── 红外触发检查 ──────────────────────────────────────────
    bool checkIrTrigger() {
        if (!ir_trigger_enabled_) return false;
        return ir_triggered_.exchange(false);
    }

    // ── 视觉超时检查 ──────────────────────────────────────────
    void checkVisionTimeout() {        std::lock_guard<std::mutex> lk(vision_mutex_);
        if (vision_stamp_ > 0.0 && (now_sec() - vision_stamp_) > vision_timeout_sec_) {
            if (active_profile_name_ != "center") {
                RCLCPP_WARN(get_logger(),
                    "视觉信号超时 %.1fs，降级为 center 方案", vision_timeout_sec_);
                active_profile_name_ = "center";
            }
            vision_stamp_ = -1.0;  // 已处理，避免重复打印
        }
    }

    // ── FOC 发送 ──────────────────────────────────────────────
    double gravityTorque(int motor_id) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        double theta1 = (getPos(0) + getPos(2)) / 2.0;
        double theta2 = (getPos(1) + getPos(3)) / 2.0;
        if (motor_id == MOTOR_L1_ID || motor_id == MOTOR_R1_ID) {
            return gravity_tau_inner_ * std::cos(theta1 + gravity_offset_) / gear_ratio_;
        } else {
            return -gravity_tau_outer_ * std::cos(theta1 + theta2 + gravity_offset_) / gear_ratio_;
        }
    }

    void sendFoc(const MotorConfig& motor, double pos, double vel,
                 double torque_ff, double kp, double kd) {
        double tau_g = gravityTorque(motor.id);
        UnitreeCmd msg;
        msg.id              = static_cast<uint8_t>(motor.id);
        msg.device          = motor.device;
        msg.mode            = UnitreeCmd::MODE_FOC;
        msg.position_target = pos;
        msg.velocity_target = vel;
        msg.torque_ff       = static_cast<float>(torque_ff + tau_g);
        msg.kp              = static_cast<float>(kp);
        msg.kd              = static_cast<float>(kd);
        cmd_pub_->publish(msg);
    }

    void sendBrake(const MotorConfig& motor) {
        UnitreeCmd msg;
        msg.id     = static_cast<uint8_t>(motor.id);
        msg.device = motor.device;
        msg.mode   = UnitreeCmd::MODE_BRAKE;
        cmd_pub_->publish(msg);
    }

    void sendPhase(const PhaseCmd& phase) {
        const auto& m = phase.main;
        const auto& s = phase.sub;
        for (const auto* motor : {&motor_l1_, &motor_r1_}) {
            sendFoc(*motor, m.pos, m.vel, m.torque_ff, m.kp, m.kd);
        }
        for (const auto* motor : {&motor_l2_, &motor_r2_}) {
            sendFoc(*motor, s.pos, s.vel, s.torque_ff, s.kp, s.kd);
        }
    }

    void holdPhase(const PhaseCmd& phase) {
        sendPhase(phase);
    }

public:
    void brakeAll() {
        for (const auto* motor : {&motor_l1_, &motor_l2_, &motor_r1_, &motor_r2_}) {
            sendBrake(*motor);
        }
        RCLCPP_INFO(get_logger(), "全部刹车");
    }

private:
    // ── 状态读取辅助 ──────────────────────────────────────────
    struct MotorStateData {
        float position {0.0f};
        float velocity {0.0f};
        float torque   {0.0f};
        bool  online   {false};
    };

    float getPos(int id) {
        auto it = motor_states_.find(id);
        return it != motor_states_.end() ? it->second.position : 0.0f;
    }

    float getVel(int id) {
        auto it = motor_states_.find(id);
        return it != motor_states_.end() ? it->second.velocity : 0.0f;
    }

    double avgPos(int id_a, int id_b) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        return (getPos(id_a) + getPos(id_b)) / 2.0;
    }

    double avgVel(int id_a, int id_b) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        return (getVel(id_a) + getVel(id_b)) / 2.0;
    }

    // ── 工具 ──────────────────────────────────────────────────
    static double now_sec() {
        return std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    double elapsed() const {
        return now_sec() - state_start_;
    }

    static double deg2rad(double deg) { return deg * M_PI / 180.0; }
    static double rad2deg(double rad) { return rad * 180.0 / M_PI; }

    const StrikeProfile& activeProfile() const {
        return profiles_.at(active_profile_name_);
    }

    void printProfiles() {
        for (const auto& [name, prof] : profiles_) {
            RCLCPP_INFO(get_logger(),
                "  方案[%s]: 蓄力大臂=%.1f° 击打vel=%.1f rad/s 刹停位=%.1f°",
                name.c_str(),
                rad2deg(prof.ready.main.pos),
                prof.strike.main.vel,
                rad2deg(prof.trigger_rad));
        }
    }

    // ── 成员变量 ──────────────────────────────────────────────
    // 硬件
    MotorConfig motor_l1_, motor_l2_, motor_r1_, motor_r2_;

    // 参数
    double control_hz_          {200.0};
    int    joy_button_lb_       {4};
    double gravity_tau_inner_   {8.0};
    double gravity_tau_outer_   {3.0};
    double gear_ratio_          {6.33};
    double gravity_offset_      {-M_PI / 2.0};
    double vision_thresh_right_ {0.15};
    double vision_thresh_left_  {-0.15};
    double vision_timeout_sec_  {2.0};
    std::string ir_trigger_topic_ {"/ir_trigger"};
    bool ir_trigger_enabled_    {true};

    // 击球方案
    std::map<std::string, StrikeProfile> profiles_;
    std::string active_profile_name_ {"center"};
    std::string pending_profile_;                        // 击打中途请求切换，等完成后生效
    std::map<int, std::string> joy_profile_map_;         // button_index → profile_name
    std::map<int, int> joy_select_prev_;                 // 上一帧各选方案键状态（防重复触发）

    // 状态机
    StrikeState state_ {StrikeState::IDLE};
    double state_start_ {0.0};

    // 插值缓存
    double move_start_main_  {0.0};
    double move_start_sub_   {0.0};
    double move_target_main_ {0.0};
    double move_target_sub_  {0.0};
    double move_kp_          {0.5};
    double move_kd_          {0.1};
    double move_time_        {1.5};

    // 电机状态
    std::mutex state_mutex_;
    std::map<int, MotorStateData> motor_states_;

    // 视觉
    std::mutex vision_mutex_;
    double landing_x_    {0.0};
    double vision_stamp_ {-1.0};

    // 手柄
    std::atomic<bool> lb_pressed_ {false};
    int lb_prev_ {0};
    // 红外触发
    std::atomic<bool> ir_triggered_ {false};

    // ROS
    rclcpp::Publisher<UnitreeCmd>::SharedPtr cmd_pub_;
    rclcpp::Subscription<UnitreeState>::SharedPtr state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<PointStamped>::SharedPtr vision_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ir_trigger_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::StrikeNode>();

    rclcpp::on_shutdown([&node]() {
        node->brakeAll();
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

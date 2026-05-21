#include "motor_control_ros2/teach_playback_control_node.hpp"

#include <nlohmann/json.hpp>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <cstdlib>

using json = nlohmann::json;
using namespace std::chrono_literals;

#ifndef WORKSPACE_DIR
#define WORKSPACE_DIR "/home/rosemaryrabbit/USB2CAN_motor"
#endif

namespace motor_control {

static double now_sec() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

// ── 轨迹处理：低通滤波 + 1000Hz 线性插值 ──────────────────────

// 前向-后向一阶低通滤波（零相移）
static void lowpass_filtfilt(std::vector<double>& data,
                             const std::vector<double>& t,
                             double tau) {
    int n = static_cast<int>(data.size());
    if (n < 3 || tau <= 0.0) return;

    // 前向
    std::vector<double> fwd(n);
    fwd[0] = data[0];
    for (int i = 1; i < n; ++i) {
        double dt = t[i] - t[i - 1];
        double alpha = dt / (tau + dt);
        fwd[i] = fwd[i - 1] + alpha * (data[i] - fwd[i - 1]);
    }
    // 后向
    data[n - 1] = fwd[n - 1];
    for (int i = n - 2; i >= 0; --i) {
        double dt = t[i + 1] - t[i];
        double alpha = dt / (tau + dt);
        data[i] = data[i + 1] + alpha * (fwd[i] - data[i + 1]);
    }
}

// 线性插值：从不等间距原始数据插值到等间距 1000Hz
static std::vector<double> interp_1khz(const std::vector<double>& t_raw,
                                        const std::vector<double>& y_raw,
                                        double duration) {
    int n_out = static_cast<int>(duration * 1000.0) + 1;
    std::vector<double> out(n_out);
    int n_raw = static_cast<int>(t_raw.size());
    int j = 0;

    for (int i = 0; i < n_out; ++i) {
        double t = i * 0.001;
        while (j + 1 < n_raw - 1 && t_raw[j + 1] < t) ++j;
        double dt = t_raw[j + 1] - t_raw[j];
        double frac = (dt > 0.0) ? (t - t_raw[j]) / dt : 0.0;
        frac = std::clamp(frac, 0.0, 1.0);
        out[i] = y_raw[j] + frac * (y_raw[j + 1] - y_raw[j]);
    }
    return out;
}

void TeachPlaybackControlNode::load_trajectory() {
    RCLCPP_INFO(get_logger(), "加载示教数据: %s", teach_file_.c_str());

    std::ifstream ifs(teach_file_);
    if (!ifs.is_open()) {
        throw std::runtime_error("teach file not found: " + teach_file_);
    }

    json raw = json::parse(ifs);
    json data;

    if (raw.contains("recordings")) {
        auto& recs = raw["recordings"];
        int ri = rec_index_;
        if (ri < 0) ri = static_cast<int>(recs.size()) + ri;
        ri = std::clamp(ri, 0, static_cast<int>(recs.size()) - 1);
        data = recs[ri];
        RCLCPP_INFO(get_logger(), "共 %zu 条录制, 使用第 %d 条 (%.1fs)",
                     recs.size(), ri, data.value("duration", 0.0));
    } else {
        data = raw;
    }

    if (data.contains("tau_inner") && data["tau_inner"].get<double>() != 0.0) {
        tau_inner_ = data["tau_inner"].get<double>();
    }
    if (data.contains("tau_outer") && data["tau_outer"].get<double>() != 0.0) {
        tau_outer_ = data["tau_outer"].get<double>();
    }
    RCLCPP_INFO(get_logger(), "重力补偿: 大臂=%.1fNm 小臂=%.1fNm",
                tau_inner_, tau_outer_);

    auto& jframes = data["frames"];
    int n_raw = static_cast<int>(jframes.size());
    std::vector<double> t_raw(n_raw);
    std::array<std::vector<double>, 4> pos_raw, vel_raw, tau_raw;
    for (auto& a : pos_raw) a.resize(n_raw);
    for (auto& a : vel_raw) a.resize(n_raw);
    for (auto& a : tau_raw) a.resize(n_raw);

    double t_start = jframes[0]["t"].get<double>();
    for (int i = 0; i < n_raw; ++i) {
        auto& jf = jframes[i];
        t_raw[i] = jf["t"].get<double>() - t_start;
        for (int m = 0; m < 4; ++m) {
            std::string mk = "m" + std::to_string(m);
            std::string vk = "v" + std::to_string(m);
            std::string tk = "tau" + std::to_string(m);
            pos_raw[m][i] = jf[mk].get<double>();
            vel_raw[m][i] = jf.value(vk, 0.0);
            tau_raw[m][i] = jf.value(tk, 0.0);
        }
    }

    double duration = t_raw.back();

    // 低通滤波三通道
    for (int m = 0; m < 4; ++m) {
        lowpass_filtfilt(pos_raw[m], t_raw, filter_tau_);
        lowpass_filtfilt(vel_raw[m], t_raw, filter_tau_);
        lowpass_filtfilt(tau_raw[m], t_raw, filter_tau_);
    }

    // 线性插值到 1000Hz
    traj_.duration = duration;
    traj_.n_frames = static_cast<int>(duration * 1000.0) + 1;
    for (int m = 0; m < 4; ++m) {
        traj_.pos[m] = interp_1khz(t_raw, pos_raw[m], duration);
        traj_.vel[m] = interp_1khz(t_raw, vel_raw[m], duration);
        traj_.torque[m] = interp_1khz(t_raw, tau_raw[m], duration);
    }

    for (int m = 0; m < 4; ++m) {
        start_pos_[m] = traj_.pos[m].front();
        start_vel_[m] = traj_.vel[m].front();
        start_torque_[m] = traj_.torque[m].front();
        end_pos_[m] = traj_.pos[m].back();
        end_vel_[m] = traj_.vel[m].back();
        end_torque_[m] = traj_.torque[m].back();
    }

    RCLCPP_INFO(get_logger(), "轨迹: %d帧@200Hz → 低通(τ=%.3fs) → 1000Hz(%d帧), %.2fs × %.1fx",
                n_raw, filter_tau_, traj_.n_frames, duration, speed_);
}

// ── 构造函数 ──────────────────────────────────────────────────

TeachPlaybackControlNode::TeachPlaybackControlNode()
    : Node("teach_playback_control_node"),
      state_(State::IDLE),
      trigger_pending_(false),
      phase_start_(0.0),
      loop_count_(0)
{
    this->declare_parameter("teach_file",
        std::string(WORKSPACE_DIR "/teach_data.json"));
    this->declare_parameter("rec_index", -1);
    this->declare_parameter("kp", 0.05);
    this->declare_parameter("kd", 0.15);
    this->declare_parameter("kp_hold", 0.05);
    this->declare_parameter("kd_hold", 0.1);
    this->declare_parameter("speed", 1.0);
    this->declare_parameter("filter_tau", 0.02);
    this->declare_parameter("tau_inner", 3.1);
    this->declare_parameter("tau_outer", 1.5);
    this->declare_parameter("gravity_offset_inner", -M_PI / 2.0);
    this->declare_parameter("gravity_offset_outer", -M_PI / 2.0);
    this->declare_parameter("goto_time", 5.0);
    this->declare_parameter("hold_time", 1.0);
    this->declare_parameter("trigger_source", std::string("joy"));
    this->declare_parameter("joy_button", 5);

    teach_file_ = this->get_parameter("teach_file").as_string();
    rec_index_ = this->get_parameter("rec_index").as_int();
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    kp_hold_ = this->get_parameter("kp_hold").as_double();
    kd_hold_ = this->get_parameter("kd_hold").as_double();
    speed_ = this->get_parameter("speed").as_double();
    filter_tau_ = this->get_parameter("filter_tau").as_double();
    tau_inner_ = this->get_parameter("tau_inner").as_double();
    tau_outer_ = this->get_parameter("tau_outer").as_double();
    gravity_offset_inner_ = this->get_parameter("gravity_offset_inner").as_double();
    gravity_offset_outer_ = this->get_parameter("gravity_offset_outer").as_double();
    goto_time_ = this->get_parameter("goto_time").as_double();
    hold_time_ = this->get_parameter("hold_time").as_double();
    trigger_source_ = this->get_parameter("trigger_source").as_string();
    joy_button_ = this->get_parameter("joy_button").as_int();

    initial_pos_.fill(0.0);
    start_pos_.fill(0.0);
    end_pos_.fill(0.0);
    start_vel_.fill(0.0);
    end_vel_.fill(0.0);
    start_torque_.fill(0.0);
    end_torque_.fill(0.0);

    cmd_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeGO8010Command>(
        "/unitree_go8010_command", 10);
    state_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/strike_busy", 10);
    motor_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeGO8010State>(
        "/unitree_go8010_states",
        rclcpp::QoS(10),
        std::bind(&TeachPlaybackControlNode::state_callback, this, std::placeholders::_1));

    if (trigger_source_ == "joy") {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&TeachPlaybackControlNode::joy_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "触发源: 手柄 button %d", joy_button_);
    } else {
        ir_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/ir_trigger", 10,
            std::bind(&TeachPlaybackControlNode::ir_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "触发源: 红外传感器 /ir_trigger");
    }

    RCLCPP_INFO(get_logger(), "节点创建完成 (kp=%.2f kd=%.2f kp_hold=%.2f kd_hold=%.2f)",
                kp_, kd_, kp_hold_, kd_hold_);
}

// ── 回调 ──────────────────────────────────────────────────────

void TeachPlaybackControlNode::state_callback(
    const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg)
{
    if (msg->joint_name.find("strike_motor") == std::string::npos) return;
    positions_[msg->motor_id] = msg->position;
    velocities_[msg->motor_id] = msg->velocity;
    temperatures_[msg->motor_id] = msg->temperature;
    errors_[msg->motor_id] = msg->error;
}

void TeachPlaybackControlNode::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (static_cast<int>(msg->buttons.size()) > joy_button_ &&
        msg->buttons[joy_button_] && state_ == State::IDLE) {
        trigger_pending_ = true;
        RCLCPP_INFO(get_logger(), "收到手柄触发!");
    }
}

void TeachPlaybackControlNode::ir_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && state_ == State::IDLE) {
        trigger_pending_ = true;
        RCLCPP_INFO(get_logger(), "收到红外触发!");
    }
}

// ── 控制辅助 ──────────────────────────────────────────────────

void TeachPlaybackControlNode::send_cmd(
    int id, double torque_ff, double kp, double kd,
    double pos_target, double vel_target)
{
    auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
    cmd.id = id;
    cmd.mode = 1;
    cmd.position_target = pos_target;
    cmd.velocity_target = vel_target;
    cmd.kp = kp;
    cmd.kd = kd;
    cmd.torque_ff = torque_ff;
    cmd_pub_->publish(cmd);
}

void TeachPlaybackControlNode::brake_all() {
    for (int mid : ALL_IDS) {
        auto cmd = motor_control_ros2::msg::UnitreeGO8010Command();
        cmd.id = mid;
        cmd.mode = 0;
        cmd_pub_->publish(cmd);
    }
}

double TeachPlaybackControlNode::smoothstep(double t) {
    t = std::clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}

double TeachPlaybackControlNode::gravity_torque(int mid, double theta1, double theta2) {
    if (is_inner(mid)) {
        return tau_inner_ * std::cos(theta1 + gravity_offset_inner_) / GEAR_RATIO;
    } else {
        return -tau_outer_ * std::cos(theta1 + theta2 + gravity_offset_outer_) / GEAR_RATIO;
    }
}

std::pair<double, double> TeachPlaybackControlNode::get_theta_pair() {
    double t1 = (positions_.count(0) ? positions_[0] : 0.0) +
                (positions_.count(2) ? positions_[2] : 0.0);
    double t2 = (positions_.count(1) ? positions_[1] : 0.0) +
                (positions_.count(3) ? positions_[3] : 0.0);
    return {t1 / 2.0, t2 / 2.0};
}

bool TeachPlaybackControlNode::check_safety() {
    for (int mid : ALL_IDS) {
        auto it = temperatures_.find(mid);
        if (it != temperatures_.end() && it->second >= TEMP_CRITICAL) {
            RCLCPP_ERROR(get_logger(), "M%d 温度=%.0fC 超限!", mid, it->second);
            return false;
        }
        auto eit = errors_.find(mid);
        if (eit != errors_.end() && eit->second >= 1 && eit->second <= 4) {
            RCLCPP_ERROR(get_logger(), "M%d 错误码=%d", mid, eit->second);
            return false;
        }
    }
    return true;
}

bool TeachPlaybackControlNode::motors_online() {
    return static_cast<int>(positions_.size()) >= 4;
}

// ── 启动 & 控制循环 ──────────────────────────────────────────

void TeachPlaybackControlNode::start() {
    load_trajectory();

    state_ = State::IDLE;
    loop_count_ = 0;
    RCLCPP_INFO(get_logger(), "进入 IDLE, 等待触发...");

    // 1000Hz 控制循环—匹配预插值轨迹帧率
    control_timer_ = this->create_wall_timer(
        std::chrono::microseconds(1000),  // 1ms = 1000Hz
        std::bind(&TeachPlaybackControlNode::control_loop, this));
}

void TeachPlaybackControlNode::control_loop() {
    if (!motors_online()) {
        if (loop_count_ % 1000 == 0) {
            RCLCPP_WARN(get_logger(), "等待电机上线...");
        }
        loop_count_++;
        return;
    }

    if (!check_safety()) {
        brake_all();
        if (state_ != State::IDLE) {
            RCLCPP_ERROR(get_logger(), "安全检查失败, 刹车!");
        }
        state_ = State::IDLE;
    }

    auto [theta1, theta2] = get_theta_pair();

    // ── IDLE ──
    if (state_ == State::IDLE) {
        {
            auto msg = std_msgs::msg::Bool();
            msg.data = false;
            state_pub_->publish(msg);
        }

        for (int mid : ALL_IDS) {
            double tau_g = gravity_torque(mid, theta1, theta2);
            send_cmd(mid, tau_g, kp_hold_, kd_hold_, 0.0);
        }

        if (trigger_pending_) {
            trigger_pending_ = false;
            phase_start_ = now_sec();
            for (int m = 0; m < 4; ++m) {
                initial_pos_[m] = positions_.count(m) ? positions_[m] : 0.0;
            }
            {
                auto msg = std_msgs::msg::Bool();
                msg.data = true;
                state_pub_->publish(msg);
            }

            double max_diff = 0.0;
            for (int m = 0; m < 4; ++m) {
                max_diff = std::max(max_diff,
                    std::abs(initial_pos_[m] - start_pos_[m]));
            }
            if (max_diff < 0.05) {
                state_ = State::PLAYBACK;
                phase_start_ = now_sec();
                RCLCPP_INFO(get_logger(), "触发! 已在起点 → PLAYBACK");
            } else {
                state_ = State::GOTO_START;
                RCLCPP_INFO(get_logger(), "触发! → GOTO_START (偏差 %.1f°)",
                            max_diff * 180.0 / M_PI);
            }
        }
    }
    // ── GOTO_START ──
    else if (state_ == State::GOTO_START) {
        double elapsed = now_sec() - phase_start_;
        double frac = (goto_time_ > 0) ? smoothstep(elapsed / goto_time_) : 1.0;

        for (int mid : ALL_IDS) {
            double p0 = initial_pos_[mid];
            double pf = start_pos_[mid];
            double p_des = p0 + (pf - p0) * frac;
            double tau_g = gravity_torque(mid, theta1, theta2);
            send_cmd(mid, tau_g, kp_hold_, kd_hold_, p_des);
        }

        if (elapsed >= goto_time_) {
            state_ = State::PLAYBACK;
            phase_start_ = now_sec();
            RCLCPP_INFO(get_logger(), "到达起点 → PLAYBACK");
        }
    }
    // ── PLAYBACK ──
    else if (state_ == State::PLAYBACK) {
        double elapsed = now_sec() - phase_start_;
        double t_traj = elapsed * speed_;

        if (t_traj >= traj_.duration) {
            state_ = State::HOLD;
            phase_start_ = now_sec();
            RCLCPP_INFO(get_logger(), "回放完成 → HOLD");
        } else {
            int idx = static_cast<int>(t_traj * 1000.0);
            idx = std::clamp(idx, 0, traj_.n_frames - 1);

            for (int mid : ALL_IDS) {
                double tau_g = gravity_torque(mid, theta1, theta2);
                double tau_ff = tau_g + traj_.torque[mid][idx];
                send_cmd(mid, tau_ff, kp_, kd_,
                         traj_.pos[mid][idx],
                         traj_.vel[mid][idx] * speed_);
            }
        }
    }
    // ── HOLD ──
    else if (state_ == State::HOLD) {
        double elapsed = now_sec() - phase_start_;

        for (int mid : ALL_IDS) {
            double tau_g = gravity_torque(mid, theta1, theta2);
            send_cmd(mid, tau_g, kp_hold_, kd_hold_, end_pos_[mid]);
        }

        if (elapsed >= hold_time_) {
            state_ = State::RETURN_ZERO;
            phase_start_ = now_sec();
            RCLCPP_INFO(get_logger(), "保持结束 → RETURN (反向回放)");
        }
    }
    // ── RETURN (反向回放) ──
    else if (state_ == State::RETURN_ZERO) {
        double elapsed = now_sec() - phase_start_;
        double t_reverse = traj_.duration - elapsed * speed_;

        if (t_reverse <= 0.0) {
            state_ = State::IDLE;
            RCLCPP_INFO(get_logger(), "倒放完成 → IDLE");
        } else {
            int idx = static_cast<int>(t_reverse * 1000.0);
            idx = std::clamp(idx, 0, traj_.n_frames - 1);

            for (int mid : ALL_IDS) {
                double tau_g = gravity_torque(mid, theta1, theta2);
                double tau_ff = tau_g + traj_.torque[mid][idx];
                send_cmd(mid, tau_ff, kp_, kd_,
                         traj_.pos[mid][idx],
                         -traj_.vel[mid][idx] * speed_);
            }
        }
    }

    // 状态汇报 (每 2 秒)
    loop_count_++;
    if (loop_count_ % 2000 == 0) {
        const char* state_str = "?";
        switch (state_) {
            case State::IDLE:        state_str = "IDLE"; break;
            case State::GOTO_START:  state_str = "GOTO_START"; break;
            case State::PLAYBACK:    state_str = "PLAYBACK"; break;
            case State::HOLD:        state_str = "HOLD"; break;
            case State::RETURN_ZERO: state_str = "RETURN"; break;
        }
        RCLCPP_INFO(get_logger(), "[%s] M0:%+.1f° M1:%+.1f° M2:%+.1f° M3:%+.1f°",
                    state_str,
                    positions_.count(0) ? positions_[0] * 180.0 / M_PI : 0.0,
                    positions_.count(1) ? positions_[1] * 180.0 / M_PI : 0.0,
                    positions_.count(2) ? positions_[2] * 180.0 / M_PI : 0.0,
                    positions_.count(3) ? positions_[3] * 180.0 / M_PI : 0.0);
    }
}

}  // namespace motor_control

// ── main ──────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::TeachPlaybackControlNode>();

    try {
        node->start();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "异常退出: %s", e.what());
    }

    node->brake_all();
    rclcpp::shutdown();
    return 0;
}

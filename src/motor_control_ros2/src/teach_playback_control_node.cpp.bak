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

// ── ProcessedTrajectory ────────────────────────────────────────

void ProcessedTrajectory::get_state(
    double time, std::array<double, 4>& out_pos,
    std::array<double, 4>& out_vel) const
{
    time = std::clamp(time, 0.0, duration);

    // 二分搜索找到 time 所在区间
    auto it = std::lower_bound(t.begin(), t.end(), time);
    int idx = static_cast<int>(std::distance(t.begin(), it));
    int seg = idx - 1;
    if (seg < 0) seg = 0;
    if (seg >= n_frames - 1) seg = n_frames - 2;

    double dt = t[seg + 1] - t[seg];
    double frac = (dt > 0.0) ? (time - t[seg]) / dt : 0.0;
    frac = std::clamp(frac, 0.0, 1.0);

    for (int m = 0; m < 4; ++m) {
        out_pos[m] = pos[m][seg] + frac * (pos[m][seg + 1] - pos[m][seg]);
        out_vel[m] = vel[m][seg] + frac * (vel[m][seg + 1] - vel[m][seg]);
    }
}

// ── 轨迹处理 ──────────────────────────────────────────────────

ProcessedTrajectory TeachPlaybackControlNode::process_frames(
    const std::vector<TeachFrame>& frames,
    double filter_tau, double spike_thresh)
{
    int n_raw = static_cast<int>(frames.size());
    std::vector<double> raw_t(n_raw);
    std::array<std::vector<double>, 4> raw_pos;

    for (auto& rp : raw_pos) rp.resize(n_raw);
    for (int i = 0; i < n_raw; ++i) {
        raw_t[i] = frames[i].t;
        for (int m = 0; m < 4; ++m) {
            raw_pos[m][i] = frames[i].pos[m];
        }
    }

    double t_start = raw_t.front();
    double duration = raw_t.back() - t_start;

    // 毛刺去除
    for (int m = 0; m < 4; ++m) {
        std::vector<bool> spike(n_raw, false);
        for (int i = 1; i < n_raw; ++i) {
            double dt = raw_t[i] - raw_t[i - 1];
            if (dt <= 0.0) { spike[i] = true; continue; }
            if (std::abs(raw_pos[m][i] - raw_pos[m][i - 1]) / dt > spike_thresh) {
                spike[i] = true;
            }
        }
        // 线性插值替换
        std::vector<int> good;
        for (int i = 0; i < n_raw; ++i) {
            if (!spike[i]) good.push_back(i);
        }
        if (good.size() >= 2) {
            std::vector<double> patched(n_raw);
            int gi = 0;
            for (int i = 0; i < n_raw; ++i) {
                while (gi + 1 < (int)good.size() && raw_t[good[gi + 1]] < raw_t[i]) ++gi;
                int a = good[gi], b = good[std::min(gi + 1, (int)good.size() - 1)];
                if (a == b) {
                    patched[i] = raw_pos[m][a];
                } else {
                    double frac = (raw_t[i] - raw_t[a]) / (raw_t[b] - raw_t[a]);
                    patched[i] = raw_pos[m][a] + frac * (raw_pos[m][b] - raw_pos[m][a]);
                }
            }
            raw_pos[m] = patched;
        }
    }

    // 去重复时间戳 (保留最后一个)
    std::vector<int> keep;
    for (int i = 0; i < n_raw; ++i) {
        if (i + 1 < n_raw && raw_t[i + 1] <= raw_t[i]) continue;
        keep.push_back(i);
    }

    int n = static_cast<int>(keep.size());
    ProcessedTrajectory traj;
    traj.duration = duration;
    traj.n_frames = n;
    traj.t.resize(n);
    for (int m = 0; m < 4; ++m) traj.pos[m].resize(n);

    for (int i = 0; i < n; ++i) {
        traj.t[i] = raw_t[keep[i]] - t_start;  // 归零
        for (int m = 0; m < 4; ++m) {
            traj.pos[m][i] = raw_pos[m][keep[i]];
        }
    }

    // ── 一阶低通滤波 (前向-后向，零相移) ──────────────────────
    // filter_tau: 滤波时间常数 (秒)。越大越平滑。0 = 不滤波。
    if (filter_tau > 0.0 && n > 2) {
        for (int m = 0; m < 4; ++m) {
            std::vector<double>& y = traj.pos[m];
            // 前向滤波
            std::vector<double> fwd(n);
            fwd[0] = y[0];
            for (int i = 1; i < n; ++i) {
                double dt = traj.t[i] - traj.t[i - 1];
                double alpha = dt / (filter_tau + dt);
                fwd[i] = fwd[i - 1] + alpha * (y[i] - fwd[i - 1]);
            }
            // 后向滤波
            std::vector<double> bwd(n);
            bwd[n - 1] = fwd[n - 1];
            for (int i = n - 2; i >= 0; --i) {
                double dt = traj.t[i + 1] - traj.t[i];
                double alpha = dt / (filter_tau + dt);
                bwd[i] = bwd[i + 1] + alpha * (fwd[i] - bwd[i + 1]);
            }
            y = bwd;
        }
    }

    // ── 中心差分计算速度（用于前馈）──────────────────────────────
    for (int m = 0; m < 4; ++m) {
        traj.vel[m].resize(n, 0.0);
        const auto& y = traj.pos[m];
        for (int i = 1; i < n - 1; ++i) {
            double dt2 = traj.t[i + 1] - traj.t[i - 1];
            traj.vel[m][i] = (dt2 > 0.0) ? (y[i + 1] - y[i - 1]) / dt2 : 0.0;
        }
        if (n >= 2) {
            double dt0 = traj.t[1] - traj.t[0];
            traj.vel[m][0] = (dt0 > 0.0) ? (y[1] - y[0]) / dt0 : 0.0;
            double dtn = traj.t[n - 1] - traj.t[n - 2];
            traj.vel[m][n - 1] = (dtn > 0.0) ? (y[n - 1] - y[n - 2]) / dtn : 0.0;
        }
    }

    RCLCPP_INFO(get_logger(), "轨迹处理: %d帧(去毛刺后%d点), %.2fs, 低通滤波(τ=%.3fs)",
                n_raw, n, duration, filter_tau);
    return traj;
}

// ── JSON 加载 ─────────────────────────────────────────────────

void TeachPlaybackControlNode::load_trajectory() {
    RCLCPP_INFO(get_logger(), "加载示教数据: %s", teach_file_.c_str());

    std::ifstream ifs(teach_file_);
    if (!ifs.is_open()) {
        RCLCPP_ERROR(get_logger(), "无法打开文件: %s", teach_file_.c_str());
        throw std::runtime_error("teach file not found");
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

    // tau 优先从 JSON 读取
    if (data.contains("tau_inner") && data["tau_inner"].get<double>() != 0.0) {
        tau_inner_ = data["tau_inner"].get<double>();
    }
    if (data.contains("tau_outer") && data["tau_outer"].get<double>() != 0.0) {
        tau_outer_ = data["tau_outer"].get<double>();
    }
    RCLCPP_INFO(get_logger(), "重力补偿: 大臂=%.1fNm 小臂=%.1fNm",
                tau_inner_, tau_outer_);

    // 解析帧数据
    auto& jframes = data["frames"];
    std::vector<TeachFrame> frames;
    frames.reserve(jframes.size());
    for (auto& jf : jframes) {
        TeachFrame f;
        f.t = jf["t"].get<double>();
        f.pos = {jf["m0"].get<double>(), jf["m1"].get<double>(),
                 jf["m2"].get<double>(), jf["m3"].get<double>()};
        f.vel = {jf.value("v0", 0.0), jf.value("v1", 0.0),
                 jf.value("v2", 0.0), jf.value("v3", 0.0)};
        frames.push_back(f);
    }

    traj_ = process_frames(frames, filter_tau_, spike_thresh_);

    // 缓存起点/终点
    traj_.get_state(0.0, start_pos_, end_pos_);  // 临时存 end
    std::array<double, 4> tmp_vel;
    traj_.get_state(traj_.duration, end_pos_, tmp_vel);
    playback_duration_ = traj_.duration / speed_;

    RCLCPP_INFO(get_logger(), "轨迹: %.2fs × %.1fx = %.2fs",
                traj_.duration, speed_, playback_duration_);
}

// ── 零位校准 ──────────────────────────────────────────────────

bool TeachPlaybackControlNode::calibrate_zero() {
    RCLCPP_INFO(get_logger(), "执行零位校准 (调用校准脚本)...");

    std::string script = std::string(WORKSPACE_DIR) +
        "/src/ROS_test/calibrate_zero_position.py";

    // 检查脚本是否存在
    std::ifstream check(script);
    if (!check.is_open()) {
        RCLCPP_WARN(get_logger(), "校准脚本不存在: %s，跳过校准", script.c_str());
        return false;
    }
    check.close();

    std::string cmd = "cd " + std::string(WORKSPACE_DIR) +
        "/src/ROS_test && python3 calibrate_zero_position.py";
    int ret = std::system(cmd.c_str());
    if (ret != 0) {
        RCLCPP_WARN(get_logger(), "零位校准返回 %d", ret);
        return false;
    }
    RCLCPP_INFO(get_logger(), "零位校准完成");
    return true;
}

// ── 构造函数 ──────────────────────────────────────────────────

TeachPlaybackControlNode::TeachPlaybackControlNode()
    : Node("teach_playback_control_node"),
      state_(State::IDLE),
      trigger_pending_(false),
      phase_start_(0.0),
      playback_duration_(0.0)
{
    // 声明参数
    this->declare_parameter("teach_file",
        std::string(WORKSPACE_DIR "/teach_data.json"));
    this->declare_parameter("rec_index", -1);
    this->declare_parameter("kp", 0.05);
    this->declare_parameter("kd", 0.15);
    this->declare_parameter("kp_hold", 0.3);
    this->declare_parameter("kd_hold", 0.05);
    this->declare_parameter("speed", 1.5);
    this->declare_parameter("filter_tau", 0.02);
    this->declare_parameter("spike_thresh", 20.0);
    this->declare_parameter("tau_inner", 3.1);
    this->declare_parameter("tau_outer", 1.5);
    this->declare_parameter("gravity_offset_inner", -M_PI / 2.0);
    this->declare_parameter("gravity_offset_outer", -M_PI / 2.0);
    this->declare_parameter("goto_time", 5.0);
    this->declare_parameter("hold_time", 0.5);
    this->declare_parameter("rate_hz", 200);
    this->declare_parameter("trigger_source", std::string("joy"));
    this->declare_parameter("joy_button", 5);
    this->declare_parameter("sym_diff_limit", 15.0);  // 对称角度差阈值(度)

    // 读取参数
    teach_file_ = this->get_parameter("teach_file").as_string();
    rec_index_ = this->get_parameter("rec_index").as_int();
    kp_ = this->get_parameter("kp").as_double();
    kd_ = this->get_parameter("kd").as_double();
    kp_hold_ = this->get_parameter("kp_hold").as_double();
    kd_hold_ = this->get_parameter("kd_hold").as_double();
    speed_ = this->get_parameter("speed").as_double();
    filter_tau_ = this->get_parameter("filter_tau").as_double();
    spike_thresh_ = this->get_parameter("spike_thresh").as_double();
    tau_inner_ = this->get_parameter("tau_inner").as_double();
    tau_outer_ = this->get_parameter("tau_outer").as_double();
    gravity_offset_inner_ = this->get_parameter("gravity_offset_inner").as_double();
    gravity_offset_outer_ = this->get_parameter("gravity_offset_outer").as_double();
    goto_time_ = this->get_parameter("goto_time").as_double();
    hold_time_ = this->get_parameter("hold_time").as_double();
    rate_hz_ = this->get_parameter("rate_hz").as_int();
    trigger_source_ = this->get_parameter("trigger_source").as_string();
    joy_button_ = this->get_parameter("joy_button").as_int();
    sym_diff_limit_ = this->get_parameter("sym_diff_limit").as_double();

    initial_pos_.fill(0.0);
    start_pos_.fill(0.0);
    end_pos_.fill(0.0);

    // ROS 接口
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

    RCLCPP_INFO(get_logger(), "节点创建完成");
}

// ── 回调 ──────────────────────────────────────────────────────

void TeachPlaybackControlNode::state_callback(
    const motor_control_ros2::msg::UnitreeGO8010State::SharedPtr msg)
{
    // 只接受击球臂电机，忽略其他节点（如 delta_arm_manager）的数据
    if (msg->joint_name.find("strike_motor") == std::string::npos) {
        return;
    }
    positions_[msg->motor_id] = msg->position;
    velocities_[msg->motor_id] = msg->velocity;
    temperatures_[msg->motor_id] = msg->temperature;
    errors_[msg->motor_id] = msg->error;
    last_update_[msg->motor_id] =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
}

void TeachPlaybackControlNode::joy_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (static_cast<int>(msg->buttons.size()) > joy_button_ &&
        msg->buttons[joy_button_]) {
        if (state_ == State::IDLE) {
            trigger_pending_ = true;
            RCLCPP_INFO(get_logger(), "收到手柄触发信号 → 触发!");
        } else {
            RCLCPP_WARN(get_logger(), "收到 RB 按下, 但当前状态非 IDLE, 忽略");
        }
    }
}

void TeachPlaybackControlNode::ir_callback(
    const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data && state_ == State::IDLE) {
        trigger_pending_ = true;
        RCLCPP_INFO(get_logger(), "收到红外触发信号");
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

    // 对称电机角度差检测 (仅在非 IDLE 状态检查)
    if (state_ != State::IDLE) {
        double max_diff_rad = sym_diff_limit_ * M_PI / 180.0;
        double t_now = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        auto p0 = positions_.find(0), p2 = positions_.find(2);
        if (p0 != positions_.end() && p2 != positions_.end()) {
            double diff = std::abs(p0->second - p2->second);
            if (diff > max_diff_rad) {
                RCLCPP_ERROR(get_logger(),
                    "大臂不对称! M0=%.1f°(%.3fs前) M2=%.1f°(%.3fs前) 差=%.1f° > %.0f°",
                    p0->second * 180.0 / M_PI,
                    t_now - (last_update_.count(0) ? last_update_[0] : 0.0),
                    p2->second * 180.0 / M_PI,
                    t_now - (last_update_.count(2) ? last_update_[2] : 0.0),
                    diff * 180.0 / M_PI, sym_diff_limit_);
                return false;
            }
        }
        auto p1 = positions_.find(1), p3 = positions_.find(3);
        if (p1 != positions_.end() && p3 != positions_.end()) {
            double diff = std::abs(p1->second - p3->second);
            if (diff > max_diff_rad) {
                RCLCPP_ERROR(get_logger(),
                    "小臂不对称! M1=%.1f°(%.3fs前) M3=%.1f°(%.3fs前) 差=%.1f° > %.0f°",
                    p1->second * 180.0 / M_PI,
                    t_now - (last_update_.count(1) ? last_update_[1] : 0.0),
                    p3->second * 180.0 / M_PI,
                    t_now - (last_update_.count(3) ? last_update_[3] : 0.0),
                    diff * 180.0 / M_PI, sym_diff_limit_);
                return false;
            }
        }
    }

    return true;
}

bool TeachPlaybackControlNode::motors_online() {
    return static_cast<int>(positions_.size()) >= 4;
}

// ── 主循环 ────────────────────────────────────────────────────

static double now_sec() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

void TeachPlaybackControlNode::start() {
    // 加载轨迹
    load_trajectory();

    state_ = State::IDLE;
    loop_count_ = 0;
    RCLCPP_INFO(get_logger(), "进入 IDLE, 等待触发 (零位校准已跳过, 请手动校准)...");

    // 创建 wall_timer，由 executor 统一调度回调和订阅
    double dt = 1.0 / rate_hz_;
    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt),
        std::bind(&TeachPlaybackControlNode::control_loop, this));
}

void TeachPlaybackControlNode::control_loop() {
    if (!motors_online()) {
        if (loop_count_ % rate_hz_ == 0) {
            RCLCPP_WARN(get_logger(), "等待电机上线...");
        }
        loop_count_++;
        return;
    }

    // 安全检查 (每帧，200Hz)
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

            // 起点接近当前位置 → 跳过 GOTO_START 直接回放
            double max_diff = 0.0;
            for (int m = 0; m < 4; ++m) {
                max_diff = std::max(max_diff,
                    std::abs(initial_pos_[m] - start_pos_[m]));
            }
            if (max_diff < 0.05) {  // < 3° 视为已在起点
                state_ = State::PLAYBACK;
                RCLCPP_INFO(get_logger(), "触发! 起点近零 → 直接 PLAYBACK");
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
            send_cmd(mid, tau_g, kp_, kd_, p_des);
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
            std::array<double, 4> pos_d, vel_d;
            traj_.get_state(t_traj, pos_d, vel_d);
            for (int mid : ALL_IDS) {
                double tau_g = gravity_torque(mid, theta1, theta2);
                send_cmd(mid, tau_g, kp_, kd_,
                         pos_d[mid], vel_d[mid] * speed_);
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
    // ── RETURN (反向回放录制轨迹) ──
    else if (state_ == State::RETURN_ZERO) {
        double elapsed = now_sec() - phase_start_;
        double t_reverse = traj_.duration - elapsed * speed_;

        if (t_reverse <= 0.0) {
            // 倒放完成，已回到轨迹起点
            state_ = State::IDLE;
            RCLCPP_INFO(get_logger(), "倒放完成 → IDLE, 等待下次触发");
        } else {
            std::array<double, 4> pos_d, vel_d;
            traj_.get_state(t_reverse, pos_d, vel_d);
            for (int mid : ALL_IDS) {
                double tau_g = gravity_torque(mid, theta1, theta2);
                // 反向回放：位置正常，速度取反
                send_cmd(mid, tau_g, kp_, kd_,
                         pos_d[mid], -vel_d[mid] * speed_);
            }
        }
    }

    // 状态汇报 (每 2 秒)
    loop_count_++;
    if (loop_count_ % (rate_hz_ * 2) == 0) {
        const char* state_str = "?";
        switch (state_) {
            case State::CALIBRATING: state_str = "CALIBRATING"; break;
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
        node->start();          // 非阻塞：加载轨迹 + 启动 200Hz timer
        rclcpp::spin(node);     // executor 统一调度订阅回调和控制 timer
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "异常退出: %s", e.what());
    }

    node->brake_all();
    rclcpp::shutdown();
    return 0;
}

#include "motor_control_ros2/arm_control_node.hpp"
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <algorithm>

namespace motor_control {

ArmControlNode::ArmControlNode() 
    : Node("parallel_arm_control_node"),
      command_timeout_(0.5),
      max_coordination_error_(0.0),
      control_loop_count_(0),
      control_mode_(0) {
    
    RCLCPP_INFO(this->get_logger(), "正在初始化双臂并联击球系统控制节点...");
    
    // 初始化状态变量
    current_motor_positions_.fill(0.0);
    current_motor_velocities_.fill(0.0);
    current_motor_torques_.fill(0.0);
    current_temperatures_.fill(0);
    motors_online_.fill(false);
    
    target_x_ = 0.4;
    target_y_ = 0.2;
    target_orientation_ = 0.0;
    target_vx_ = 0.0;
    target_vy_ = 0.0;
    
    target_motor_positions_.fill(0.0);
    target_motor_velocities_.fill(0.0);
    target_motor_torques_.fill(0.0);
    control_kp_.fill(60.0);
    control_kd_.fill(6.0);
    
    // 加载配置文件
    try {
        std::string config_file = ament_index_cpp::get_package_share_directory("motor_control_ros2") + 
                                 "/config/strike_system_params.yaml";
        RCLCPP_INFO(this->get_logger(), "正在加载配置文件: %s", config_file.c_str());
        loadParallelArmParams(config_file);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "配置文件加载失败: %s", e.what());
        throw;
    }
    
    // 初始化时间戳
    last_command_time_ = this->now();
    last_state_pub_time_ = this->now();
    
    // 创建订阅者
    strike_cmd_sub_ = this->create_subscription<motor_control_ros2::msg::ArmJointCommand>(
        "/strike_command", 10,
        std::bind(&ArmControlNode::strikeCommandCallback, this, std::placeholders::_1)
    );
    
    motor_state_sub_ = this->create_subscription<motor_control_ros2::msg::UnitreeMotorState>(
        "/unitree_motor_states", 10,
        std::bind(&ArmControlNode::motorStateCallback, this, std::placeholders::_1)
    );
    
    // 创建发布者
    motor_cmd_pub_ = this->create_publisher<motor_control_ros2::msg::UnitreeMotorCommand>(
        "/unitree_motor_command", 10
    );
    
    strike_state_pub_ = this->create_publisher<motor_control_ros2::msg::ArmState>(
        "/strike_state", 10
    );
    
    // 创建控制循环定时器
    auto period = std::chrono::duration<double>(1.0 / params_.control_frequency);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&ArmControlNode::controlLoop, this)
    );
    
    RCLCPP_INFO(this->get_logger(), 
        "双臂并联控制节点启动成功 - 控制频率: %.1f Hz", params_.control_frequency);
    RCLCPP_INFO(this->get_logger(), 
        "电机配置 [左主, 左副, 右主, 右副]: [%s, %s, %s, %s]",
        motor_names_[0].c_str(), motor_names_[1].c_str(),
        motor_names_[2].c_str(), motor_names_[3].c_str());
    RCLCPP_INFO(this->get_logger(),
        "几何参数 - 臂间距: %.3fm, 大臂: %.3fm, 小臂: %.3fm, 连杆: %.3fm",
        params_.arm_spacing, params_.upper_arm_length,
        params_.forearm_length, params_.linkage_length);
}

void ArmControlNode::loadParallelArmParams(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);
    
    // 加载几何参数
    auto geom = config["geometry"];
    params_.arm_spacing = geom["arm_spacing"].as<double>();
    params_.upper_arm_length = geom["upper_arm_length"].as<double>();
    params_.forearm_length = geom["forearm_length"].as<double>();
    params_.linkage_length = geom["linkage_length"].as<double>();
    params_.linkage_attach_upper = geom["linkage_attach_on_upper"].as<double>();
    params_.linkage_attach_forearm = geom["linkage_attach_on_forearm"].as<double>();
    params_.plate_width = geom["plate_width"].as<double>();
    params_.plate_height = geom["plate_height"].as<double>();
    
    // 加载工作空间参数
    auto ws = config["workspace"];
    params_.workspace_x_min = ws["x_min"].as<double>();
    params_.workspace_x_max = ws["x_max"].as<double>();
    params_.workspace_y_min = ws["y_min"].as<double>();
    params_.workspace_y_max = ws["y_max"].as<double>();
    params_.max_orientation = ws["orientation_max"].as<double>() * M_PI / 180.0;
    
    // 加载动力学参数
    auto dyn = config["dynamics"];
    params_.max_velocity = dyn["max_velocity"].as<double>();
    params_.max_acceleration = dyn["max_acceleration"].as<double>();
    params_.max_motor_velocity = dyn["max_motor_velocity"].as<double>();
    params_.max_motor_torque = dyn["max_motor_torque"].as<double>();
    
    // 加载控制参数
    auto ctrl = config["control"];
    params_.control_frequency = ctrl["frequency"].as<double>();
    command_timeout_ = ctrl["command_timeout"].as<double>();
    params_.coordination_threshold = ctrl["coordination_threshold"].as<double>();
    
    auto kp_list = ctrl["default_kp"];
    auto kd_list = ctrl["default_kd"];
    for (size_t i = 0; i < 4; ++i) {
        control_kp_[i] = kp_list[i].as<double>();
        control_kd_[i] = kd_list[i].as<double>();
    }
    
    // 加载电机映射
    auto motors = config["motor_mapping"];
    motor_names_[0] = motors["left_main_motor"].as<std::string>();
    motor_names_[1] = motors["left_sub_motor"].as<std::string>();
    motor_names_[2] = motors["right_main_motor"].as<std::string>();
    motor_names_[3] = motors["right_sub_motor"].as<std::string>();
    
    RCLCPP_INFO(this->get_logger(), 
        "配置加载完成 - 工作空间: X[%.2f, %.2f]m, Y[%.2f, %.2f]m",
        params_.workspace_x_min, params_.workspace_x_max,
        params_.workspace_y_min, params_.workspace_y_max);
}

void ArmControlNode::strikeCommandCallback(
    const motor_control_ros2::msg::ArmJointCommand::SharedPtr msg) {
    
    last_command_time_ = this->now();
    control_mode_ = msg->control_mode;
    
    if (control_mode_ == 0) {
        // 任务空间控制
        target_x_ = msg->target_position.x;
        target_y_ = msg->target_position.y;
        target_orientation_ = msg->target_orientation;
        target_vx_ = msg->target_velocity.x;
        target_vy_ = msg->target_velocity.y;
        
        // 工作空间限位
        target_x_ = std::clamp(target_x_, params_.workspace_x_min, params_.workspace_x_max);
        target_y_ = std::clamp(target_y_, params_.workspace_y_min, params_.workspace_y_max);
        target_orientation_ = std::clamp(target_orientation_, -params_.max_orientation, 
                                        params_.max_orientation);
        
        // 逆运动学求解
        std::array<double, 4> joint_solution;
        if (parallelIK(target_x_, target_y_, target_orientation_, joint_solution)) {
            target_motor_positions_ = joint_solution;
            RCLCPP_DEBUG(this->get_logger(), "逆运动学成功: 目标=(%.3f, %.3f, %.2f°)",
                target_x_, target_y_, target_orientation_ * 180.0 / M_PI);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "逆运动学无解，保持当前目标");
        }
        
    } else {
        // 关节空间控制
        target_motor_positions_[0] = msg->left_main_motor;
        target_motor_positions_[1] = msg->left_sub_motor;
        target_motor_positions_[2] = msg->right_main_motor;
        target_motor_positions_[3] = msg->right_sub_motor;
    }
    
    // 更新控制参数
    for (size_t i = 0; i < 4; ++i) {
        control_kp_[i] = msg->kp[i];
        control_kd_[i] = msg->kd[i];
        target_motor_torques_[i] = msg->torque_ff[i];
    }
}

void ArmControlNode::motorStateCallback(
    const motor_control_ros2::msg::UnitreeMotorState::SharedPtr msg) {
    
    // 查找电机索引
    for (size_t i = 0; i < 4; ++i) {
        if (msg->joint_name == motor_names_[i]) {
            current_motor_positions_[i] = msg->position;
            current_motor_velocities_[i] = msg->velocity;
            current_motor_torques_[i] = msg->torque;
            current_temperatures_[i] = msg->temperature;
            motors_online_[i] = msg->online;
            break;
        }
    }
}

void ArmControlNode::controlLoop() {
    control_loop_count_++;
    
    // 检查命令超时
    auto now = this->now();
    double time_since_cmd = (now - last_command_time_).seconds();
    if (time_since_cmd > command_timeout_) {
        // 超时保持当前位置
        target_motor_positions_ = current_motor_positions_;
        target_motor_velocities_.fill(0.0);
        target_motor_torques_.fill(0.0);
    }
    
    // 计算协调误差
    double coord_error = computeCoordinationError();
    if (coord_error > max_coordination_error_) {
        max_coordination_error_ = coord_error;
    }
    
    // 协调误差报警
    if (coord_error > params_.coordination_threshold) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "双臂协调误差过大: %.4f m (阈值: %.4f m)", 
            coord_error, params_.coordination_threshold);
    }
    
    // 发布电机命令
    publishMotorCommands();
    
    // 发布系统状态（降频到10Hz）
    if ((now - last_state_pub_time_).seconds() > 0.1) {
        publishStrikeState();
        last_state_pub_time_ = now;
    }
}

void ArmControlNode::publishMotorCommands() {
    auto now = this->now();
    
    // 4个电机独立控制（并联机构）
    for (size_t i = 0; i < 4; ++i) {
        auto cmd = motor_control_ros2::msg::UnitreeMotorCommand();
        cmd.header.stamp = now;
        cmd.joint_name = motor_names_[i];
        cmd.mode = 10;  // 闭环FOC模式
        
        cmd.pos_des = target_motor_positions_[i];
        cmd.vel_des = target_motor_velocities_[i];
        cmd.torque_ff = target_motor_torques_[i];
        cmd.kp = control_kp_[i];
        cmd.kd = control_kd_[i];
        
        motor_cmd_pub_->publish(cmd);
    }
}

void ArmControlNode::publishStrikeState() {
    auto state = motor_control_ros2::msg::ArmState();
    state.header.stamp = this->now();
    
    // 电机状态
    for (size_t i = 0; i < 4; ++i) {
        state.motor_positions[i] = current_motor_positions_[i];
        state.motor_velocities[i] = current_motor_velocities_[i];
        state.motor_torques[i] = current_motor_torques_[i];
        state.motor_temperatures[i] = current_temperatures_[i];
        state.motors_online[i] = motors_online_[i];
    }
    
    // 末端状态（正运动学）
    auto plate_pose = parallelFK(current_motor_positions_);
    state.plate_position.x = plate_pose[0];
    state.plate_position.y = plate_pose[1];
    state.plate_position.z = 0.0;  // 2D系统，Z=0
    state.plate_orientation = plate_pose[2];
    
    // 计算末端速度（数值微分 - 简化）
    static double last_x = plate_pose[0];
    static double last_y = plate_pose[1];
    static auto last_time = this->now();
    
    double dt = (this->now() - last_time).seconds();
    if (dt > 0.001) {
        state.plate_velocity.x = (plate_pose[0] - last_x) / dt;
        state.plate_velocity.y = (plate_pose[1] - last_y) / dt;
        state.plate_velocity.z = 0.0;
        
        last_x = plate_pose[0];
        last_y = plate_pose[1];
        last_time = this->now();
    }
    
    // 五杆机构状态
    state.left_elbow_angle = computeElbowAngle(
        current_motor_positions_[0], current_motor_positions_[1]);
    state.right_elbow_angle = computeElbowAngle(
        current_motor_positions_[2], current_motor_positions_[3]);
    
    state.left_upper_arm_angle = current_motor_positions_[0];
    state.right_upper_arm_angle = current_motor_positions_[2];
    
    // 协调性指标
    state.coordination_error = computeCoordinationError();
    
    // 运动学可行性（简化：检查是否在工作空间内）
    bool in_workspace = (plate_pose[0] >= params_.workspace_x_min && 
                        plate_pose[0] <= params_.workspace_x_max &&
                        plate_pose[1] >= params_.workspace_y_min && 
                        plate_pose[1] <= params_.workspace_y_max);
    state.feasibility_index = in_workspace ? 1.0 : 0.5;
    
    strike_state_pub_->publish(state);
}

// ========== 五杆机构运动学实现 ==========

std::array<double, 2> ArmControlNode::singleArmFK(double theta1, double theta2, bool is_left) {
    /**
     * 单臂正运动学：(主电机角, 副电机角) → 末端位置
     * 
     * 简化模型（忽略连杆约束，当作两自由度串联臂）：
     *   - 主电机控制大臂绕肩部旋转（theta1）
     *   - 副电机控制小臂相对大臂的角度（theta2）
     *   - 末端 = 肩部 + 大臂向量 + 小臂向量
     * 
     * 注意：完整五杆机构需要求解连杆闭环约束，这里简化处理
     */
    
    double L1 = params_.upper_arm_length;
    double L2 = params_.forearm_length;
    
    // 肩部位置（左右对称）
    double shoulder_y = is_left ? params_.arm_spacing / 2.0 : -params_.arm_spacing / 2.0;
    
    // 大臂末端（肘关节）位置
    double elbow_x = L1 * std::cos(theta1);
    double elbow_y = shoulder_y + L1 * std::sin(theta1);
    
    // 小臂方向（相对于大臂的绝对角度）
    // 简化：假设副电机直接控制小臂相对水平的角度
    double forearm_angle = theta1 + theta2;
    
    // 末端位置
    double end_x = elbow_x + L2 * std::cos(forearm_angle);
    double end_y = elbow_y + L2 * std::sin(forearm_angle);
    
    return {end_x, end_y};
}

bool ArmControlNode::singleArmIK(double target_x, double target_y, bool is_left,
                                 double& theta1, double& theta2) {
    /**
     * 单臂逆运动学：末端位置 → (主电机角, 副电机角)
     * 
     * 两自由度串联臂标准逆运动学（解析解）
     */
    
    double L1 = params_.upper_arm_length;
    double L2 = params_.forearm_length;
    
    // 相对于肩部的坐标
    double shoulder_y = is_left ? params_.arm_spacing / 2.0 : -params_.arm_spacing / 2.0;
    double dx = target_x;
    double dy = target_y - shoulder_y;
    
    // 到目标的距离
    double d = std::sqrt(dx * dx + dy * dy);
    
    // 检查可达性
    if (d > (L1 + L2) || d < std::abs(L1 - L2)) {
        RCLCPP_DEBUG(this->get_logger(), 
            "%s臂无解：距离%.3f超出范围[%.3f, %.3f]",
            is_left ? "左" : "右", d, std::abs(L1 - L2), L1 + L2);
        return false;
    }
    
    // 余弦定理求"肘"角度
    double cos_elbow = (L1 * L1 + L2 * L2 - d * d) / (2 * L1 * L2);
    if (std::abs(cos_elbow) > 1.0) {
        return false;
    }
    
    // 选择肘部弯曲方向（这里选负角，肘部向下/向内）
    double elbow_angle = -std::acos(cos_elbow);
    
    // 求肩部角度
    double phi = std::atan2(dy, dx);
    double beta = std::acos((L1 * L1 + d * d - L2 * L2) / (2 * L1 * d));
    
    theta1 = phi - beta;  // 主电机角度（大臂）
    theta2 = elbow_angle;  // 副电机角度（相对大臂）
    
    return true;
}

double ArmControlNode::computeElbowAngle(double theta1, double theta2) {
    /**
     * 计算肘关节实际角度
     * 简化：直接返回 theta1 + theta2（串联模型）
     * 完整模型需考虑连杆约束
     */
    return theta1 + theta2;
}

std::array<double, 3> ArmControlNode::parallelFK(const std::array<double, 4>& motor_angles) {
    /**
     * 并联正运动学：4个电机角度 → 击球板位姿 [x, y, orientation]
     * 
     * 方法：
     * 1. 计算左臂末端位置
     * 2. 计算右臂末端位置
     * 3. 击球板中心 = 两末端中点
     * 4. 击球板姿态 = 两末端连线方向
     */
    
    // 左臂末端
    auto left_end = singleArmFK(motor_angles[0], motor_angles[1], true);
    
    // 右臂末端
    auto right_end = singleArmFK(motor_angles[2], motor_angles[3], false);
    
    // 击球板中心位置
    double plate_x = (left_end[0] + right_end[0]) / 2.0;
    double plate_y = (left_end[1] + right_end[1]) / 2.0;
    
    // 击球板姿态（两末端连线的角度）
    double dx = right_end[0] - left_end[0];
    double dy = right_end[1] - left_end[1];
    double plate_orientation = std::atan2(dy, dx);
    
    return {plate_x, plate_y, plate_orientation};
}

bool ArmControlNode::parallelIK(double target_x, double target_y, double target_ori,
                                std::array<double, 4>& solution) {
    /**
     * 并联逆运动学：击球板位姿 → 4个电机角度
     * 
     * 方法：
     * 1. 根据击球板姿态和尺寸，计算左右末端应在的位置
     * 2. 对每条臂分别求逆运动学
     * 3. 检查解的一致性
     */
    
    double half_width = params_.plate_width / 2.0;
    
    // 计算左臂应该到达的位置（击球板左端点）
    double left_target_x = target_x - half_width * std::cos(target_ori);
    double left_target_y = target_y - half_width * std::sin(target_ori);
    
    // 计算右臂应该到达的位置（击球板右端点）
    double right_target_x = target_x + half_width * std::cos(target_ori);
    double right_target_y = target_y + half_width * std::sin(target_ori);
    
    // 左臂逆运动学
    double left_theta1, left_theta2;
    if (!singleArmIK(left_target_x, left_target_y, true, left_theta1, left_theta2)) {
        return false;
    }
    
    // 右臂逆运动学
    double right_theta1, right_theta2;
    if (!singleArmIK(right_target_x, right_target_y, false, right_theta1, right_theta2)) {
        return false;
    }
    
    // 输出解
    solution[0] = left_theta1;   // 左主电机
    solution[1] = left_theta2;   // 左副电机
    solution[2] = right_theta1;  // 右主电机
    solution[3] = right_theta2;  // 右副电机
    
    return true;
}

double ArmControlNode::computeCoordinationError() {
    /**
     * 计算双臂协调误差：左右臂末端位置差异
     */
    
    if (!motors_online_[0] || !motors_online_[1] || 
        !motors_online_[2] || !motors_online_[3]) {
        return 0.0;  // 电机离线，无法计算
    }
    
    // 左臂末端
    auto left_end = singleArmFK(current_motor_positions_[0], 
                                current_motor_positions_[1], true);
    
    // 右臂末端
    auto right_end = singleArmFK(current_motor_positions_[2], 
                                 current_motor_positions_[3], false);
    
    // 欧氏距离
    double dx = left_end[0] - right_end[0];
    double dy = left_end[1] - right_end[1];
    
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::ArmControlNode>();
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "节点运行异常: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}

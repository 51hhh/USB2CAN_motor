#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

namespace motor_control {

class VisionCatchControllerNode : public rclcpp::Node {
public:
    VisionCatchControllerNode() : Node("vision_catch_controller_node") {
        this->declare_parameter("config_file", "");

        const std::string config_file = resolveConfigFile();

        try {
            loadParams(config_file);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "vision_catch_controller_node 配置加载失败: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "配置文件路径: %s", config_file.c_str());
            throw;
        }

        if (control_frequency_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "control_frequency=%.3f 非法，回退到 50.0", control_frequency_);
            control_frequency_ = 50.0;
        }
        if (approach_gain_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "approach_gain=%.3f 非法，回退到 1.0", approach_gain_);
            approach_gain_ = 1.0;
        }
        if (max_linear_speed_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "max_linear_speed=%.3f 非法，回退到 1.0", max_linear_speed_);
            max_linear_speed_ = 1.0;
        }
        if (accel_limit_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "accel_limit=%.3f 非法，回退到 2.0", accel_limit_);
            accel_limit_ = 2.0;
        }
        if (decel_limit_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "decel_limit=%.3f 非法，回退到 3.0", decel_limit_);
            decel_limit_ = 3.0;
        }
        if (position_tolerance_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "position_tolerance=%.3f 非法，回退到 0.05", position_tolerance_);
            position_tolerance_ = 0.05;
        }
        if (goal_timeout_sec_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "goal_timeout_sec=%.3f 非法，回退到 1.0", goal_timeout_sec_);
            goal_timeout_sec_ = 1.0;
        }
        if (odom_timeout_sec_ <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "odom_timeout_sec=%.3f 非法，回退到 0.5", odom_timeout_sec_);
            odom_timeout_sec_ = 0.5;
        }
        if (yaw_kp_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "yaw_kp=%.3f 非法，回退到 1.5", yaw_kp_);
            yaw_kp_ = 1.5;
        }
        if (max_angular_speed_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "max_angular_speed=%.3f 非法，回退到 0.1", max_angular_speed_);
            max_angular_speed_ = 0.1;
        }
        if (std::abs(yaw_cmd_sign_) <= 1e-6) {
            RCLCPP_WARN(this->get_logger(), "yaw_cmd_sign=%.3f 非法，回退到 -1.0", yaw_cmd_sign_);
            yaw_cmd_sign_ = -1.0;
        }
        if (yaw_tolerance_rad_ < 0.0) {
            RCLCPP_WARN(this->get_logger(), "yaw_tolerance_rad=%.3f 非法，回退到 0.03", yaw_tolerance_rad_);
            yaw_tolerance_rad_ = 0.03;
        }

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            goal_topic_, rclcpp::SensorDataQoS(),
            std::bind(&VisionCatchControllerNode::goalCallback, this, std::placeholders::_1));//接球目标订阅(BEST_EFFORT匹配视觉发布者)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 20,
            std::bind(&VisionCatchControllerNode::odomCallback, this, std::placeholders::_1));//里程计订阅
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);//速度发布

        auto period = std::chrono::duration<double>(1.0 / control_frequency_);
        control_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            std::bind(&VisionCatchControllerNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "vision_catch_controller_node 启动: goal=%s odom=%s cmd_vel=%s goal_frame=%s freq=%.1fHz",
            goal_topic_.c_str(), odom_topic_.c_str(), cmd_vel_topic_.c_str(), goal_frame_.c_str(),
            control_frequency_);
    }

private:
    std::string resolveConfigFile() {
        std::string config_file = this->get_parameter("config_file").as_string();
        if (!config_file.empty()) {
            return config_file;
        }

        return ament_index_cpp::get_package_share_directory("motor_control_ros2") +
            "/config/vision_catch_controller_params.yaml";
    }

    void loadParams(const std::string & config_file) {
        RCLCPP_INFO(this->get_logger(), "正在加载视觉接球控制参数: %s", config_file.c_str());

        YAML::Node root = YAML::LoadFile(config_file);
        YAML::Node params = root;

        if (root["vision_catch_controller_node"] && root["vision_catch_controller_node"]["ros__parameters"]) {
            params = root["vision_catch_controller_node"]["ros__parameters"];
        }

        auto loadString = [&params](const char * key, std::string & value) {
            if (params[key]) {
                value = params[key].as<std::string>();
            }
        };

        auto loadDouble = [&params](const char * key, double & value) {
            if (params[key]) {
                value = params[key].as<double>();
            }
        };

        loadString("goal_topic", goal_topic_);
        loadString("odom_topic", odom_topic_);
        loadString("cmd_vel_topic", cmd_vel_topic_);
        loadString("goal_frame", goal_frame_);
        loadDouble("control_frequency", control_frequency_);
        loadDouble("approach_gain", approach_gain_);
        loadDouble("max_linear_speed", max_linear_speed_);
        loadDouble("accel_limit", accel_limit_);
        loadDouble("decel_limit", decel_limit_);
        loadDouble("position_tolerance", position_tolerance_);
        loadDouble("goal_timeout_sec", goal_timeout_sec_);
        loadDouble("odom_timeout_sec", odom_timeout_sec_);
        loadDouble("yaw_kp", yaw_kp_);
        loadDouble("max_angular_speed", max_angular_speed_);
        loadDouble("yaw_cmd_sign", yaw_cmd_sign_);
        loadDouble("yaw_tolerance_rad", yaw_tolerance_rad_);
        if (params["yaw_hold_enabled"]) {
            yaw_hold_enabled_ = params["yaw_hold_enabled"].as<bool>();
        }
        if (params["use_goal_yaw"]) {
            use_goal_yaw_ = params["use_goal_yaw"].as<bool>();
        }
        if (params["speed_profile_enabled"]) {
            speed_profile_enabled_ = params["speed_profile_enabled"].as<bool>();
        }
        if (params["rotate_after_position_reached"]) {
            rotate_after_position_reached_ = params["rotate_after_position_reached"].as<bool>();
        }

        RCLCPP_INFO(this->get_logger(), "vision_catch_controller_node 参数加载完成: %s", config_file.c_str());
    }

    static double yawFromQuaternion(const geometry_msgs::msg::Quaternion & q) {
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    static bool sameGoalPosition(
        const geometry_msgs::msg::PointStamped & lhs,
        const geometry_msgs::msg::PointStamped & rhs) {
        constexpr double same_position_epsilon = 1e-6;
        return std::abs(lhs.point.x - rhs.point.x) <= same_position_epsilon &&
            std::abs(lhs.point.y - rhs.point.y) <= same_position_epsilon;
    }

    geometry_msgs::msg::Twist zeroTwist() const {
        return geometry_msgs::msg::Twist();
    }

    void initializeYawHoldFromCurrentOdom(const char * reason) {
        if (!has_odom_) {
            return;
        }

        target_yaw_ = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        target_yaw_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "yaw 保持目标初始化(%s): %.3f rad", reason, target_yaw_);
    }

    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        const bool first_goal = !has_goal_;
        const bool goal_position_changed = has_goal_ && !sameGoalPosition(*msg, latest_goal_);

        latest_goal_ = *msg;
        last_goal_time_ = this->now();
        has_goal_ = true;
        reached_goal_ = false;

        if (!msg->header.frame_id.empty() && msg->header.frame_id != goal_frame_) {
            RCLCPP_WARN(this->get_logger(),
                "收到目标 frame_id=%s，当前测试模式仍按 %s 解释，请后续接 TF",
                msg->header.frame_id.c_str(), goal_frame_.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "更新接球目标: x=%.3f y=%.3f",
            msg->point.x, msg->point.y);

        // PointStamped 不含 orientation，yaw 保持由 odom 初始化
        if (yaw_hold_enabled_ &&
            (first_goal || goal_position_changed || !target_yaw_initialized_)) {
            initializeYawHoldFromCurrentOdom("当前目标");
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        last_odom_time_ = this->now();
        has_odom_ = true;

        if (yaw_hold_enabled_ && !use_goal_yaw_ && has_goal_ && !target_yaw_initialized_) {
            initializeYawHoldFromCurrentOdom("首帧可用 /odom");
        }
    }

    double computeYawCommand(double current_yaw) const {
        if (!yaw_hold_enabled_ || !target_yaw_initialized_) {
            return 0.0;
        }

        const double yaw_error = normalizeAngle(target_yaw_ - current_yaw);
        if (std::abs(yaw_error) <= yaw_tolerance_rad_) {
            return 0.0;
        }

        const double yaw_cmd = std::clamp(yaw_kp_ * yaw_error, -max_angular_speed_, max_angular_speed_);
        return yaw_cmd_sign_ * yaw_cmd;
    }

    double calculateControlDt(const rclcpp::Time & now) {
        double dt = 1.0 / control_frequency_;//时间间隔
        if (last_control_time_.nanoseconds() > 0) {
            dt = (now - last_control_time_).seconds();
            if (dt <= 0.0) {
                dt = 1.0 / control_frequency_;
            }
        }
        last_control_time_ = now;
        return dt;
    }

    double computeProfileSpeed(double distance, double dt) const {
        if (!speed_profile_enabled_) {
            return std::min(max_linear_speed_, approach_gain_ * distance);
        }

        const double remaining_distance = std::max(0.0, distance - position_tolerance_);
        const double brake_limited_speed = std::sqrt(2.0 * decel_limit_ * remaining_distance);
        const double target_speed = std::min(max_linear_speed_, brake_limited_speed);
        const double current_speed = std::hypot(last_cmd_linear_x_, last_cmd_linear_y_);

        if (target_speed > current_speed) {
            return std::min(target_speed, current_speed + accel_limit_ * dt);
        }
        return std::max(target_speed, current_speed - decel_limit_ * dt);
    }

    void limitLinearCommand(
        double target_x,
        double target_y,
        double dt,
        double & limited_x,
        double & limited_y) const {
        if (!speed_profile_enabled_) {
            limited_x = target_x;
            limited_y = target_y;
            return;
        }

        const double delta_x = target_x - last_cmd_linear_x_;
        const double delta_y = target_y - last_cmd_linear_y_;
        const double delta_norm = std::hypot(delta_x, delta_y);
        if (delta_norm <= 0.0) {
            limited_x = target_x;
            limited_y = target_y;
            return;
        }

        const double current_speed = std::hypot(last_cmd_linear_x_, last_cmd_linear_y_);
        const double target_speed = std::hypot(target_x, target_y);
        const double limit = ((target_speed > current_speed) ? accel_limit_ : decel_limit_) * dt;
        if (delta_norm <= limit) {
            limited_x = target_x;
            limited_y = target_y;
            return;
        }

        const double scale = limit / delta_norm;
        limited_x = last_cmd_linear_x_ + delta_x * scale;
        limited_y = last_cmd_linear_y_ + delta_y * scale;
    }

    void resetSpeedProfile() {
        last_cmd_linear_x_ = 0.0;
        last_cmd_linear_y_ = 0.0;
        last_control_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    }

    void controlLoop() {
        if ( !has_odom_) {
            resetSpeedProfile();
            RCLCPP_ERROR(this->get_logger(), "缺少里程计信息，无法控制，停车等待");
            publishCommand(zeroTwist());
            return;
        }
         if (!has_goal_ ) {
            resetSpeedProfile();
            RCLCPP_ERROR(this->get_logger(), "缺少接球目标信息，无法控制，停车等待");
            publishCommand(zeroTwist());
            return;
        }

        const auto now = this->now();
        if ((now - last_goal_time_).seconds() > goal_timeout_sec_) {
            if (!goal_timeout_reported_) {
                RCLCPP_WARN(this->get_logger(), "目标超时，停车等待新落点");
                goal_timeout_reported_ = true;
            }
            resetSpeedProfile();
            publishCommand(zeroTwist());
            return;
        }
        goal_timeout_reported_ = false;

        if ((now - last_odom_time_).seconds() > odom_timeout_sec_) {
            if (!odom_timeout_reported_) {
                RCLCPP_WARN(this->get_logger(), "/odom 超时，停车等待位姿恢复");
                odom_timeout_reported_ = true;
            }
            resetSpeedProfile();
            publishCommand(zeroTwist());
            return;
        }
        odom_timeout_reported_ = false;
        const double dt = calculateControlDt(now);

        const double current_x = latest_odom_.pose.pose.position.x;
        const double current_y = latest_odom_.pose.pose.position.y;

        const double current_yaw = yawFromQuaternion(latest_odom_.pose.pose.orientation);
        const double yaw_cmd = computeYawCommand(current_yaw);

        const double dx_world = latest_goal_.point.x - current_x;
        const double dy_world = latest_goal_.point.y - current_y;
        const double distance = std::hypot(dx_world, dy_world);

        if (distance <= position_tolerance_) {
            if (!reached_goal_) {
                RCLCPP_INFO(this->get_logger(), "已到达接球等待点，进入停车等待");
                reached_goal_ = true;
            }
            resetSpeedProfile();
            geometry_msgs::msg::Twist cmd;
            cmd.angular.z = rotate_after_position_reached_ ? yaw_cmd : 0.0;
            publishCommand(cmd);
            return;
        }
        reached_goal_ = false;

        const double ex_body = std::cos(current_yaw) * dx_world + std::sin(current_yaw) * dy_world;
        const double ey_body = -std::sin(current_yaw) * dx_world + std::cos(current_yaw) * dy_world;
        const double heading_error = normalizeAngle(std::atan2(ey_body, ex_body));

        const double speed = computeProfileSpeed(distance, dt);
        const double target_linear_x = speed * std::cos(heading_error);
        const double target_linear_y = speed * std::sin(heading_error);

        geometry_msgs::msg::Twist cmd;
        limitLinearCommand(target_linear_x, target_linear_y, dt, cmd.linear.x, cmd.linear.y);
        cmd.angular.z = yaw_cmd;
        last_cmd_linear_x_ = cmd.linear.x;
        last_cmd_linear_y_ = cmd.linear.y;

        publishCommand(cmd);
    }

    void publishCommand(const geometry_msgs::msg::Twist & cmd) {
        cmd_vel_pub_->publish(cmd);
    }

    std::string goal_topic_ {"/ball/landing"};
    std::string odom_topic_ {"/odom"};
    std::string cmd_vel_topic_ {"/cmd_vel_remote"};
    std::string goal_frame_ {"vision_world"};

    double control_frequency_ {50.0};
    double approach_gain_ {1.0};
    double max_linear_speed_ {1.0};
    double accel_limit_ {2.0};
    double decel_limit_ {3.0};
    double position_tolerance_ {0.05};
    double goal_timeout_sec_ {1.0};
    double odom_timeout_sec_ {0.5};
    bool speed_profile_enabled_ {true};
    bool yaw_hold_enabled_ {false};
    bool use_goal_yaw_ {false};
    bool rotate_after_position_reached_ {true};
    double yaw_kp_ {1.5};
    double max_angular_speed_ {0.1};
    double yaw_cmd_sign_ {-1.0};
    double yaw_tolerance_rad_ {0.03};
    double target_yaw_ {0.0};

    bool has_goal_ {false};
    bool has_odom_ {false};
    bool reached_goal_ {false};
    bool target_yaw_initialized_ {false};
    bool goal_timeout_reported_ {false};
    bool odom_timeout_reported_ {false};
    double last_cmd_linear_x_ {0.0};
    double last_cmd_linear_y_ {0.0};

    geometry_msgs::msg::PointStamped latest_goal_;
    nav_msgs::msg::Odometry latest_odom_;
    rclcpp::Time last_goal_time_;
    rclcpp::Time last_odom_time_;
    rclcpp::Time last_control_time_ {0, 0, RCL_ROS_TIME};

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace motor_control

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::VisionCatchControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

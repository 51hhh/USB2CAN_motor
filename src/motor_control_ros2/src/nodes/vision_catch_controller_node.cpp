#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <yaml-cpp/yaml.h>

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

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_, 10,
            std::bind(&VisionCatchControllerNode::goalCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 20,
            std::bind(&VisionCatchControllerNode::odomCallback, this, std::placeholders::_1));
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

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
        loadDouble("position_tolerance", position_tolerance_);
        loadDouble("goal_timeout_sec", goal_timeout_sec_);
        loadDouble("odom_timeout_sec", odom_timeout_sec_);

        RCLCPP_INFO(this->get_logger(), "vision_catch_controller_node 参数加载完成: %s", config_file.c_str());
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

    geometry_msgs::msg::Twist zeroTwist() const {
        return geometry_msgs::msg::Twist();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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
            msg->pose.position.x, msg->pose.position.y);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = *msg;
        last_odom_time_ = this->now();
        has_odom_ = true;
    }

    void controlLoop() {
        if (!has_goal_ || !has_odom_) {
            publishIfChanged(zeroTwist());
            return;
        }

        const auto now = this->now();
        if ((now - last_goal_time_).seconds() > goal_timeout_sec_) {
            if (!goal_timeout_reported_) {
                RCLCPP_WARN(this->get_logger(), "目标超时，停车等待新落点");
                goal_timeout_reported_ = true;
            }
            publishIfChanged(zeroTwist());
            return;
        }
        goal_timeout_reported_ = false;

        if ((now - last_odom_time_).seconds() > odom_timeout_sec_) {
            if (!odom_timeout_reported_) {
                RCLCPP_WARN(this->get_logger(), "/odom 超时，停车等待位姿恢复");
                odom_timeout_reported_ = true;
            }
            publishIfChanged(zeroTwist());
            return;
        }
        odom_timeout_reported_ = false;

        const double current_x = latest_odom_.pose.pose.position.x;
        const double current_y = latest_odom_.pose.pose.position.y;

        const auto & q = latest_odom_.pose.pose.orientation;
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double current_yaw = std::atan2(siny_cosp, cosy_cosp);

        const double dx_world = latest_goal_.pose.position.x - current_x;
        const double dy_world = latest_goal_.pose.position.y - current_y;
        const double distance = std::hypot(dx_world, dy_world);

        if (distance <= position_tolerance_) {
            if (!reached_goal_) {
                RCLCPP_INFO(this->get_logger(), "已到达接球等待点，进入停车等待");
                reached_goal_ = true;
            }
            publishIfChanged(zeroTwist());
            return;
        }
        reached_goal_ = false;

        const double ex_body = std::cos(current_yaw) * dx_world + std::sin(current_yaw) * dy_world;
        const double ey_body = -std::sin(current_yaw) * dx_world + std::cos(current_yaw) * dy_world;
        const double heading_error = normalizeAngle(std::atan2(ey_body, ex_body));

        const double speed = std::min(max_linear_speed_, approach_gain_ * distance);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = speed * std::cos(heading_error);
        cmd.linear.y = speed * std::sin(heading_error);
        cmd.angular.z = 0.0;

        publishIfChanged(cmd);
    }

    void publishIfChanged(const geometry_msgs::msg::Twist & cmd) {
        constexpr double eps = 1e-6;
        const bool changed =
            std::abs(last_published_cmd_.linear.x - cmd.linear.x) > eps ||
            std::abs(last_published_cmd_.linear.y - cmd.linear.y) > eps ||
            std::abs(last_published_cmd_.angular.z - cmd.angular.z) > eps;

        if (changed || !has_published_cmd_) {
            cmd_vel_pub_->publish(cmd);
            last_published_cmd_ = cmd;
            has_published_cmd_ = true;
        }
    }

    std::string goal_topic_ {"/auto/goal_pose"};
    std::string odom_topic_ {"/odom"};
    std::string cmd_vel_topic_ {"/cmd_vel_remote"};
    std::string goal_frame_ {"vision_world"};

    double control_frequency_ {50.0};
    double approach_gain_ {1.0};
    double max_linear_speed_ {1.0};
    double position_tolerance_ {0.05};
    double goal_timeout_sec_ {1.0};
    double odom_timeout_sec_ {0.5};

    bool has_goal_ {false};
    bool has_odom_ {false};
    bool reached_goal_ {false};
    bool goal_timeout_reported_ {false};
    bool odom_timeout_reported_ {false};
    bool has_published_cmd_ {false};

    geometry_msgs::msg::PoseStamped latest_goal_;
    nav_msgs::msg::Odometry latest_odom_;
    geometry_msgs::msg::Twist last_published_cmd_;
    rclcpp::Time last_goal_time_;
    rclcpp::Time last_odom_time_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

namespace motor_control {

class CmdVelMuxNode : public rclcpp::Node {
public:
    CmdVelMuxNode() : Node("cmd_vel_mux_node") {
        this->declare_parameter("config_file", "");

        const std::string config_file = resolveConfigFile();
        try {
            loadParams(config_file);
        } catch (const std::exception & e) {
            RCLCPP_ERROR(this->get_logger(), "cmd_vel_mux_node 配置加载失败: %s", e.what());
            RCLCPP_ERROR(this->get_logger(), "配置文件路径: %s", config_file.c_str());
            throw;
        }

        // active_source 单独作为 ROS2 参数，支持运行时 ros2 param set 切换
        this->declare_parameter("active_source", active_source_);
        active_source_ = this->get_parameter("active_source").as_string();

        if (!isValidSource(active_source_)) {
            RCLCPP_WARN(this->get_logger(),
                "未知 active_source=%s，回退到 joy", active_source_.c_str());
            active_source_ = "joy";
        }

        if (source_timeout_sec_ < 0.0) {
            RCLCPP_WARN(this->get_logger(),
                "source_timeout_sec=%.3f 非法，回退到 0.5", source_timeout_sec_);
            source_timeout_sec_ = 0.5;
        }

        if (timeout_mode_ != "brake" && timeout_mode_ != "fallback") {
            RCLCPP_WARN(this->get_logger(),
                "未知 timeout_mode=%s，回退到 brake", timeout_mode_.c_str());
            timeout_mode_ = "brake";
        }

        if (!isValidSource(fallback_source_)) {
            RCLCPP_WARN(this->get_logger(),
                "未知 fallback_source=%s，回退到 joy", fallback_source_.c_str());
            fallback_source_ = "joy";
        }

        joy_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            joy_input_topic_, 10,
            std::bind(&CmdVelMuxNode::joyCallback, this, std::placeholders::_1));

        remote_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            remote_input_topic_, 10,
            std::bind(&CmdVelMuxNode::remoteCallback, this, std::placeholders::_1));

        joy_raw_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_raw_topic_, 10,
            std::bind(&CmdVelMuxNode::joyRawCallback, this, std::placeholders::_1));

        output_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

        const auto now = this->now();
        last_joy_stamp_ = now;
        last_remote_stamp_ = now;

        timeout_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&CmdVelMuxNode::checkSourceTimeout, this));

        param_cb_handle_ = this->add_on_set_parameters_callback(
            std::bind(&CmdVelMuxNode::onSetParameters, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "cmd_vel_mux_node 启动: joy=%s remote=%s output=%s active=%s timeout=%.3fs mode=%s fallback=%s lock=%s btn_remote=%d btn_joy=%d",
            joy_input_topic_.c_str(), remote_input_topic_.c_str(),
            output_topic_.c_str(), active_source_.c_str(), source_timeout_sec_,
            timeout_mode_.c_str(), fallback_source_.c_str(), lock_active_source_ ? "true" : "false",
            switch_to_remote_button_, switch_to_joy_button_);
    }

private:
    std::string resolveConfigFile() {
        std::string config_file = this->get_parameter("config_file").as_string();
        if (!config_file.empty()) {
            return config_file;
        }
        return ament_index_cpp::get_package_share_directory("motor_control_ros2")
               + "/config/cmd_vel_mux_params.yaml";
    }

    void loadParams(const std::string & config_file) {
        RCLCPP_INFO(this->get_logger(), "正在加载 cmd_vel_mux 参数: %s", config_file.c_str());
        YAML::Node root = YAML::LoadFile(config_file);
        YAML::Node p = root;
        if (root["cmd_vel_mux_node"] && root["cmd_vel_mux_node"]["ros__parameters"]) {
            p = root["cmd_vel_mux_node"]["ros__parameters"];
        }

        auto loadStr = [&p](const char * key, std::string & out) {
            if (p[key]) out = p[key].as<std::string>();
        };
        auto loadDbl = [&p](const char * key, double & out) {
            if (p[key]) out = p[key].as<double>();
        };
        auto loadBool = [&p](const char * key, bool & out) {
            if (p[key]) out = p[key].as<bool>();
        };
        auto loadInt = [&p](const char * key, int & out) {
            if (p[key]) out = p[key].as<int>();
        };

        loadStr("joy_input_topic", joy_input_topic_);
        loadStr("remote_input_topic", remote_input_topic_);
        loadStr("output_topic", output_topic_);
        loadStr("active_source", active_source_);
        loadDbl("source_timeout_sec", source_timeout_sec_);
        loadStr("timeout_mode", timeout_mode_);
        loadStr("fallback_source", fallback_source_);
        loadBool("lock_active_source", lock_active_source_);
        loadStr("joy_raw_topic", joy_raw_topic_);
        loadInt("switch_to_remote_button", switch_to_remote_button_);
        loadInt("switch_to_joy_button", switch_to_joy_button_);

        RCLCPP_INFO(this->get_logger(), "cmd_vel_mux 参数加载完成");
    }

    bool isValidSource(const std::string& source) const {
        return source == "joy" || source == "remote";
    }

    std::string sourceText(const std::string& source) const {
        if (source == "joy") {
            return "joy(手柄)";
        }
        if (source == "remote") {
            return "remote(远程/视觉)";
        }
        return source;
    }

    geometry_msgs::msg::Twist zeroTwist() const {
        return geometry_msgs::msg::Twist();
    }

    bool isSourceFreshLocked(const std::string& source, const rclcpp::Time& now) const {
        if (source == "joy") {
            return has_joy_ && ((now - last_joy_stamp_).seconds() <= source_timeout_sec_);
        }
        return has_remote_ && ((now - last_remote_stamp_).seconds() <= source_timeout_sec_);
    }

    geometry_msgs::msg::Twist latestSourceCmdLocked(const std::string& source) const {
        return (source == "joy") ? last_joy_cmd_ : last_remote_cmd_;
    }

    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter>& parameters) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        bool publish_cached = false;
        geometry_msgs::msg::Twist cached_cmd;

        for (const auto& parameter : parameters) {
            if (parameter.get_name() != "active_source") {
                continue;
            }

            const auto value = parameter.as_string();
            if (!isValidSource(value)) {
                result.successful = false;
                result.reason = "active_source 仅支持 joy 或 remote";
                return result;
            }

            {
                std::lock_guard<std::mutex> lock(source_mutex_);
                if (lock_active_source_ && value != active_source_) {
                    result.successful = false;
                    result.reason = "lock_active_source=true，禁止运行时切换控制源";
                    return result;
                }
                active_source_ = value;
                source_timeout_active_ = false;
                if (isSourceFreshLocked(value, this->now())) {
                    cached_cmd = latestSourceCmdLocked(value);
                    publish_cached = true;
                }
            }
            RCLCPP_INFO(this->get_logger(), "切换 active_source -> %s", value.c_str());
        }

        if (publish_cached) {
            output_pub_->publish(cached_cmd);
        }

        return result;
    }

    void joyCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        bool should_publish = false;
        bool should_log_recovered = false;
        std::string active_source_snapshot;
        const auto now = this->now();
        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            if (has_joy_ && ((now - last_joy_stamp_).seconds() > source_timeout_sec_)) {
                should_log_recovered = true;
                active_source_snapshot = active_source_;
            }
            last_joy_cmd_ = *msg;
            last_joy_stamp_ = now;
            has_joy_ = true;
            if (active_source_ == "joy") {
                source_timeout_active_ = false;
                should_publish = true;
            }
        }

        if (should_publish) {
            output_pub_->publish(*msg);
        }

        if (should_log_recovered) {
            if (active_source_snapshot == "joy") {
                RCLCPP_INFO(this->get_logger(),
                    "joy(手柄) 重新收到指令，控制已恢复");
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "joy(手柄) 重新收到指令，连接已恢复；当前仍由 %s 控制",
                    sourceText(active_source_snapshot).c_str());
            }
        }
    }

    void remoteCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        bool should_publish = false;
        bool should_log_recovered = false;
        std::string active_source_snapshot;
        const auto now = this->now();
        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            if (has_remote_ && ((now - last_remote_stamp_).seconds() > source_timeout_sec_)) {
                should_log_recovered = true;
                active_source_snapshot = active_source_;
            }
            last_remote_cmd_ = *msg;
            last_remote_stamp_ = now;
            has_remote_ = true;
            if (active_source_ == "remote") {
                source_timeout_active_ = false;
                should_publish = true;
            }
        }

        if (should_publish) {
            output_pub_->publish(*msg);
        }

        if (should_log_recovered) {
            if (active_source_snapshot == "remote") {
                RCLCPP_INFO(this->get_logger(),
                    "remote(远程/视觉) 重新收到指令，控制已恢复");
            } else {
                RCLCPP_INFO(this->get_logger(),
                    "remote(远程/视觉) 重新收到指令，连接已恢复；当前仍由 %s 控制，如需切回请按 RB 或设置 active_source=remote",
                    sourceText(active_source_snapshot).c_str());
            }
        }
    }

    void joyRawCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        const int n = static_cast<int>(msg->buttons.size());
        std::string new_source;

        if (switch_to_remote_button_ >= 0 && switch_to_remote_button_ < n &&
            msg->buttons[switch_to_remote_button_] == 1 &&
            prev_remote_btn_ == 0) {
            new_source = "remote";
        } else if (switch_to_joy_button_ >= 0 && switch_to_joy_button_ < n &&
                   msg->buttons[switch_to_joy_button_] == 1 &&
                   prev_joy_btn_ == 0) {
            new_source = "joy";
        }

        if (switch_to_remote_button_ >= 0 && switch_to_remote_button_ < n) {
            prev_remote_btn_ = msg->buttons[switch_to_remote_button_];
        }
        if (switch_to_joy_button_ >= 0 && switch_to_joy_button_ < n) {
            prev_joy_btn_ = msg->buttons[switch_to_joy_button_];
        }

        if (new_source.empty()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            if (lock_active_source_ && new_source != active_source_) {
                RCLCPP_WARN(this->get_logger(),
                    "lock_active_source=true，按键切换被拒绝");
                return;
            }
            if (new_source == active_source_) {
                return;
            }
            active_source_ = new_source;
            source_timeout_active_ = false;
        }
        RCLCPP_INFO(this->get_logger(),
            "按键切换 active_source -> %s", new_source.c_str());
    }

    void checkSourceTimeout() {
        if (source_timeout_sec_ <= 0.0) {
            return;
        }

        geometry_msgs::msg::Twist cmd_to_publish;
        bool should_publish = false;
        bool should_warn_timeout = false;
        std::string timed_out_source;
        std::string switched_to;

        {
            std::lock_guard<std::mutex> lock(source_mutex_);

            // 两个来源都从未收到数据（如启动阶段无手柄）→ 静默等待，不触发超时
            if (!has_joy_ && !has_remote_) {
                return;
            }

            const auto now = this->now();
            if (isSourceFreshLocked(active_source_, now)) {
                source_timeout_active_ = false;
                return;
            }

            timed_out_source = active_source_;
            should_warn_timeout = !source_timeout_active_;

            if (timeout_mode_ == "fallback" && fallback_source_ != active_source_ &&
                isSourceFreshLocked(fallback_source_, now)) {
                active_source_ = fallback_source_;
                switched_to = active_source_;
                source_timeout_active_ = false;
                cmd_to_publish = latestSourceCmdLocked(active_source_);
                should_publish = true;
            } else {
                source_timeout_active_ = true;
                cmd_to_publish = zeroTwist();
                should_publish = true;
            }
        }

        if (!switched_to.empty()) {
            RCLCPP_WARN(this->get_logger(),
                "%s 在 %.3fs 内未收到新指令，已自动回退到 %s 继续控制；若 %s 恢复，需要手动切回",
                sourceText(timed_out_source).c_str(), source_timeout_sec_,
                sourceText(switched_to).c_str(), sourceText(timed_out_source).c_str());
        } else if (should_warn_timeout) {
            RCLCPP_WARN(this->get_logger(),
                "%s 在 %.3fs 内未收到新指令，已保持当前 active_source 不变并输出 0 速执行刹停保护；待该来源恢复后会继续响应",
                sourceText(timed_out_source).c_str(), source_timeout_sec_);
        }

        if (should_publish) {
            output_pub_->publish(cmd_to_publish);
        }
    }

    std::string joy_input_topic_;
    std::string remote_input_topic_;
    std::string output_topic_;
    std::string active_source_;
    double source_timeout_sec_ {0.5};
    std::string timeout_mode_ {"brake"};
    std::string fallback_source_ {"joy"};
    bool lock_active_source_ {false};
    std::string joy_raw_topic_ {"/joy"};
    int switch_to_remote_button_ {5};
    int switch_to_joy_button_ {4};

    mutable std::mutex source_mutex_;

    geometry_msgs::msg::Twist last_joy_cmd_;
    geometry_msgs::msg::Twist last_remote_cmd_;
    rclcpp::Time last_joy_stamp_;
    rclcpp::Time last_remote_stamp_;
    bool has_joy_ {false};
    bool has_remote_ {false};
    bool source_timeout_active_ {false};
    int prev_remote_btn_ {0};
    int prev_joy_btn_ {0};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr remote_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_raw_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr output_pub_;
    rclcpp::TimerBase::SharedPtr timeout_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

}  // namespace motor_control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<motor_control::CmdVelMuxNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

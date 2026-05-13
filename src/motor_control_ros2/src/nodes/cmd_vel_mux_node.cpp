#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <chrono>
#include <mutex>
#include <string>
#include <vector>

namespace motor_control {

class CmdVelMuxNode : public rclcpp::Node {
public:
    CmdVelMuxNode() : Node("cmd_vel_mux_node") {
        this->declare_parameter("joy_input_topic", "/cmd_vel_joy");
        this->declare_parameter("remote_input_topic", "/cmd_vel_remote");
        this->declare_parameter("output_topic", "/cmd_vel");
        this->declare_parameter("active_source", "remote");
        this->declare_parameter("source_timeout_sec", 0.5);
        this->declare_parameter("timeout_mode", "brake");
        this->declare_parameter("fallback_source", "joy");
        this->declare_parameter("lock_active_source", false);

        joy_input_topic_ = this->get_parameter("joy_input_topic").as_string();
        remote_input_topic_ = this->get_parameter("remote_input_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        active_source_ = this->get_parameter("active_source").as_string();
        source_timeout_sec_ = this->get_parameter("source_timeout_sec").as_double();
        timeout_mode_ = this->get_parameter("timeout_mode").as_string();
        fallback_source_ = this->get_parameter("fallback_source").as_string();
        lock_active_source_ = this->get_parameter("lock_active_source").as_bool();

        if (!isValidSource(active_source_)) {
            RCLCPP_WARN(this->get_logger(),
                "未知 active_source=%s，回退到 remote", active_source_.c_str());
            active_source_ = "remote";
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
            "cmd_vel_mux_node 启动: joy=%s remote=%s output=%s active=%s timeout=%.3fs mode=%s fallback=%s lock=%s",
            joy_input_topic_.c_str(), remote_input_topic_.c_str(),
            output_topic_.c_str(), active_source_.c_str(), source_timeout_sec_,
            timeout_mode_.c_str(), fallback_source_.c_str(), lock_active_source_ ? "true" : "false");
    }

private:
    bool isValidSource(const std::string& source) const {
        return source == "joy" || source == "remote";
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
        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            last_joy_cmd_ = *msg;
            last_joy_stamp_ = this->now();
            has_joy_ = true;
            if (active_source_ == "joy") {
                source_timeout_active_ = false;
                should_publish = true;
            }
        }

        if (should_publish) {
            output_pub_->publish(*msg);
        }
    }

    void remoteCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        bool should_publish = false;
        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            last_remote_cmd_ = *msg;
            last_remote_stamp_ = this->now();
            has_remote_ = true;
            if (active_source_ == "remote") {
                source_timeout_active_ = false;
                should_publish = true;
            }
        }

        if (should_publish) {
            output_pub_->publish(*msg);
        }
    }

    void checkSourceTimeout() {
        if (source_timeout_sec_ <= 0.0) {
            return;
        }

        geometry_msgs::msg::Twist cmd_to_publish;
        bool should_publish = false;
        bool should_warn_timeout = false;
        std::string switched_to;

        {
            std::lock_guard<std::mutex> lock(source_mutex_);
            const auto now = this->now();
            if (isSourceFreshLocked(active_source_, now)) {
                source_timeout_active_ = false;
                return;
            }

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
                "active_source 超时，自动回退到 %s", switched_to.c_str());
        } else if (should_warn_timeout) {
            RCLCPP_WARN(this->get_logger(),
                "active_source 超时（> %.3fs），执行刹停保护", source_timeout_sec_);
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

    mutable std::mutex source_mutex_;

    geometry_msgs::msg::Twist last_joy_cmd_;
    geometry_msgs::msg::Twist last_remote_cmd_;
    rclcpp::Time last_joy_stamp_;
    rclcpp::Time last_remote_stamp_;
    bool has_joy_ {false};
    bool has_remote_ {false};
    bool source_timeout_active_ {false};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr remote_sub_;
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

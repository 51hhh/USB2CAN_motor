/**
 * mcu_odom_bridge_node.cpp
 *
 * MCU里程计桥接节点：解析STM32固件的ODOM_STATE帧 → ROS2
 *
 * 输入: /dev/ttyUSB0 (UART 115200) — MCU二进制ODOM协议
 * 输出: /odom (nav_msgs/Odometry) — MCU 已融合里程计 (AS5048 xy + YIS130 yaw)
 *       /tf (odom → base_link)
 *
 * MCU固件已完成传感器融合，此节点仅做格式转换和TF发布。
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

// ODOM协议常量（与MCU固件同步）
static constexpr uint8_t FRAME_HEADER_0 = 0xAA;
static constexpr uint8_t FRAME_HEADER_1 = 0x55;
static constexpr uint8_t MSG_TYPE_STATE = 0x02;
static constexpr size_t STATE_FRAME_LEN = 45;  // 7(header) + 36(payload) + 2(crc)
static constexpr double PI = 3.14159265358979323846;
static constexpr uint16_t STATUS_YAW_VALID = 0x04;
static constexpr uint16_t STATUS_POS_VALID = 0x08;
static constexpr uint16_t STATUS_VEL_VALID = 0x10;
static constexpr uint16_t STATUS_ODOM_REQUIRED =
    STATUS_YAW_VALID | STATUS_POS_VALID | STATUS_VEL_VALID;

/**
 * CRC16-CCITT calculation (matches MCU firmware)
 * Polynomial: 0x1021, Init: 0xFFFF, No reflection
 */
static uint16_t odom_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}

#pragma pack(push, 1)
struct OdomStatePayload {
    uint64_t t_sample_us; // 时间戳 (微秒)
    float x;              // 位置 X (m)
    float y;              // 位置 Y (m)
    float yaw;            // 航向角 (弧度)
    float vx;             // 速度 X body (m/s)
    float vy;             // 速度 Y body (m/s)
    float wz;             // 角速度 Z (rad/s)
    uint16_t status_bits; // 状态位
    uint8_t quality;      // 质量等级
    uint8_t reserved;     // 保留
};
#pragma pack(pop)

static_assert(sizeof(OdomStatePayload) == 36, "ODOM_STATE payload must stay 36 bytes");

class McuOdomBridgeNode : public rclcpp::Node
{
public:
    McuOdomBridgeNode() : Node("mcu_odom_bridge_node"), serial_fd_(-1)
    {
        // 参数
        this->declare_parameter("serial_port", std::string("/dev/ttyUSB0"));
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("odom_topic", std::string("/odom"));
        this->declare_parameter("odom_frame", std::string("odom"));
        this->declare_parameter("base_frame", std::string("base_link"));
        this->declare_parameter("publish_tf", true);

        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();

        // 打开串口
        if (!openSerial()) {
            const std::string error = "无法打开串口: " + serial_port_;
            RCLCPP_FATAL(this->get_logger(), "%s", error.c_str());
            throw std::runtime_error(error);
        }
        RCLCPP_INFO(this->get_logger(), "串口已打开: %s @ %d", serial_port_.c_str(), baud_rate_);

        // 发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // 定时器：以最大速度读取串口
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),  // 1ms周期 = 1kHz轮询
            std::bind(&McuOdomBridgeNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "MCU里程计桥接启动: %s → %s",
                    serial_port_.c_str(), odom_topic_.c_str());
    }

    ~McuOdomBridgeNode()
    {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
        }
    }

private:
    bool openSerial()
    {
        serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_fd_ < 0) {
            return false;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        // 设置波特率
        speed_t speed = B115200;
        switch (baud_rate_) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default: speed = B115200;
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // 8N1, 无流控
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;

        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);

        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;

        tty.c_cc[VTIME] = 0;
        tty.c_cc[VMIN] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            close(serial_fd_);
            serial_fd_ = -1;
            return false;
        }

        return true;
    }

    void timerCallback()
    {
        if (serial_fd_ < 0) {
            return;
        }

        // 读取可用数据
        uint8_t chunk[256];
        ssize_t n = read(serial_fd_, chunk, sizeof(chunk));
        if (n <= 0) {
            return;
        }

        // 追加到缓冲区
        buffer_.insert(buffer_.end(), chunk, chunk + n);

        // 解析帧
        while (buffer_.size() >= STATE_FRAME_LEN) {
            // 查找帧头
            if (buffer_[0] != FRAME_HEADER_0 || buffer_[1] != FRAME_HEADER_1) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            // 检查消息类型（跳过version字段）
            if (buffer_[3] != MSG_TYPE_STATE) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            // 检查payload长度
            uint16_t payload_len = buffer_[5] | (buffer_[6] << 8);
            if (payload_len != 36) {
                buffer_.erase(buffer_.begin());
                continue;
            }

            // 验证CRC16（覆盖 buffer_[2..42]，共41字节）
            uint16_t crc_calc = odom_crc16(buffer_.data() + 2, 41);
            uint16_t crc_recv = buffer_[43] | (buffer_[44] << 8);
            if (crc_calc != crc_recv) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "CRC校验失败: calc=0x%04X recv=0x%04X, 丢弃帧", crc_calc, crc_recv);
                buffer_.erase(buffer_.begin());
                continue;
            }

            // 提取payload（跳过7字节头部: header(2)+ver(1)+type(1)+seq(1)+len(2)）
            // payload在offset 7，长度36，然后是crc(2)
            OdomStatePayload payload;
            std::memcpy(&payload, buffer_.data() + 7, sizeof(payload));

            // 发布里程计
            publishOdom(payload);

            // 移除已处理帧
            buffer_.erase(buffer_.begin(), buffer_.begin() + STATE_FRAME_LEN);
        }
    }

    void publishOdom(const OdomStatePayload &payload)
    {
        if (!isPayloadValid(payload)) {
            invalid_frame_count_++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "ODOM_STATE无效，丢弃: status=0x%04X quality=%u invalid=%lu",
                payload.status_bits, payload.quality, invalid_frame_count_);
            return;
        }

        auto now = this->now();

        // 填充Odometry消息
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = now;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        // 位置
        odom.pose.pose.position.x = payload.x;
        odom.pose.pose.position.y = payload.y;
        odom.pose.pose.position.z = 0.0;

        // 姿态（yaw已经是弧度）
        tf2::Quaternion q;
        q.setRPY(0, 0, payload.yaw);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // 速度（body frame）
        odom.twist.twist.linear.x = payload.vx;
        odom.twist.twist.linear.y = payload.vy;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = payload.wz;

        const double covariance_scale = payload.quality >= 2 ? 1.0 : 10.0;

        // 协方差（基于测试结果，低质量帧放大协方差）
        // 位置协方差：σ_x=0.021mm, σ_y=0.020mm
        odom.pose.covariance[0] = 0.000000441 * covariance_scale;   // x (0.021mm)^2
        odom.pose.covariance[7] = 0.000000400 * covariance_scale;   // y (0.020mm)^2
        odom.pose.covariance[35] = 0.000000123 * covariance_scale;  // yaw (0.020°)^2

        // 速度协方差：实测 ±0.025 m/s
        odom.twist.covariance[0] = 0.000625 * covariance_scale;     // vx
        odom.twist.covariance[7] = 0.000625 * covariance_scale;     // vy
        odom.twist.covariance[35] = 0.000036 * covariance_scale;    // wz (0.006 rad/s)^2

        odom_pub_->publish(odom);

        // 发布TF
        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = now;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = payload.x;
            tf.transform.translation.y = payload.y;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_->sendTransform(tf);
        }

        // 统计
        frame_count_++;
        if (frame_count_ % 200 == 0) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "收到 %lu 帧 | x=%.3f y=%.3f yaw=%.3f° vx=%.3f vy=%.3f | quality=%u",
                frame_count_, payload.x, payload.y, payload.yaw * 180.0 / PI,
                payload.vx, payload.vy, payload.quality);
        }
    }

    bool isPayloadValid(const OdomStatePayload &payload) const
    {
        return payload.quality > 0 &&
            (payload.status_bits & STATUS_ODOM_REQUIRED) == STATUS_ODOM_REQUIRED;
    }

    int serial_fd_;
    std::vector<uint8_t> buffer_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::string serial_port_;
    int baud_rate_;
    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    bool publish_tf_;

    size_t frame_count_ = 0;
    size_t invalid_frame_count_ = 0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<McuOdomBridgeNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("mcu_odom_bridge_node"), "%s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

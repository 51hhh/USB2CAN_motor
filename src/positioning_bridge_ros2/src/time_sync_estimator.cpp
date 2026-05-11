#include "positioning_bridge_ros2/time_sync_estimator.hpp"

namespace positioning_bridge_ros2 {

void TimeSyncEstimator::reset()
{
  offset_us_ = 0.0;
  rtt_us_ = 0.0;
  sample_count_ = 0;
}

void TimeSyncEstimator::update(uint64_t host_request_send_us, uint64_t host_response_recv_us, uint64_t mcu_time_us)
{
  if (host_response_recv_us <= host_request_send_us) {
    return;
  }

  const double sample_rtt_us = static_cast<double>(host_response_recv_us - host_request_send_us);
  const double host_mid_us = static_cast<double>(host_request_send_us) + sample_rtt_us * 0.5;
  const double sample_offset_us = host_mid_us - static_cast<double>(mcu_time_us);

  if (sample_count_ == 0) {
    offset_us_ = sample_offset_us;
    rtt_us_ = sample_rtt_us;
  } else {
    constexpr double alpha = 0.2;
    offset_us_ = (1.0 - alpha) * offset_us_ + alpha * sample_offset_us;
    rtt_us_ = (1.0 - alpha) * rtt_us_ + alpha * sample_rtt_us;
  }

  ++sample_count_;
}

std::optional<rclcpp::Time> TimeSyncEstimator::toRosTime(uint64_t mcu_time_us) const
{
  if (!locked()) {
    return std::nullopt;
  }

  const int64_t ros_time_ns = static_cast<int64_t>((static_cast<double>(mcu_time_us) + offset_us_) * 1000.0);
  return rclcpp::Time(ros_time_ns, RCL_ROS_TIME);
}

}  // namespace positioning_bridge_ros2

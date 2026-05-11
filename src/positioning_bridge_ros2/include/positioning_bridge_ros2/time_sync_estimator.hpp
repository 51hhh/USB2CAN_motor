#ifndef POSITIONING_BRIDGE_ROS2__TIME_SYNC_ESTIMATOR_HPP_
#define POSITIONING_BRIDGE_ROS2__TIME_SYNC_ESTIMATOR_HPP_

#include <cstdint>
#include <optional>

#include <rclcpp/rclcpp.hpp>

namespace positioning_bridge_ros2 {

class TimeSyncEstimator {
public:
  void reset();

  void update(uint64_t host_request_send_us, uint64_t host_response_recv_us, uint64_t mcu_time_us);
  bool locked() const { return sample_count_ >= 3; }
  std::size_t sampleCount() const { return sample_count_; }
  double offsetUs() const { return offset_us_; }
  double rttUs() const { return rtt_us_; }

  std::optional<rclcpp::Time> toRosTime(uint64_t mcu_time_us) const;

private:
  double offset_us_ {0.0};
  double rtt_us_ {0.0};
  std::size_t sample_count_ {0};
};

}  // namespace positioning_bridge_ros2

#endif  // POSITIONING_BRIDGE_ROS2__TIME_SYNC_ESTIMATOR_HPP_

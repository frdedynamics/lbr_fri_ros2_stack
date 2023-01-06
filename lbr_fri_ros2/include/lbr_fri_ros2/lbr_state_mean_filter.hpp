#ifndef LBR_FRI_ROS2__LBR_STATE_MEAN_FILTER_HPP_
#define LBR_FRI_ROS2__LBR_STATE_MEAN_FILTER_HPP_

#include <memory>

#include "filters/filter_base.hpp"
#include "filters/realtime_circular_buffer.hpp"

#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/lbr.hpp"

namespace lbr_fri_ros2 {

class LBRStateMeanFilter : public filters::FilterBase<lbr_fri_msgs::msg::LBRState> {

public:
  LBRStateMeanFilter();

  bool configure() override;
  bool update(const lbr_fri_msgs::msg::LBRState &lbr_state_in,
              lbr_fri_msgs::msg::LBRState &lbr_state_out) override;

protected:
  void zero_non_steady_values_(lbr_fri_msgs::msg::LBRState &lbr_state);
  void transfer_steady_values_(const lbr_fri_msgs::msg::LBRState &lbr_state_in,
                               lbr_fri_msgs::msg::LBRState &lbr_state_out);

  std::unique_ptr<filters::RealtimeCircularBuffer<lbr_fri_msgs::msg::LBRState>> lbr_state_buffer_;
  uint32_t last_updated_row_;
  lbr_fri_msgs::msg::LBRState temp_;
  uint32_t number_of_observations_;
};
} // end of namespace lbr_fri_ros2
#endif // LBR_FRI_ROS2__LBR_STATE_MEAN_FILTER_HPP_

#include "lbr_fri_ros2/lbr_state_mean_filter.hpp"

namespace lbr_fri_ros2 {

LBRStateMeanFilter::LBRStateMeanFilter() : number_of_observations_(0) {}

bool LBRStateMeanFilter::configure() {
  if (!FilterBase<lbr_fri_msgs::msg::LBRState>::getParam("number_of_observations",
                                                         number_of_observations_)) {
    RCLCPP_ERROR(logging_interface_->get_logger(),
                 "LBRStateMeanFilter did not find param number_of_observations");
    return false;
  }

  lbr_state_buffer_.reset(new filters::RealtimeCircularBuffer<lbr_fri_msgs::msg::LBRState>(
      number_of_observations_, temp_));

  return true;
}

bool LBRStateMeanFilter::update(const lbr_fri_msgs::msg::LBRState &lbr_state_in,
                                lbr_fri_msgs::msg::LBRState &lbr_state_out) {
  // update active row
  if (last_updated_row_ >= number_of_observations_ - 1) {
    last_updated_row_ = 0;
  } else {
    ++last_updated_row_;
  }

  lbr_state_buffer_->push_back(lbr_state_in);

  size_t length = lbr_state_buffer_->size();

  // set non steady data zero
  zero_non_steady_values_(lbr_state_out);

  for (size_t i = 0; i < length; ++i) {
    for (uint8_t j = 0; j < LBR::JOINT_DOF; ++j) {
      lbr_state_out.measured_joint_position[j] +=
          lbr_state_buffer_->at(i).measured_joint_position[j];
      lbr_state_out.commanded_joint_position[j] +=
          lbr_state_buffer_->at(i).commanded_joint_position[j];
      lbr_state_out.measured_torque[j] += lbr_state_buffer_->at(i).measured_torque[j];
      lbr_state_out.commanded_torque[j] += lbr_state_buffer_->at(i).commanded_torque[j];
      lbr_state_out.external_torque[j] += lbr_state_buffer_->at(i).external_torque[j];
    }
  }

  for (uint8_t j = 0; j < LBR::JOINT_DOF; ++j) {
    lbr_state_out.measured_joint_position[j] /= length;
    lbr_state_out.commanded_joint_position[j] /= length;
    lbr_state_out.measured_torque[j] /= length;
    lbr_state_out.commanded_torque[j] /= length;
    lbr_state_out.external_torque[j] /= length;
  }

  // copy steady data
  transfer_steady_values_(lbr_state_in, lbr_state_out);

  return true;
}

void LBRStateMeanFilter::zero_non_steady_values_(lbr_fri_msgs::msg::LBRState &lbr_state) {
  lbr_state.measured_joint_position.assign(LBR::JOINT_DOF, 0.);
  lbr_state.commanded_joint_position.assign(LBR::JOINT_DOF, 0.);
  lbr_state.measured_torque.assign(LBR::JOINT_DOF, 0.);
  lbr_state.commanded_torque.assign(LBR::JOINT_DOF, 0.);
  lbr_state.external_torque.assign(LBR::JOINT_DOF, 0.);
}
void LBRStateMeanFilter::transfer_steady_values_(const lbr_fri_msgs::msg::LBRState &lbr_state_in,
                                                 lbr_fri_msgs::msg::LBRState &lbr_state_out) {
  lbr_state_out.client_command_mode = lbr_state_in.client_command_mode;
  lbr_state_out.sample_time = lbr_state_in.sample_time;
  lbr_state_out.session_state = lbr_state_in.session_state;
  lbr_state_out.connection_quality = lbr_state_in.connection_quality;
  lbr_state_out.safety_state = lbr_state_in.safety_state;
  lbr_state_out.operation_mode = lbr_state_in.operation_mode;
  lbr_state_out.drive_state = lbr_state_in.drive_state;
  lbr_state_out.client_command_mode = lbr_state_in.client_command_mode;
  lbr_state_out.overlay_type = lbr_state_in.overlay_type;
  lbr_state_out.control_mode = lbr_state_in.control_mode;

  lbr_state_out.time_stamp_sec = lbr_state_in.time_stamp_sec;
  lbr_state_out.time_stamp_nano_sec = lbr_state_in.time_stamp_nano_sec;

  lbr_state_out.ipo_joint_position.assign(lbr_state_in.ipo_joint_position.begin(),
                                          lbr_state_in.ipo_joint_position.end());
  lbr_state_out.tracking_performance = lbr_state_in.tracking_performance;
}
} // end of namespace lbr_fri_ros2

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "lbr_fri_msgs/msg/lbr_state.hpp"
#include "lbr_fri_ros2/lbr_state_mean_filter.hpp"

namespace lbr_fri_ros2 {
class LBRStateMeanFilterNode : public rclcpp::Node {
public:
  LBRStateMeanFilterNode(const std::string &node_name = "lbr_state_filter_node")
      : rclcpp::Node(node_name) {
    this->declare_parameter("number_of_observations", 100);

    lbr_state_mean_filter_ = std::make_unique<LBRStateMeanFilter>();
    lbr_state_mean_filter_->configure("", "mean", get_node_logging_interface(),
                                      get_node_parameters_interface());

    lbr_state_sub_ = create_subscription<lbr_fri_msgs::msg::LBRState>(
        "/lbr_state", rclcpp::SystemDefaultsQoS(),
        std::bind(&LBRStateMeanFilterNode::lbr_state_sub_cb_, this, std::placeholders::_1));

    lbr_filtered_state_pub_ = create_publisher<lbr_fri_msgs::msg::LBRState>(
        "/lbr_state/smooth", rclcpp::SystemDefaultsQoS());
  }

protected:
  void lbr_state_sub_cb_(const lbr_fri_msgs::msg::LBRState::SharedPtr lbr_state) {
    lbr_fri_msgs::msg::LBRState mean_filtered_lbr_state_;
    lbr_state_mean_filter_->update(*lbr_state, mean_filtered_lbr_state_);

    // RCLCPP_INFO(get_logger(), "%f, %f, %f, %f, %f, %f, %f",
    // lbr_state->measured_joint_position[0],
    //             lbr_state->measured_joint_position[2], lbr_state->measured_joint_position[2],
    //             lbr_state->measured_joint_position[3], lbr_state->measured_joint_position[4],
    //             lbr_state->measured_joint_position[5], lbr_state->measured_joint_position[6]);

    // RCLCPP_INFO(get_logger(), "%f, %f, %f, %f, %f, %f, %f", filtered[0], filtered[2],
    // filtered[2],
    //             filtered[3], filtered[4], filtered[5], filtered[6]);

    lbr_filtered_state_pub_->publish(mean_filtered_lbr_state_);
  }

  std::unique_ptr<filters::FilterBase<lbr_fri_msgs::msg::LBRState>> lbr_state_mean_filter_;

  rclcpp::Subscription<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_state_sub_;
  rclcpp::Publisher<lbr_fri_msgs::msg::LBRState>::SharedPtr lbr_filtered_state_pub_;
};
} // namespace lbr_fri_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lbr_fri_ros2::LBRStateMeanFilterNode>());
  rclcpp::shutdown();
  return 0;
}

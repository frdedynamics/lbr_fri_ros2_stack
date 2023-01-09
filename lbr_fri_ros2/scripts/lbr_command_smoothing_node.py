#!/usr/bin/python3
from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node

from lbr_fri_msgs.msg import LBRCommand


class LBRCommandSmoothingNode(Node):
    def __init__(self) -> None:
        super().__init__("lbr_command_smoothing_node")

        # Declare and get parameters
        self.declare_parameter("max_window_length", 20)
        self.max_window_length_ = int(self.get_parameter("max_window_length").value)
        self.window_ = []

        # Set filter method
        self.filter_ = self.filter_mean_
        # self.filter_ = self.filter_median_

        # Create publishers/subscribers
        self.lbr_command_smooth_publisher_ = self.create_publisher(
            LBRCommand, "/lbr_command/smooth/in", 1
        )
        self.lbr_command_subscription_ = self.create_subscription(
            LBRCommand,
            "/lbr_command/smooth/out",
            self.lbr_command_callback_,
            1,
        )

    def update_window_(self, command: LBRCommand) -> None:
        self.window_.append(command)
        if len(self.window_) > self.max_window_length_:
            self.window_.pop(0)

    def get_most_recent_command_(self) -> LBRCommand:
        return deepcopy(self.window_[-1])

    def filter_mean_(self) -> LBRCommand:
        smooth_command = self.get_most_recent_command_()
        smooth_command.joint_position = np.mean(
            [m.measured_joint_position for m in self.window_], axis=0
        ).tolist()
        return smooth_command

    def filter_median_(self) -> LBRCommand:
        smooth_command = self.get_most_recent_command_()
        smooth_command.joint_position = np.median(
            [m.measured_joint_position for m in self.window_], axis=0
        ).tolist()
        return smooth_command

    def lbr_command_callback_(self, command: LBRCommand):
        self.update_window_(command)
        self.lbr_command_smooth_publisher_.publish(self.filter_())


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LBRCommandSmoothingNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()

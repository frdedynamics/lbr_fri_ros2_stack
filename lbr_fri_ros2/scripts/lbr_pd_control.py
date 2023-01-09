#!/usr/bin/python3
import numpy as np
import rclpy
from rclpy import qos
from rclpy.node import Node

from lbr_fri_msgs.msg import LBRCommand, LBRState

class LBRPDControlNode(Node):

    def __init__(self) -> None:
        super().__init__("lbr_pd_control_node")

        # Declare and get parameters
        self.declare_parameter("sampling_frequency", 100)
        self.declare_parameter("K", 1.)
        self.declare_parameter("D", 0.)
        freq = int(self.get_parameter("sampling_frequency").value)
        self.dt = 1./float(freq)        
        self.K = float(self.get_parameter("K").value)
        self.D = float(self.get_parameter("D").value)
        
        # Create publishers/subscribers
        self.lbr_command_publisher = self.create_publisher(
            LBRCommand, "/lbr_command/out", 1
        )
        self.lbr_command_subscription = self.create_subscription(
            LBRCommand,
            "/lbr_command/in",
            self.lbr_command_callback,
            1,
        )

        self.q_curr = None
        self.dq_curr = None
        self.lbr_state_sub_ = self.create_subscription(
            LBRState, "/lbr_state", self.lbr_state_callback, qos.qos_profile_system_default
        )

        self.q_goal = None
        self.create_timer(self.dt, self.timer_callback)

    def lbr_state_callback(self, msg):
        q_now = np.array(msg.measured_joint_position)
        if self.q_curr is not None:
            dt = float(msg.sample_time)
            self.dq_curr = (q_now - self.q_curr)/dt
        self.q_curr = q_now.copy()

    def lbr_command_callback(self, msg):
        self.q_goal = np.array(msg.joint_position)

    def timer_callback(self):
        if self.dq_curr is None: return
        if self.q_goal is None: return

        q_curr = self.q_curr.copy()
        dq_curr = self.dq_curr.copy()

        q_goal = np.array(msg.joint_position)

        dq_cmd = self.K*(q_goal - q_curr) - self.D*dq_curr

        q_cmd = q_curr + self.dt*dq_cmd

        command = LBRCommand(joint_position=q_cmd.tolist())
        self.lbr_command_publisher.publish(command)
        
def main():
    rclpy.init()
    rclpy.spin(LBRPDControlNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

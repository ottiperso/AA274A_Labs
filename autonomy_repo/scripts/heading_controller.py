#!/usr/bin/env python3
import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController):
    
    def __init__(self):
        super().__init__('heading_controller')
        self.declare_parameter("kp", 2.0)   # gain
        
    @property
    def kp(self) -> float:
        """ Get real-time parameter value of kp gain

        Returns:
            float: latest parameter value of kp gain
        """
        return self.get_parameter("kp").value
        
    def compute_control_with_goal(self, curr_state: TurtleBotState, des_state: TurtleBotState) -> TurtleBotControl:
        heading_err = wrap_angle(des_state.theta - curr_state.theta)
        w = heading_err * self.kp
        msg = TurtleBotControl()
        msg.omega = w
        return msg
    
if __name__ == '__main__':
    rclpy.init()
    heading_ctrl = HeadingController()
    rclpy.spin(heading_ctrl)
    rclpy.shutdown()
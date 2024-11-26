#!/usr/bin/env python 

"""
Test PD+gravity tracking sinusoidal joint trajectory for Panda robot (7Dof).
"""


import numpy as np
import rclpy
from rclpy.node import Node
import rclpy.duration

import rclpy.time
from std_msgs.msg import String
from linear_feedback_controller_msgs.msg import Sensor, Control

import pinocchio as pin

Ndof = 7

AMPLITUDE = np.zeros(Ndof)
AMPLITUDE[0] = 0.1
 
PERIOD = 4.0*np.ones(Ndof)


def compute_sinusoid_q_delta_reference(amp, period, dt):

    """
    Compute a delta joint sinusoid reference with zero initial velocity. 

    a and c obtained for each joint using constraints:
    delta_q(t=0.0) = 0
    delta_q(t=period/2) = amp
    """

    w = 2*np.pi/period
    a = - amp
    c = amp
    
    delta_q = c + a*np.cos(w*dt)
    dq = -w*a*np.sin(w*dt)

    return delta_q, dq


class PDPlusController(Node):

    def __init__(self):
        super().__init__('pd_plus_demo')
        self.publisher_control_ = self.create_publisher(
            Control, "/linear_feedback_controller/desired_control", qos_profile=1
        )
        self.subscriber_sensor_ = self.create_subscription(
            Sensor, "/linear_feedback_controller/sensor_state", self.sensor_callback, qos_profile=5
        )
        # self.subscriber_robot_model_ = self.create_subscription(
        #     Sensor, "/robot_description", self.sensor_callback, qos_profile=5
        # )
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # Shared between subscriber and timer callbacks
        self.t0 = None
        self.q0 = None 
        self.q_m = None 
        self.dq_m = None 

        # PD gains
        # TODO: read from ROS params
        self.Kp = np.array([60.0, 60.0, 60.0, 60.0, 20.0, 10.0, 5.0])
        self.Kd = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 2.0, 1.0])

    def robot_description_callback(self, msg):
        # TODO:
        # - read model
        # - build with pinocchio
        # - reduce model
        pass

    def sensor_callback(self, msg: Sensor):
        self.get_logger().warn('sensor_callback')

        t_now = self.get_clock().now()

        # TODO: get q and dq from sensor
        self.q_m = np.ones(Ndof)
        self.dq_m = np.ones(Ndof)
    
        if not self.sensor_received:
            self.t0 = t_now
            self.q0 = self.q_m

    def timer_callback(self):
        self.get_logger().warn('timer_callback')

        t_now = self.get_clock().now()
        if self.t0 is None:
            self.get_logger().warn('Not publishing: did not receive first Sensor msg.')
            return
        
        dt_sec = (t_now - self.t0).nanoseconds / 1e9
        delta_q, dq_ref = compute_sinusoid_q_delta_reference(AMPLITUDE, PERIOD, dt_sec)
        q_ref = self.q0 + delta_q

        # TODO: get robot model from somewhere
        tau_g = np.zeros(Ndof)
        tau = tau_g - self.Kp * (self.q_m - q_ref) + - self.Kd * (self.dq_m - dq_ref)

        K_ricattti = np.zeros((Ndof,2*Ndof))

        sensor = Sensor(
            base_pose=np.ones(7),
            base_twist=np.ones(6),
            joint_state=self.q_m,
            contacts=[],
        )

        ctrl_msg = Control(
            feedback_gain=K_ricattti,
            feedforward=tau,
            initial_state=sensor,
        )
        self.publisher_control_.publish(ctrl_msg)
        

def main(args=None):
    rclpy.init(args=args)
    pd_plus_demo = PDPlusController()

    rclpy.spin(pd_plus_demo)

    pd_plus_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
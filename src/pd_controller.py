#! /usr/bin/env python

"""
TODO: this node is a Talos specific ROS1 implementation of a PD+ controller
used to test lfc. Needs to be migrated to ROS2
"""

from math import sin, pi
from threading import Lock
from copy import deepcopy
import numpy as np
import rospy
from std_msgs.msg import MultiArrayDimension
from std_msgs.msg import Float64MultiArray
from linear_feedback_controller_msgs.msg import Sensor, Control


def numpy_to_multiarray(np_array):
    multiarray = Float64MultiArray()

    label = "row"
    size = np_array.shape[0]
    stride = np_array.shape[0]
    if len(np_array.shape) == 2:
        stride *= np_array.shape[1]
    dim0 = MultiArrayDimension(label, size, stride)

    label = "col"
    if len(np_array.shape) == 2:
        size = np_array.shape[1]
    else:
        size = 1
    stride = size
    dim1 = MultiArrayDimension(label, size, stride)

    multiarray.layout.dim = [dim0, dim1]
    multiarray.data = np_array.reshape([1, -1])[0].tolist()
    return multiarray


class PDController:
    def __init__(self):
        self.ctrl_pub = rospy.Publisher(
            "/linear_feedback_controller/desired_control",
            Control,
            queue_size=1,
        )
        self.sensor_sub = rospy.Subscriber(
            "/linear_feedback_controller/sensor_state",
            Sensor,
            self.sensor_callback,
        )
        param_name = rospy.search_param("frequency")
        if param_name is not None:
            self.frequency = rospy.get_param(param_name)
        else:
            self.frequency = 500  # Hz
        self.rate = rospy.Rate(self.frequency)  # 10hz
        self.sensor_mutex = Lock()
        self.control = Control()
        self.sensor_copy = Sensor()
        self.sensor = Sensor()
        self.p_arm_gain = 100.0
        self.d_arm_gain = 8.0
        self.p_torso_gain = 500.0
        self.d_torso_gain = 20.0
        self.p_leg_gain = 800.0
        self.d_leg_gain = 35.0
        self.feedback_gain = None

        self.start = False

    def sensor_callback(self, msg):
        rospy.loginfo_throttle(0.1, "Received msg")
        self.sensor_mutex.acquire()
        self.sensor_copy = deepcopy(msg)
        # Init from first sensor
        if not self.start:
            self.ijs = deepcopy(msg.joint_state)
            self.init_time = rospy.Time.now()
            self.start = True

        self.sensor_mutex.release()

    def build_feedback_gain(self):
        if self.feedback_gain is None:
            nb_joints = len(self.sensor.joint_state.name)
            self.feedback_gain = np.zeros((nb_joints, 2 * (nb_joints + 6)))
            for i, name in enumerate(self.sensor.joint_state.name):
                if "torso" in name:
                    self.feedback_gain[i, 6 + i] = self.p_torso_gain
                    self.feedback_gain[i, 6 + nb_joints + 6 + i] = self.d_torso_gain
                elif "arm" in name:
                    self.feedback_gain[i, 6 + i] = self.p_arm_gain
                    self.feedback_gain[i, 6 + nb_joints + 6 + i] = self.d_arm_gain
                else:
                    self.feedback_gain[i, 6 + i] = self.p_leg_gain
                    self.feedback_gain[i, 6 + nb_joints + 6 + i] = self.d_leg_gain
        return self.feedback_gain

    def run(self):
        while not rospy.is_shutdown() and not self.start:
            self.rate.sleep()

        t = 0
        while not rospy.is_shutdown():
            self.sensor_mutex.acquire()
            self.sensor = deepcopy(self.sensor_copy)
            self.sensor_mutex.release()
            self.control.header.stamp = rospy.Time.now()
            self.control.feedback_gain = numpy_to_multiarray(self.build_feedback_gain())
            self.control.feedforward = numpy_to_multiarray(np.array(self.ijs.effort))
            self.control.initial_state = deepcopy(self.sensor)

            # create squatting motion
            omega = 2 * pi * (1.0 / 60.0)  # 2 * pi * f
            a = 0.1
            mvt = a * sin(omega * t)
            opp = -0.5 * mvt

            self.control.initial_state.joint_state.position = [
                self.ijs.position[0],  # leg_left_1_joint
                self.ijs.position[1],  # leg_left_2_joint
                self.ijs.position[2] + opp,  # leg_left_3_joint
                self.ijs.position[3] + mvt,  # leg_left_4_joint
                self.ijs.position[4] + opp,  # leg_left_5_joint
                self.ijs.position[5],  # leg_left_6_joint
                self.ijs.position[6],  # leg_right_1_joint
                self.ijs.position[7],  # leg_right_2_joint
                self.ijs.position[8] + opp,  # leg_right_3_joint
                self.ijs.position[9] + mvt,  # leg_right_4_joint
                self.ijs.position[10] + opp,  # leg_right_5_joint
                self.ijs.position[11],  # leg_right_6_joint
                self.ijs.position[12],  # torso_1_joint
                self.ijs.position[13],  # torso_2_joint
            ]
            self.ctrl_pub.publish(self.control)
            self.rate.sleep()
            t += 1.0 / self.frequency


if __name__ == "__main__":
    try:
        rospy.init_node("talker", anonymous=True)
        ctrl = PDController()
        ctrl.run()
    except rospy.ROSInterruptException:
        pass

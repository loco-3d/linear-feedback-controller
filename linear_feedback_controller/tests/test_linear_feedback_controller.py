#!/usr/bin/env python
PKG = "linear_feedback_controller"

# standard import
import sys
import numpy as np

np.set_printoptions(threshold=sys.maxsize)
import subprocess
import time
import unittest

# ros import
import rostest
import rospy
from sensor_msgs.msg import JointState
from linear_feedback_controller_msgs.msg import Sensor

# sys.stderr.write
class CheckInitPause(object):
    def __init__(self):
        self.default_joint_position = np.array(
            [
                2.295723385640216e-05,
                -0.031217533851476284,
                -0.37826160292121835,
                0.6686476948163876,
                -0.2844027468792062,
                0.030968738530664102,
                -2.114218662552792e-05,
                -0.030373884099596484,
                -0.37796303137040127,
                0.6682617068999609,
                -0.2843337254390096,
                0.030534431559915098,
                -4.3216261365292814e-06,
                0.05085408038491371,
                0.33942869601693704,
                0.2368586059375524,
                -0.6005350100183053,
                -1.4473303842192178,
                0.004032533513432849,
                -0.025948509565568684,
                0.022324815530329157,
                0.00037752504530725553,
                -0.33944565421923156,
                -0.23626948314552995,
                0.6015602319399317,
                -1.446824097911113,
                -0.003078895633172175,
                0.029936186866785366,
                0.023418201451482348,
                0.00026234916458073364,
                0.0005169013384967594,
                -0.0015516616196827751,
            ]
        )

    def check(self):
        msg = rospy.wait_for_message(
            "/simulator/joint_states", JointState, timeout=None
        )
        return np.allclose(
            msg.position, self.default_joint_position, atol=1e-1, rtol=0.0
        )


## A sample python unit test
class TestLinearFeedbackController(unittest.TestCase):
    lfc_process = None

    @classmethod
    def start_and_stop_default_controller(cls):
        default_ctrl_process = subprocess.Popen(
            [
                "roslaunch",
                "talos_controller_configuration",
                "default_controllers.launch",
            ]
        )
        # Let the time of the default controller to settle
        check_init_pose = CheckInitPause()
        while not check_init_pose.check():
            time.sleep(0.001)
        default_ctrl_process.terminate()
        default_ctrl_process.wait()

    @classmethod
    def load_and_start_linear_feedback_controller(cls):
        cls.lfc_process = subprocess.Popen(
            [
                "roslaunch",
                PKG,
                "talos_linear_feedback_controller.launch",
            ]
        )

    @classmethod
    def setUpClass(cls):
        rospy.init_node("LinearFeedbackController")
        time.sleep(2)
        cls.start_and_stop_default_controller()
        cls.load_and_start_linear_feedback_controller()

    def setUp(self):
        pass
    
    @classmethod
    def tearDownClass(cls):
        cls.lfc_process.terminate()
        cls.lfc_process.wait()

    def test_load_params(
        self,
    ):
        self.assertEqual(
            rospy.get_param("linear_feedback_controller/robot_has_free_flyer"),
            True,
        )
        self.assertEqual(
            rospy.get_param("linear_feedback_controller/moving_joint_names"),
            [
                "leg_right_1_joint",
                "leg_right_2_joint",
                "leg_right_3_joint",
                "leg_right_4_joint",
                "leg_right_5_joint",
                "leg_right_6_joint",
                "torso_1_joint",
                "torso_2_joint",
                "leg_left_1_joint",
                "leg_left_2_joint",
                "leg_left_3_joint",
                "leg_left_4_joint",
                "leg_left_5_joint",
                "leg_left_6_joint",
                "root_joint",
            ],
        )

    ## test 1 == 1
    def test_moving_joint_names_from_controller(
        self,
    ):
        msg = rospy.wait_for_message(
            "linear_feedback_controller/sensor_state", Sensor, timeout=None
        )
        self.assertEqual(
            msg.joint_state.name,
            [
                "root_joint",
                "leg_left_1_joint",
                "leg_left_2_joint",
                "leg_left_3_joint",
                "leg_left_4_joint",
                "leg_left_5_joint",
                "leg_left_6_joint",
                "leg_right_1_joint",
                "leg_right_2_joint",
                "leg_right_3_joint",
                "leg_right_4_joint",
                "leg_right_5_joint",
                "leg_right_6_joint",
                "torso_1_joint",
                "torso_2_joint",
            ],
        )


if __name__ == "__main__":
    rostest.rosrun(PKG, "test_" + PKG, TestLinearFeedbackController)

#!/usr/bin/env python

"""
Test PD+gravity tracking sinusoidal joint trajectory for Panda robot (7Dof).
"""
import numpy as np
import pinocchio as pin
import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
import rclpy.time

from std_msgs.msg import String

from linear_feedback_controller_msgs.msg import Sensor, Control
from linear_feedback_controller_msgs_py.numpy_conversions import (
    sensor_msg_to_numpy,
    control_numpy_to_msg,
)
import linear_feedback_controller_msgs_py.lfc_py_types as lfc_py_types


class PDPlusController(Node):
    def __init__(self):
        super().__init__("pd_plus_controller")

        # Moving joint names.
        self.declare_parameter("moving_joint_names", [""])
        # Amplitude of the sinus per joints.
        self.declare_parameter("amplitudes", [0.0])
        # Periods of the sinus per joints.
        self.declare_parameter("periods", [0.0])
        # P gains.
        self.declare_parameter("p_gains", [0.0])
        # D gains.
        self.declare_parameter("d_gains", [0.0])
        # Controller period.
        self.declare_parameter("controller_period", 0.01)
        # Do we use the Ricatti gains or not.
        self.declare_parameter("with_ricatti_gains", True)

        # Obtain the parameter value
        self.moving_joint_names = (
            self.get_parameter("moving_joint_names")
            .get_parameter_value()
            .string_array_value
        )
        self.amplitudes = np.array(
            self.get_parameter("amplitudes").get_parameter_value().double_array_value
        )
        self.periods = np.array(
            self.get_parameter("periods").get_parameter_value().double_array_value
        )
        self.p_gains = np.array(
            self.get_parameter("p_gains").get_parameter_value().double_array_value
        )
        self.d_gains = np.array(
            self.get_parameter("d_gains").get_parameter_value().double_array_value
        )
        self.controller_period = (
            self.get_parameter("controller_period").get_parameter_value().double_value
        )
        self.with_ricatti_gains = (
            self.get_parameter("with_ricatti_gains").get_parameter_value().bool_value
        )

        self.get_logger().info("\tmoving_joint_names = " + str(self.moving_joint_names))
        self.get_logger().info("\tamplitudes = " + str(self.amplitudes))
        self.get_logger().info("\tperiods = " + str(self.periods))
        self.get_logger().info("\tp_gains = " + str(self.p_gains))
        self.get_logger().info("\td_gains = " + str(self.d_gains))
        self.get_logger().info("\tcontroller_period = " + str(self.controller_period))

        # Load publisher and subscriber for LFC communication.
        self.publisher_control_ = self.create_publisher(
            Control,
            "/control",
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )
        self.subscriber_sensor_ = self.create_subscription(
            Sensor,
            "sensor",
            self.sensor_callback,
            qos_profile=QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
            ),
            qos_overriding_options=QoSOverridingOptions.with_default_policies(),
        )

        # Get the robot model.
        self.subscriber_robot_description_ = self.create_subscription(
            String,
            "/robot_description",
            self.robot_description_callback,
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            ),
        )
        self.timer = self.create_timer(self.controller_period, self.timer_callback)

        # Shared between subscriber and timer callbacks
        self.t0_sensor = None
        self.q0 = None
        self.q_m = None
        self.dq_m = None
        self.pin_model = None
        self.pin_data = None
        self.sensor_received = False

        self.get_logger().info("Controller stated.")

    def compute_sinusoid_q_delta_reference(self, amp, period, dt):
        w = np.zeros(period.shape)
        for i in range(period.shape[0]):
            if period[i] >= 1e-5:
                w[i] = 2 * np.pi / period[i]
        a = -amp
        c = amp
        delta_q = c + a * np.cos(w * dt)
        dq = -w * a * np.sin(w * dt)
        return delta_q, dq

    def robot_description_callback(self, msg: String):
        pin_model_complete = pin.buildModelFromXML(msg.data)
        locked_joint_names = [
            name
            for name in pin_model_complete.names
            if name not in self.moving_joint_names and name != "universe"
        ]
        locked_joint_ids = [
            pin_model_complete.getJointId(name) for name in locked_joint_names
        ]
        self.pin_model = pin.buildReducedModel(
            pin_model_complete,
            list_of_geom_models=[],
            list_of_joints_to_lock=locked_joint_ids,
            reference_configuration=np.zeros(pin_model_complete.nq),
        )[0]
        self.pin_data = self.pin_model.createData()
        self.get_logger().info("robot_description_callback")
        self.get_logger().info(f"pin_model.nq {self.pin_model.nq}")

    def sensor_callback(self, sensor_msg: Sensor):
        sensor: lfc_py_types.Sensor = sensor_msg_to_numpy(sensor_msg)
        self.q_m = sensor.joint_state.position
        self.dq_m = sensor.joint_state.velocity

        if not self.sensor_received:
            self.t0_sensor = self.get_clock().now()
            self.q0 = self.q_m
            self.sensor_received = True

    def timer_callback(self):
        t_now = self.get_clock().now()
        if self.pin_model is None:
            self.get_logger().info(
                "Waiting for the Robot model...", throttle_duration_sec=2.0
            )
            return
        if self.t0_sensor is None:
            self.get_logger().info(
                "Waiting for the first sensor message...", throttle_duration_sec=2.0
            )
            return

        dt_sec = (t_now - self.t0_sensor).nanoseconds / 1e9
        delta_q, dq_ref = self.compute_sinusoid_q_delta_reference(
            self.amplitudes, self.periods, dt_sec
        )
        q_ref = self.q0 + delta_q

        tau_g = pin.computeGeneralizedGravity(self.pin_model, self.pin_data, self.q_m)
        tau = (
            tau_g
            - self.p_gains * (self.q_m - q_ref)
            + -self.d_gains * (self.dq_m - dq_ref)
        ).reshape(self.pin_model.nv, 1)

        K_ricattti = np.zeros((self.pin_model.nv, 2 * self.pin_model.nv))
        if self.with_ricatti_gains:
            K_ricattti[:, : self.pin_model.nv] = np.diag(self.p_gains)
            K_ricattti[:, self.pin_model.nv :] = np.diag(self.d_gains)

        sensor = lfc_py_types.Sensor(
            base_pose=np.zeros(7),
            base_twist=np.zeros(6),
            joint_state=lfc_py_types.JointState(
                name=self.moving_joint_names,
                position=self.q_m,
                velocity=np.zeros((self.pin_model.nv, 1)),
                effort=np.zeros((self.pin_model.nv, 1)),
            ),
            contacts=[],
        )

        ctrl_msg = lfc_py_types.Control(
            feedback_gain=K_ricattti,
            feedforward=tau,
            initial_state=sensor,
        )
        self.publisher_control_.publish(control_numpy_to_msg(ctrl_msg))


def main(args=None):
    rclpy.init(args=args)
    pd_plus_controller = PDPlusController()

    try:
        rclpy.spin(pd_plus_controller)
    except KeyboardInterrupt:
        pass

    pd_plus_controller.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()

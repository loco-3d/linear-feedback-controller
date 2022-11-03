#ifndef ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP
#define ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

#include <ros_wbmpc_msgs/Sensor.h>
#include <ros_wbmpc_msgs/Control.h>

namespace ros_wbmpc_msgs {

namespace Eigen {

struct JointState {
  std::vector<std::string> name;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd effort;
}

struct Sensor {
  Eigen::Affine3d base_pose;
  Eigen::Matrix<double, 6, 1> base_twist;
  JointState joint_state;
};

struct Control {
  Eigen::MatrixXd feedback_gain;
  Eigen::VectorXd position;
  ros_wbmpc_msgs::Eigen::Sensor initial_state;
};
}  // namespace Eigen

template <class Derived>
void matrixMsgToEigen(const std_msgs::Float64MultiArray& m, Eigen::MatrixBase<Derived>& e) {
  assert(m.layout.dim.size() == 2);
  // resize the eigen object.
  e.resize(m.layout.dim[0].size, m.layout.dim[1].stride);
  // verify the input object.
  assert(m.layout.dim[0].stride == e.size());
  assert(m.layout.dim[0].size == e.rows());
  assert(m.layout.dim[1].stride == e.cols());
  assert(m.layout.dim[1].size == e.cols());

  int ii = 0;
  for (int i = 0; i < e.rows(); ++i) {
    for (int j = 0; j < e.cols(); ++j) {
      e.coeff(i, j) = m.data[ii++];
    }
  }
}

inline void jointStateMsgToEigen(const sensor_msgs/JointState& m, ros_wbmpc_msgs::Eigen::JointState& e) {
  e.name = m.name;
  e.position = Eigen::Map< Eigen::VectorXd >(m.position.data());
  e.velocity = Eigen::Map< Eigen::VectorXd >(m.velocity.data());
  e.effort = Eigen::Map< Eigen::VectorXd >(m.effort.data());
}

inline void controlMsgToEigen(const Control& m, ros_wbmpc_msgs::Eigen::Control& e) {
  matrixMsgToEigen(m.feedback_gain, e.feedback_gain);
  matrixMsgToEigen(m.feedforward, e.feedforward);
  sensorMsgToEigen(m.initial_state, e.initial_state);
}

inline void sensorMsgToEigen(const Sensor& m, ros_wbmpc_msgs::Eigen::Sensor e) {
    poseMsgToEigen(m.base_pose, e.base_pose);
    twistMsgToEigen(m.base_twist, e.base_twist);
    jointStateMsgToEigen(m.joint_state, e.joint_state);
}

inline void jointStateEigenToMsg(const sensor_msgs/JointState& m, ros_wbmpc_msgs::Eigen::JointState& e) {
  m.name = e.name;
  m.position = std::vector<double> vec(e.position.data(), e.position.data() + e.position.size());
  m.velocity = std::vector<double> vec(e.velocity.data(), e.velocity.data() + e.velocity.size());
  m.effort = std::vector<double> vec(e.effort.data(), e.effort.data() + e.effort.size());
}

inline void controlEigenToMsg(const ros_wbmpc_msgs::Eigen::Control& e, Control& m) {
  matrixEigenToMsg(m.feedback_gain, e.feedback_gain);
  matrixEigenToMsg(m.feedforward, e.feedforward);
  sensorEigenToMsg(m.initial_state, e.initial_state);
}

inline void sensorEigenToMsg(const ros_wbmpc_msgs::Eigen::Sensor e, Sensor& m) {
    poseEigenToMsg(e.base_pose, m.base_pose);
    twistEigenToMsg(e.base_twist, m.base_twist);
    jointStateEigenToMsg(e.joint_state, m.joint_state);
}

}  // namespace ros_wbmpc_msgs

#endif  // ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP

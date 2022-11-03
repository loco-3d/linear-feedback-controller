#ifndef ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP
#define ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

#include <linear_feedback_controller_msgs/Sensor.h>
#include <linear_feedback_controller_msgs/Control.h>

namespace linear_feedback_controller_msgs {

namespace Eigen {

struct JointState {
  std::vector<std::string> name;
  ::Eigen::VectorXd position;
  ::Eigen::VectorXd velocity;
  ::Eigen::VectorXd effort;
};

struct Sensor {
  ::Eigen::Affine3d base_pose;
  ::Eigen::Matrix<double, 6, 1> base_twist;
  JointState joint_state;
};

struct Control {
  ::Eigen::MatrixXd feedback_gain;
  ::Eigen::VectorXd feedforward;
  linear_feedback_controller_msgs::Eigen::Sensor initial_state;
};
}  // namespace Eigen

// Create an alias to use only one namespace here.
template <class Derived>
void matrixEigenToMsg(const ::Eigen::MatrixBase<Derived>& e, std_msgs::Float64MultiArray& m) {
  tf::matrixEigenToMsg(e, m);
}

template <class Derived>
void matrixMsgToEigen(const std_msgs::Float64MultiArray& m, ::Eigen::MatrixBase<Derived>& e) {
  assert(m.layout.dim.size() == 2);
  // resize the eigen object.
  e = ::Eigen::MatrixBase<Derived>::Zero(m.layout.dim[0].size, m.layout.dim[1].stride);
  // verify the input object.
  assert(m.layout.dim[0].stride == e.size());
  assert(m.layout.dim[0].size == e.rows());
  assert(m.layout.dim[1].stride == e.cols());
  assert(m.layout.dim[1].size == e.cols());

  int ii = 0;
  for (int i = 0; i < e.rows(); ++i) {
    for (int j = 0; j < e.cols(); ++j) {
      e(i, j) = m.data[ii++];
    }
  }
}

inline void jointStateMsgToEigen(const sensor_msgs::JointState& m,
                                 linear_feedback_controller_msgs::Eigen::JointState& e) {
  e.name = m.name;
  e.position = ::Eigen::Map<const ::Eigen::VectorXd>(m.position.data(), m.position.size());
  e.velocity = ::Eigen::Map<const ::Eigen::VectorXd>(m.velocity.data(), m.velocity.size());
  e.effort = ::Eigen::Map<const ::Eigen::VectorXd>(m.effort.data(), m.effort.size());
}

inline void sensorMsgToEigen(const Sensor& m, linear_feedback_controller_msgs::Eigen::Sensor e) {
  tf::poseMsgToEigen(m.base_pose, e.base_pose);
  tf::twistMsgToEigen(m.base_twist, e.base_twist);
  jointStateMsgToEigen(m.joint_state, e.joint_state);
}

inline void controlMsgToEigen(const Control& m, linear_feedback_controller_msgs::Eigen::Control& e) {
  matrixMsgToEigen(m.feedback_gain, e.feedback_gain);
  matrixMsgToEigen(m.feedforward, e.feedforward);
  sensorMsgToEigen(m.initial_state, e.initial_state);
}

inline void jointStateEigenToMsg(const linear_feedback_controller_msgs::Eigen::JointState& e,
                                 sensor_msgs::JointState& m) {
  m.name = e.name;
  m.position = std::vector<double>(e.position.data(), e.position.data() + e.position.size());
  m.velocity = std::vector<double>(e.velocity.data(), e.velocity.data() + e.velocity.size());
  m.effort = std::vector<double>(e.effort.data(), e.effort.data() + e.effort.size());
}

inline void sensorEigenToMsg(const linear_feedback_controller_msgs::Eigen::Sensor e, Sensor& m) {
  tf::poseEigenToMsg(e.base_pose, m.base_pose);
  tf::twistEigenToMsg(e.base_twist, m.base_twist);
  jointStateEigenToMsg(e.joint_state, m.joint_state);
}

inline void controlEigenToMsg(const linear_feedback_controller_msgs::Eigen::Control& e, Control& m) {
  matrixEigenToMsg(e.feedback_gain, m.feedback_gain);
  matrixEigenToMsg(e.feedforward, m.feedforward);
  sensorEigenToMsg(e.initial_state, m.initial_state);
}

}  // namespace linear_feedback_controller_msgs

#endif  // ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP

#ifndef ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP
#define ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_conversions/eigen_msg.h>

#include <linear_feedback_controller_msgs/Sensor.h>
#include <linear_feedback_controller_msgs/Control.h>

namespace linear_feedback_controller_msgs
{

namespace Eigen
{

struct JointState
{
  std::vector<std::string> name;
  ::Eigen::VectorXd position;
  ::Eigen::VectorXd velocity;
  ::Eigen::VectorXd effort;
};

struct Contact
{
  bool active;
  std::string name;
  ::Eigen::Matrix<double, 6, 1> wrench;
};

struct Sensor
{
  ::Eigen::Matrix<double, 7, 1> base_pose;
  ::Eigen::Matrix<double, 6, 1> base_twist;
  JointState joint_state;
  std::vector<Contact> contacts;
};

struct Control
{
  ::Eigen::MatrixXd feedback_gain;
  ::Eigen::VectorXd feedforward;
  linear_feedback_controller_msgs::Eigen::Sensor initial_state;
};
}  // namespace Eigen

// Create an alias to use only one namespace here.
template <class Derived>
void matrixEigenToMsg(const ::Eigen::MatrixBase<Derived>& e, std_msgs::Float64MultiArray& m)
{
  tf::matrixEigenToMsg(e, m);
}

template <class Derived>
void matrixMsgToEigen(const std_msgs::Float64MultiArray& m, ::Eigen::MatrixBase<Derived>& e)
{
  assert(m.layout.dim.size() == 2);
  // resize the eigen object.
  e = ::Eigen::MatrixBase<Derived>::Zero(m.layout.dim[0].size, m.layout.dim[1].stride);
  // verify the input object.
  assert(m.layout.dim[0].stride == e.size());
  assert(m.layout.dim[0].size == e.rows());
  assert(m.layout.dim[1].stride == e.cols());
  assert(m.layout.dim[1].size == e.cols());

  int ii = 0;
  for (int i = 0; i < e.rows(); ++i)
  {
    for (int j = 0; j < e.cols(); ++j)
    {
      e(i, j) = m.data[ii++];
    }
  }
}

/**
 * Msg To Eigen.
 */

inline void jointStateMsgToEigen(const sensor_msgs::JointState& m,
                                 linear_feedback_controller_msgs::Eigen::JointState& e)
{
  e.name = m.name;
  e.position = ::Eigen::Map<const ::Eigen::VectorXd>(m.position.data(), m.position.size());
  e.velocity = ::Eigen::Map<const ::Eigen::VectorXd>(m.velocity.data(), m.velocity.size());
  e.effort = ::Eigen::Map<const ::Eigen::VectorXd>(m.effort.data(), m.effort.size());
}

inline void contactMsgToEigen(const linear_feedback_controller_msgs::Contact& m,
                              linear_feedback_controller_msgs::Eigen::Contact& e)
{
  e.active = m.active;
  e.name = m.name;
  tf::wrenchMsgToEigen(m.wrench, e.wrench);
}

inline void sensorMsgToEigen(const Sensor& m, linear_feedback_controller_msgs::Eigen::Sensor& e)
{
  e.base_pose(0) = m.base_pose.position.x;
  e.base_pose(1) = m.base_pose.position.y;
  e.base_pose(2) = m.base_pose.position.z;
  e.base_pose(3) = m.base_pose.orientation.x;
  e.base_pose(4) = m.base_pose.orientation.y;
  e.base_pose(5) = m.base_pose.orientation.z;
  e.base_pose(6) = m.base_pose.orientation.w;
  tf::twistMsgToEigen(m.base_twist, e.base_twist);
  jointStateMsgToEigen(m.joint_state, e.joint_state);
  e.contacts.resize(m.contacts.size());
  for (std::size_t i = 0; i < m.contacts.size() ; ++i)
  {
    contactMsgToEigen(m.contacts[i], e.contacts[i]);
  }
}

inline void controlMsgToEigen(const Control& m, linear_feedback_controller_msgs::Eigen::Control& e)
{
  matrixMsgToEigen(m.feedback_gain, e.feedback_gain);
  matrixMsgToEigen(m.feedforward, e.feedforward);
  sensorMsgToEigen(m.initial_state, e.initial_state);
}

/**
 * Eigen To Msg.
 */

inline void jointStateEigenToMsg(const linear_feedback_controller_msgs::Eigen::JointState& e,
                                 sensor_msgs::JointState& m)
{
  m.name = e.name;
  m.position = std::vector<double>(e.position.data(), e.position.data() + e.position.size());
  m.velocity = std::vector<double>(e.velocity.data(), e.velocity.data() + e.velocity.size());
  m.effort = std::vector<double>(e.effort.data(), e.effort.data() + e.effort.size());
}

inline void contactEigenToMsg(const linear_feedback_controller_msgs::Eigen::Contact& e,
                              linear_feedback_controller_msgs::Contact& m)
{
  m.active = e.active;
  m.name = e.name;
  tf::wrenchEigenToMsg(e.wrench, m.wrench);
}

inline void sensorEigenToMsg(const linear_feedback_controller_msgs::Eigen::Sensor e, Sensor& m)
{
  m.base_pose.position.x = e.base_pose(0);
  m.base_pose.position.y = e.base_pose(1);
  m.base_pose.position.z = e.base_pose(2);
  m.base_pose.orientation.x = e.base_pose(3);
  m.base_pose.orientation.y = e.base_pose(4);
  m.base_pose.orientation.z = e.base_pose(5);
  m.base_pose.orientation.w = e.base_pose(6);
  tf::twistEigenToMsg(e.base_twist, m.base_twist);
  jointStateEigenToMsg(e.joint_state, m.joint_state);
  m.contacts.resize(e.contacts.size());
  for (std::size_t i = 0; i < e.contacts.size() ; ++i)
  {
    contactEigenToMsg(e.contacts[i], m.contacts[i]);
  }
}

inline void controlEigenToMsg(const linear_feedback_controller_msgs::Eigen::Control& e, Control& m)
{
  matrixEigenToMsg(e.feedback_gain, m.feedback_gain);
  matrixEigenToMsg(e.feedforward, m.feedforward);
  sensorEigenToMsg(e.initial_state, m.initial_state);
}

}  // namespace linear_feedback_controller_msgs

#endif  // ROS_WBMPC_MSGS_ROS_EIGEN_CONVERSION_HPP

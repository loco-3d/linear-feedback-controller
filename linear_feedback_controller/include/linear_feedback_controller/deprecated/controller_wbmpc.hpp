/*
 * Copyright 2020 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef MEMMO_SUBSCRIBER_CONTROLLER_SANDING_LPF_H
#define MEMMO_SUBSCRIBER_CONTROLLER_SANDING_LPF_H


#include <pinocchio/fwd.hpp>
#include <boost/algorithm/string.hpp>
#include <algorithm>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <pinocchio/algorithm/model.hpp>

#include <pal_base_ros_controller/base_robot_with_estimator_controller.h>
#include <realtime_tools/realtime_publisher.h>
#include <sensor_msgs/JointState.h>
#include <memmo_trajectory_controller/JointStateLPF.h>
#include <ddynamic_reconfigure/ddynamic_reconfigure.h>
#include <nav_msgs/Odometry.h>

namespace memmo
{
class SubscriberControllerSandingLPF : public pal_base_ros_controller::BaseRobotWithEsimatorController

{
public:
  bool loadEtras(ros::NodeHandle& control_nh) override;

  void updateExtras(const ros::Time& time, const ros::Duration& period) override;

  void startingExtras(const ros::Time& time) override;

  void stoppingExtra(const ros::Time& time) override;

private:

  void controlCb(const memmo_trajectory_controller::JointStateLPF &desired_state);

  ros::Subscriber state_sub_;
  std::timed_mutex mutex_;
  pinocchio::Model pin_model_;
  std::vector<std::string> controlled_joints_names_;
  std::map<std::string, int> actual_state_map_;
  Eigen::VectorXd desired_effort_;
  Eigen::VectorXd desired_pos_;
  Eigen::VectorXd desired_vel_;
  Eigen::VectorXd actual_pos_;
  Eigen::VectorXd actual_vel_;
  Eigen::VectorXd actual_tau_;
  Eigen::VectorXd desired_tau_;
  Eigen::VectorXd diff_state_;
  Eigen::MatrixXd riccati_gains_;
  memmo_trajectory_controller::JointStateLPF desired_state_;
  std::vector<double> initial_torque_;
  std::vector<double> initial_position_;
  std::vector<double> desired_torque_;
  std::vector<double> actual_torque_;
  std::vector<double> torque_offsets_;
  
  ddynamic_reconfigure::DDynamicReconfigurePtr dd_reconfigure_;
  double p_arm_gain_;
  double d_arm_gain_;
  double p_torso_gain_;
  double d_torso_gain_;
  double p_leg_gain_;
  double d_leg_gain_;

  boost::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::JointState>> joint_states_pub_;
  boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> base_state_pub_;
  sensor_msgs::JointState actual_js_state_;
  nav_msgs::Odometry actual_base_state_;

  pal_statistics::RegistrationsRAII registered_variables_;
  pal_robot_tools::TimeProfilerPtr tp_;
  int ms_mutex_;
  
  Eigen::Vector3d base_linear_vel_;
  Eigen::Vector3d base_angular_vel_;
  Eigen::Quaterniond base_orientation_;
  Eigen::Vector3d base_position_;
  std::deque<std::vector<double>> v_measures_;
  std::deque<std::vector<double>> tau_measures_;
  double nb_measure_;

};
}

#endif  // MEMMO_SUBSCRIBER_CONTROLLER_SANDING_LPF_H

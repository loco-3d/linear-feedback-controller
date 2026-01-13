#pragma once

#include <pinocchio/parsers/urdf.hpp>

#include "gmock/gmock.h"
#include "linear_feedback_controller/robot_model_builder.hpp"

namespace linear_feedback_controller {

// Alias
using StringVector = std::vector<std::string>;
using PinocchioIndexVector = std::vector<pinocchio::Index>;
using PinocchioModel =
    pinocchio::Model;  // pinocchio::Model -> pinocchio::ModelTpl<double, 0>
using PinocchioData =
    pinocchio::Data;  // pinocchio::Data -> pinocchio::DataTpl<double, 0>
using PinocchioToHardwareMap = std::map<int, int>;

// This mock is "smart": it is pre-configured with a valid 2-DoF robot.
// It provides real implementations for the base getters (get_model, get_nq,
// etc.) while allowing important interaction methods (like
// construct_robot_state) to be mocked.
class SmartMockRobotModelBuilder : public RobotModelBuilder {
 public:
  const std::string BASIC_URDF = R"(
    <robot name="simple_2dof_robot">
      <link name="base_link"/>
      <link name="link1"/>
      <link name="link2"/>
      <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <origin xyz="0 0 1" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit effort="100" velocity="100"/>
      </joint>
      <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0 0 1" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <limit effort="100" velocity="100"/>
      </joint>
    </robot>
  )";

  ~SmartMockRobotModelBuilder() override = default;

  // We do everything on the constructor
  SmartMockRobotModelBuilder() {
    moving_joint_names_ = {"joint1", "joint2"};

    pinocchio::urdf::buildModelFromXML(BASIC_URDF, model_);
    data_ = pinocchio::Data(model_);
    moving_joint_ids_ = {model_.getJointId("joint1"),
                         model_.getJointId("joint2")};
    locked_joint_ids_ = {};
    pinocchio_to_hardware_map_ = {{0, 0}, {1, 1}};
  }

  const PinocchioModel& get_model() const override { return model_; }
  PinocchioData& get_data() override { return data_; }
  const StringVector& get_moving_joint_names() const override {
    return moving_joint_names_;
  }
  const PinocchioIndexVector& get_moving_joint_ids() const override {
    return moving_joint_ids_;
  }
  const PinocchioIndexVector& get_locked_joint_ids() const override {
    return locked_joint_ids_;
  }
  const PinocchioToHardwareMap& get_pinocchio_to_hardware_interface_map()
      const override {
    return pinocchio_to_hardware_map_;
  }

  int get_nq() const override { return model_.nq; }
  int get_nv() const override { return model_.nv; }
  int get_joint_configuration_nq() const override { return model_.nq; }
  int get_joint_position_nq() const override { return model_.nq; }
  int get_joint_nv() const override { return model_.nv; }
  bool get_robot_has_free_flyer() const override { return false; }

  Eigen::VectorXd jointConfigToJointPositions(
      const Eigen::VectorXd& q_joint) const override {
    return q_joint;  // For revolute joints without free-flyer, config =
                     // position
  }

  Eigen::VectorXd jointPositionsToJointConfig(
      const Eigen::VectorXd& q_position) const override {
    return q_position;  // For revolute joints without free-flyer, position =
                        // config
  }

  MOCK_METHOD(void, construct_robot_state,
              (const linear_feedback_controller_msgs::Eigen::Sensor& Sensor,
               Eigen::VectorXd& robot_configuration,
               Eigen::VectorXd& robot_velocity),
              (override));

  using SharedPtr = std::shared_ptr<SmartMockRobotModelBuilder>;
  using ConstSharedPtr = std::shared_ptr<const SmartMockRobotModelBuilder>;

 private:
  PinocchioModel model_;
  PinocchioData data_;
  StringVector moving_joint_names_;
  PinocchioIndexVector moving_joint_ids_;
  PinocchioIndexVector locked_joint_ids_;
  PinocchioToHardwareMap pinocchio_to_hardware_map_;
};

}  // namespace linear_feedback_controller

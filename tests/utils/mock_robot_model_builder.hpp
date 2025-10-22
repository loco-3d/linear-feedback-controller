#pragma once

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

class MockRobotModelBuilder : public RobotModelBuilder {
 public:
  ~MockRobotModelBuilder() override = default;

  MOCK_METHOD(bool, build_model,
              (const std::string& urdf, const StringVector& moving_joint_names,
               const StringVector& controlled_joint_names,
               const bool robot_has_free_flyer),
              (override));

  MOCK_METHOD(const StringVector&, get_moving_joint_names, (),
              (const, override));

  MOCK_METHOD(const PinocchioIndexVector&, get_moving_joint_ids, (),
              (const, override));

  MOCK_METHOD(const PinocchioIndexVector&, get_locked_joint_ids, (),
              (const, override));

  MOCK_METHOD(const PinocchioModel&, get_model, (), (const, override));

  MOCK_METHOD(PinocchioData&, get_data, (), (override));

  MOCK_METHOD(bool, get_robot_has_free_flyer, (), (const, override));

  MOCK_METHOD(const PinocchioToHardwareMap&,
              get_pinocchio_to_hardware_interface_map, (), (const, override));

  MOCK_METHOD(void, construct_robot_state,
              (const linear_feedback_controller_msgs::Eigen::Sensor& Sensor,
               Eigen::VectorXd& robot_configuration,
               Eigen::VectorXd& robot_velocity),
              (override));

  MOCK_METHOD(int, get_joint_nv, (), (const, override));
  MOCK_METHOD(int, get_joint_nq, (), (const, override));
  MOCK_METHOD(int, get_nv, (), (const, override));
  MOCK_METHOD(int, get_nq, (), (const, override));

  using SharedPtr = std::shared_ptr<MockRobotModelBuilder>;
  using ConstSharedPtr = std::shared_ptr<const MockRobotModelBuilder>;
};

}  // namespace linear_feedback_controller

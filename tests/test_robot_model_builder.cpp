#include <gtest/gtest.h>

#include "linear_feedback_controller/robot_model_builder.hpp"

// URDF exemple, as simple as possible
const std::string simple_urdf_content = R"(
    <robot name="simple_test_robot">
      <link name="base_link"/>
      <link name="link1"/>
      <link name="link2"/>
      <link name="link3"/>
      <joint name="joint1" type="revolute">
        <parent link="base_link"/>
        <child link="link1"/>
        <limit effort="54.0" lower="-3.14159265" upper="3.14159265" velocity="3.2"/>
      </joint>
      <joint name="joint2" type="revolute">
        <parent link="link1"/>
        <child link="link2"/>
        <limit effort="54.0" lower="-3.14159265" upper="3.14159265" velocity="3.2"/>
      </joint>
      <joint name="joint3" type="continuous">
        <parent link="link2"/>
        <child link="link3"/>
        <limit effort="54.0" lower="-3.14159265" upper="3.14159265" velocity="3.2"/>
      </joint>
    </robot>
)";

using namespace linear_feedback_controller;

// Create a fixture to test NON free-flyer cases
class RobotModelBuilderNonFreeFlyerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    builder = std::make_unique<RobotModelBuilder>();

    std::vector<std::string> moving_joints = {"joint1", "joint2", "joint3"};
    std::vector<std::string> controlled_joints = {"joint1", "joint2", "joint3"};

    bool build_success = builder->build_model(
        simple_urdf_content, moving_joints, controlled_joints, false);
    ASSERT_TRUE(build_success) << "Failed to build non-free-flyer model.";
  }

  std::unique_ptr<RobotModelBuilder> builder;
};

TEST_F(RobotModelBuilderNonFreeFlyerTest, PinocchioModelAndDataAreCorrect) {
  const auto& model = builder->get_model();
  // Pinocchio add a "universe" joint, so we get 3+1 joints
  EXPECT_EQ(model.njoints, 4);
  EXPECT_EQ(model.nv, 3);
  // continuous joint adds 2 DoF (cos, sin)
  EXPECT_EQ(model.nq, 4);

  const auto& data = builder->get_data();
  EXPECT_EQ(data.f.size(), model.njoints);
}

TEST_F(RobotModelBuilderNonFreeFlyerTest, BuilderParametersAreCorrect) {
  EXPECT_EQ(builder->get_nv(), 3);
  // continuous joint adds 2 DoF (cos, sin)
  EXPECT_EQ(builder->get_nq(), 4);

  // *_join_* represent the idea of considering only the joints provided by the
  // model. So it is different only in case of free flyer nv(+6) and nq(+7)
  EXPECT_EQ(builder->get_joint_nv(), builder->get_nv());
  EXPECT_EQ(builder->get_joint_pin_nq(), builder->get_nq());
  // get_joint_hw_nq() should return nq -1 for continuous joint
  EXPECT_EQ(builder->get_joint_hw_nq(), builder->get_nq() - 1);
}

TEST_F(RobotModelBuilderNonFreeFlyerTest, RobotHasFreeFlyerFlagIsFalse) {
  EXPECT_FALSE(builder->get_robot_has_free_flyer());
}

TEST_F(RobotModelBuilderNonFreeFlyerTest, MovingJointNamesAreCorrect) {
  // No changes should be made by the builder
  std::vector<std::string> expected_names = {"joint1", "joint2", "joint3"};
  const auto& returned_names = builder->get_moving_joint_names();
  ASSERT_EQ(returned_names.size(), expected_names.size());
  EXPECT_EQ(returned_names, expected_names);
}

TEST_F(RobotModelBuilderNonFreeFlyerTest, MovingJointIdsAreCorrect) {
  const auto& returned_ids = builder->get_moving_joint_ids();

  ASSERT_EQ(returned_ids.size(), 3);

  EXPECT_EQ(returned_ids[0], 1);
  EXPECT_EQ(returned_ids[1], 2);
  EXPECT_EQ(returned_ids[2], 3);
}

TEST_F(RobotModelBuilderNonFreeFlyerTest, LockedJointIdsAreEmpty) {
  // No locked joints on this model
  const auto& returned_locked_ids = builder->get_locked_joint_ids();
  EXPECT_TRUE(returned_locked_ids.empty());
}

TEST_F(RobotModelBuilderNonFreeFlyerTest,
       PinocchioToHardwareInterfaceMapIsCorrect) {
  const auto& model = builder->get_model();
  const auto& pin_to_hwi = builder->get_pinocchio_to_hardware_interface_map();
  ASSERT_EQ(pin_to_hwi.size(), 3);

  EXPECT_EQ(pin_to_hwi.at(0), 0);  // (0, 0)
  EXPECT_EQ(pin_to_hwi.at(1), 1);  // (1, 1)
  EXPECT_EQ(pin_to_hwi.at(2), 2);  // (2, 2)
}

TEST_F(RobotModelBuilderNonFreeFlyerTest, ConstructRobotStateCorrectly) {
  linear_feedback_controller_msgs::Eigen::Sensor sensor;
  const int n_hw = builder->get_joint_hw_nq();  // 3 : 3 joints for the hw
  const int nq = builder->get_nq();             // 4 : 1+1+2 for pinocchio
  const int nv = builder->get_nv();             // 3

  Eigen::VectorXd joint_positions(n_hw);
  joint_positions << 0.1, 0.2, 0.3;
  sensor.joint_state.position = joint_positions;

  Eigen::VectorXd joint_velocities(n_hw);
  joint_velocities << 1.1, 1.2, 1.3;
  sensor.joint_state.velocity = joint_velocities;

  Eigen::VectorXd robot_configuration(nq);
  Eigen::VectorXd robot_velocity(nv);

  builder->construct_robot_state(sensor, robot_configuration, robot_velocity);

  ASSERT_EQ(robot_configuration.size(), nq);

  const double th1 = joint_positions[0];  // joint1 (revolute)
  const double th2 = joint_positions[1];  // joint2 (revolute)
  const double th3 = joint_positions[2];  // joint3 (continuous)

  // joint1 (nq=1)
  EXPECT_NEAR(robot_configuration[0], th1, 1e-12);

  // joint2 (nq=1)
  EXPECT_NEAR(robot_configuration[1], th2, 1e-12);

  // joint3 (nq=2): [cos θ3, sin θ3]
  EXPECT_NEAR(robot_configuration[2], std::cos(th3), 1e-12);
  EXPECT_NEAR(robot_configuration[3], std::sin(th3), 1e-12);

  ASSERT_EQ(robot_velocity.size(), nv);
  EXPECT_TRUE(robot_velocity.isApprox(joint_velocities, 1e-12));
}

// Create a fixture to test free-flyer cases
class RobotModelBuilderFreeFlyerTest : public ::testing::Test {
 protected:
  // Function called before each test
  void SetUp() override {
    builder = std::make_unique<RobotModelBuilder>();

    std::vector<std::string> moving_joints = {"joint1", "joint2", "joint3"};
    std::vector<std::string> controlled_joints = {"joint1", "joint2", "joint3"};

    bool build_success = builder->build_model(
        simple_urdf_content, moving_joints, controlled_joints, true);
    ASSERT_TRUE(build_success) << "Failed to build free-flyer model.";
  }

  // Objects used on each test
  std::unique_ptr<RobotModelBuilder> builder;
};

TEST_F(RobotModelBuilderFreeFlyerTest, PinocchioModelIsCorrect) {
  const auto& model = builder->get_model();
  // Pinocchio add a "universe" joint and add a "root_joint" joint for
  // freeflyers, so we get 3+1+1 joints
  EXPECT_EQ(model.njoints, 5);
  // The "root_joint" is a 6 DoF freedown movement - represented by nv += 6
  EXPECT_EQ(model.nv, 9);
  // and nq += 7
  // continuous joint adds 2 DoF (cos, sin) so we add 1 here because we have 1
  // continuous joint
  EXPECT_EQ(model.nq, 11);
}

TEST_F(RobotModelBuilderFreeFlyerTest, BuilderParametersAreCorrect) {
  EXPECT_EQ(builder->get_nv(), 9);
  EXPECT_EQ(builder->get_nq(), 11);

  // *_join_* represent the idea of considering only the joints provided by the
  // model. So it is different only in case of free flyer
  EXPECT_NE(builder->get_joint_nv(), builder->get_nv());
  EXPECT_NE(builder->get_joint_pin_nq(), builder->get_nq());
}

TEST_F(RobotModelBuilderFreeFlyerTest, RobotHasFreeFlyerFlagIsTrue) {
  EXPECT_TRUE(builder->get_robot_has_free_flyer());
}

TEST_F(RobotModelBuilderFreeFlyerTest, MovingJointNamesAreCorrect) {
  // No changes should be made by the builder
  // root_joint should not be reported by the builder
  std::vector<std::string> expected_names = {"joint1", "joint2", "joint3"};
  const auto& returned_names = builder->get_moving_joint_names();
  ASSERT_EQ(returned_names.size(), expected_names.size());
  EXPECT_EQ(returned_names, expected_names);
}

TEST_F(RobotModelBuilderFreeFlyerTest, MovingJointIdsAreCorrect) {
  const auto& returned_ids = builder->get_moving_joint_ids();

  ASSERT_EQ(returned_ids.size(), 3);
  // with "root_joint" added, ids are shifted
  EXPECT_EQ(returned_ids[0], 2);
  EXPECT_EQ(returned_ids[1], 3);
  EXPECT_EQ(returned_ids[2], 4);
}

TEST_F(RobotModelBuilderFreeFlyerTest,
       PinocchioToHardwareInterfaceMapIsCorrect) {
  const auto& model = builder->get_model();
  const auto& pin_to_hwi = builder->get_pinocchio_to_hardware_interface_map();
  ASSERT_EQ(pin_to_hwi.size(), 3);

  EXPECT_EQ(pin_to_hwi.at(0), 0);
  EXPECT_EQ(pin_to_hwi.at(1), 1);
  EXPECT_EQ(pin_to_hwi.at(2), 2);
}

TEST_F(RobotModelBuilderFreeFlyerTest, ConstructRobotStateCorrectly) {
  linear_feedback_controller_msgs::Eigen::Sensor sensor;

  // Data for free-flyer base
  Eigen::Vector<double, 7> base_pose;
  base_pose << 1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0;  // x,y,z, qx,qy,qz,qw
  sensor.base_pose = base_pose;

  Eigen::Vector<double, 6> base_twist;
  base_twist << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;  // v_x,y,z, w_x,y,z
  sensor.base_twist = base_twist;

  // Data for joints
  const int n_hw = builder->get_joint_hw_nq();  // 3 : 3 joints
  const int nq = builder->get_nq();  // 7 (FF) + 1 + 1 + 2 (continuous) = 11
  const int nv = builder->get_nv();  // 6 (FF) + 1 + 1 + 1 = 9

  Eigen::VectorXd joint_positions(n_hw);
  joint_positions << 0.7, 0.8, 0.9;
  sensor.joint_state.position = joint_positions;

  Eigen::VectorXd joint_velocities(n_hw);
  joint_velocities << 1.7, 1.8, 1.9;
  sensor.joint_state.velocity = joint_velocities;

  Eigen::VectorXd robot_configuration(nq);
  Eigen::VectorXd robot_velocity(nv);

  builder->construct_robot_state(sensor, robot_configuration, robot_velocity);

  // Verify "free-flyer" part (beginning)
  EXPECT_TRUE(robot_configuration.head<7>().isApprox(base_pose, 1e-12));
  EXPECT_TRUE(robot_velocity.head<6>().isApprox(base_twist, 1e-12));

  // Verify joint part (end)

  const int iq0 = 7;  // offset en q après la base
  const int iv0 = 6;  // offset en v après la base

  const double th1 = joint_positions[0];  // joint1 (revolute)
  const double th2 = joint_positions[1];  // joint2 (revolute)
  const double th3 = joint_positions[2];  // joint3 (continuous)

  // joint1 (nq=1)
  EXPECT_NEAR(robot_configuration[iq0 + 0], th1, 1e-12);

  // joint2 (nq=1)
  EXPECT_NEAR(robot_configuration[iq0 + 1], th2, 1e-12);

  // joint3 (nq=2): q = [cos θ3, sin θ3]
  EXPECT_NEAR(robot_configuration[iq0 + 2], std::cos(th3), 1e-12);
  EXPECT_NEAR(robot_configuration[iq0 + 3], std::sin(th3), 1e-12);

  // Velocity part
  EXPECT_TRUE(
      robot_velocity.segment(iv0, n_hw).isApprox(joint_velocities, 1e-12));
}

// Fixture to test filtering functionalities
class RobotModelBuilderFilteringTest : public ::testing::Test {
 protected:
  void SetUp() override { builder = std::make_unique<RobotModelBuilder>(); }

  std::unique_ptr<RobotModelBuilder> builder;
  std::vector<std::string> moving_joints_;
  std::vector<std::string> controlled_joints_;
};

TEST_F(RobotModelBuilderFilteringTest, CorrectlyFiltersMovingAndLockedJoints) {
  moving_joints_ = {"joint1", "joint3"};
  controlled_joints_ = {"joint1", "joint3"};

  bool build_success = builder->build_model(simple_urdf_content, moving_joints_,
                                            controlled_joints_, false);
  ASSERT_TRUE(build_success) << "Failed to build filtered model.";

  const auto& moving_ids = builder->get_moving_joint_ids();
  EXPECT_EQ(moving_ids.size(), 2);
  EXPECT_EQ(moving_ids[0], 1);
  EXPECT_EQ(moving_ids[1], 3);

  const auto& locked_ids = builder->get_locked_joint_ids();
  ASSERT_EQ(locked_ids.size(), 1);
  EXPECT_EQ(locked_ids[0], 2);

  const auto& returned_moving_names = builder->get_moving_joint_names();
  EXPECT_EQ(returned_moving_names, moving_joints_);
}

TEST_F(RobotModelBuilderFilteringTest,
       CorrectlyHandlesInvertedControlledAndMovingJointNames) {
  moving_joints_ = {"joint3", "joint2"};
  controlled_joints_ = {"joint2", "joint1", "joint3"};

  bool build_success = builder->build_model(simple_urdf_content, moving_joints_,
                                            controlled_joints_, false);
  ASSERT_TRUE(build_success) << "Failed to build filtered model.";

  // should be sorted
  const auto& returned_moving_names = builder->get_moving_joint_names();
  EXPECT_EQ(returned_moving_names[0], "joint2");
  EXPECT_EQ(returned_moving_names[1], "joint3");

  const auto& pin_to_hwi = builder->get_pinocchio_to_hardware_interface_map();
  // looks like pin_to_hwi is used because controlled_joints is not sorted
  // sorted moving_joint ID: 0 -> controlled_joints (HWI) Index: 0
  // sorted moving_joint ID: 1 -> controlled_joints (HWI) Index: 2
  ASSERT_EQ(pin_to_hwi.size(), 2);
  EXPECT_EQ(pin_to_hwi.at(0), 0);
  EXPECT_EQ(pin_to_hwi.at(1), 2);
}

// Test errors from builder
class RobotModelBuilderErrorTest : public ::testing::Test {
 protected:
  void SetUp() override { builder = std::make_unique<RobotModelBuilder>(); }
  std::unique_ptr<RobotModelBuilder> builder;
};

TEST_F(RobotModelBuilderErrorTest, BuildFailsWithNonexistentJointName) {
  std::vector<std::string> moving_joints = {"joint1", "non_existent_joint"};
  std::vector<std::string> controlled_joints = {"joint1"};

  EXPECT_FALSE(builder->build_model(simple_urdf_content, moving_joints,
                                    controlled_joints, false));
}

TEST_F(RobotModelBuilderErrorTest,
       BuildFailsWithMoreMovingJointsThanControlledJoints) {
  std::vector<std::string> moving_joints = {"joint1", "joint2"};
  std::vector<std::string> controlled_joints = {"joint1"};

  EXPECT_FALSE(builder->build_model(simple_urdf_content, moving_joints,
                                    controlled_joints, false));
}

#if !defined(NDEBUG)
TEST_F(RobotModelBuilderErrorTest, ConstructStateDeathOnWrongConfigSize) {
  std::vector<std::string> moving_joints = {"joint1"};
  std::vector<std::string> controlled_joints = {"joint1"};
  builder->build_model(simple_urdf_content, moving_joints, controlled_joints,
                       false);

  linear_feedback_controller_msgs::Eigen::Sensor sensor;
  Eigen::VectorXd robot_velocity(builder->get_nv());  // correct vector size

  // Create vector with incorrect size
  Eigen::VectorXd wrong_size_config(builder->get_nq() + 1);

  ASSERT_DEATH(
      builder->construct_robot_state(sensor, wrong_size_config, robot_velocity),
      "robot_configuration has the wrong size");
}

TEST_F(RobotModelBuilderErrorTest, ConstructStateDeathOnWrongVelocitySize) {
  std::vector<std::string> moving_joints = {"joint1"};
  std::vector<std::string> controlled_joints = {"joint1"};
  builder->build_model(simple_urdf_content, moving_joints, controlled_joints,
                       false);

  linear_feedback_controller_msgs::Eigen::Sensor sensor;
  Eigen::VectorXd robot_configuration(
      builder->get_nq());  // correct vector size

  // Create vector with incorrect size
  Eigen::VectorXd wrong_size_velocity(builder->get_nv() - 1);

  ASSERT_DEATH(builder->construct_robot_state(sensor, robot_configuration,
                                              wrong_size_velocity),
               "robot_velocity has the wrong size");
}
#endif  // !defined(NDEBUG)

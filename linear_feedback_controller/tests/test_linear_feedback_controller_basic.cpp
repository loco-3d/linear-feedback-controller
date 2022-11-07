#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <example-robot-data/path.hpp>

#include "linear_feedback_controller/linear_feedback_controller.hpp"

using namespace linear_feedback_controller;

class LinearFeedbackControllerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string urdf_file = std::string(EXAMPLE_ROBOT_DATA_MODEL_DIR) + "/talos_data/robots/talos_reduced.urdf";
    std::string srdf_file = std::string(EXAMPLE_ROBOT_DATA_MODEL_DIR) + "/talos_data/srdf/talos.srdf";

    std::ifstream urdf_stream(urdf_file);
    if (!urdf_stream) {
      std::cout << "URDF file " << urdf_file << " is not found." << std::endl;
    }
    ASSERT_TRUE(urdf_stream);
    std::stringstream urdf;
    urdf << urdf_stream.rdbuf();

    std::ifstream srdf_stream(srdf_file);
    if (!srdf_stream) {
      std::cout << "SRDF file " << srdf_file << " is not found." << std::endl;
    }
    ASSERT_TRUE(srdf_stream);
    std::stringstream srdf;
    srdf << srdf_stream.rdbuf();

    // clang-format off
    // controlled_joint_names
    sorted_controlled_joint_names_ = {
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
    };
    mixed_controlled_joint_names_ = {
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
    };
    wrong_controlled_joint_names_ = {
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
        "banana",
    };
    duplicate_controlled_joint_names_ = {
      "root_joint", "root_joint",
      "leg_left_1_joint", "leg_left_1_joint",
      "leg_left_2_joint", "leg_left_2_joint",
      "leg_left_3_joint", "leg_left_3_joint",
      "leg_left_4_joint", "leg_left_4_joint",
      "leg_left_5_joint", "leg_left_5_joint",
      "leg_left_6_joint", "leg_left_6_joint",
      "leg_right_1_joint", "leg_right_1_joint",
      "leg_right_2_joint", "leg_right_2_joint",
      "leg_right_3_joint", "leg_right_3_joint",
      "leg_right_4_joint", "leg_right_4_joint",
      "leg_right_5_joint", "leg_right_5_joint",
      "leg_right_6_joint", "leg_right_6_joint",
      "torso_1_joint", "torso_1_joint",
      "torso_2_joint", "torso_2_joint",
    };
    sorted_controlled_joint_ids_ =
      { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
    sorted_locked_joint_ids_ =
      { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33 };
    // clang-format on
    bool robot_has_free_flyer = true;

    torque_offset_names_ = {
        "joints/root_joint/actuator_params/torque_sensor_offset",
        "joints/leg_left_1_joint/actuator_params/torque_sensor_offset",
        "joints/leg_left_2_joint/actuator_params/torque_sensor_offset",
        "joints/leg_left_3_joint/actuator_params/torque_sensor_offset",
        "joints/leg_left_4_joint/actuator_params/torque_sensor_offset",
        "joints/leg_left_5_joint/actuator_params/torque_sensor_offset",
        "joints/leg_left_6_joint/actuator_params/torque_sensor_offset",
        "joints/leg_right_1_joint/actuator_params/torque_sensor_offset",
        "joints/leg_right_2_joint/actuator_params/torque_sensor_offset",
        "joints/leg_right_3_joint/actuator_params/torque_sensor_offset",
        "joints/leg_right_4_joint/actuator_params/torque_sensor_offset",
        "joints/leg_right_5_joint/actuator_params/torque_sensor_offset",
        "joints/leg_right_6_joint/actuator_params/torque_sensor_offset",
        "joints/torso_1_joint/actuator_params/torque_sensor_offset",
        "joints/torso_2_joint/actuator_params/torque_sensor_offset",
    };
    torque_offset_values_ = {
        0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4,
    };

    // Get the parameters of the node.
    nh_.setParam("robot_description", urdf.str());
    nh_.setParam("robot_description_semantic", srdf.str());
    nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
    nh_.setParam("robot_has_free_flyer", robot_has_free_flyer);
  }

  void TearDown() override {}

  ros::NodeHandle nh_;
  std::vector<long unsigned int> sorted_controlled_joint_ids_;
  std::vector<long unsigned int> sorted_locked_joint_ids_;
  std::vector<std::string> sorted_controlled_joint_names_;
  std::vector<std::string> mixed_controlled_joint_names_;
  std::vector<std::string> wrong_controlled_joint_names_;
  std::vector<std::string> duplicate_controlled_joint_names_;
  std::vector<std::string> torque_offset_names_;
  std::vector<double> torque_offset_values_;
};
class DISABLED_LinearFeedbackControllerTest : public LinearFeedbackControllerTest {};

TEST_F(LinearFeedbackControllerTest, checkConstructor) { LinearFeedbackController obj; }

TEST_F(LinearFeedbackControllerTest, checkInitializeNoThrow) {
  LinearFeedbackController obj;
  ASSERT_TRUE(obj.loadEtras(nh_));
}

// TEST_F(LinearFeedbackControllerTest, checkControlledJointNamesSorted) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   ASSERT_EQ(obj.getControlledJointNames(), sorted_controlled_joint_names_);
// }

// TEST_F(LinearFeedbackControllerTest, checkControlledJointNamesMixed) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   ASSERT_EQ(obj.getControlledJointNames(), sorted_controlled_joint_names_);
// }

// TEST_F(LinearFeedbackControllerTest, checkControlledJointNamesWrong) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   sorted_controlled_joint_names_.pop_back();
//   ASSERT_EQ(obj.getControlledJointNames(), sorted_controlled_joint_names_);
// }

// TEST_F(LinearFeedbackControllerTest, checkControlledJointNamesDuplicate) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   ASSERT_EQ(obj.getControlledJointNames(), sorted_controlled_joint_names_);
// }

// TEST_F(LinearFeedbackControllerTest, checkControlledJointIdsSorted) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> controlled_joint_ids = obj.getControlledJointIds();
//   for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
//     ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
//   }
//   ASSERT_EQ(obj.getControlledJointIds(), sorted_controlled_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerTest, checkControlledJointIdsMixed) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> controlled_joint_ids = obj.getControlledJointIds();
//   for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
//     ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
//   }
//   ASSERT_EQ(obj.getControlledJointIds(), sorted_controlled_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerTest, checkControlledJointIdsWrong) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   sorted_controlled_joint_names_.pop_back();

//   std::vector<long unsigned int> controlled_joint_ids = obj.getControlledJointIds();
//   for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
//     ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
//   }
//   sorted_controlled_joint_ids_.pop_back();
//   ASSERT_EQ(obj.getControlledJointIds(), sorted_controlled_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerTest, checkControlledJointIdsDuplicate) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> controlled_joint_ids = obj.getControlledJointIds();
//   for (std::size_t i = 1; i < controlled_joint_ids.size(); ++i) {
//     ASSERT_LE(controlled_joint_ids[i - 1], controlled_joint_ids[i]);
//   }
//   ASSERT_EQ(obj.getControlledJointIds(), sorted_controlled_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerTest, checkLockedJointIdsSorted) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", sorted_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> locked_joint_ids = obj.getLockedJointIds();
//   for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
//     ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
//   }
//   ASSERT_EQ(obj.getLockedJointIds(), sorted_locked_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerTest, checkLockedJointIdsMixed) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", mixed_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> locked_joint_ids = obj.getLockedJointIds();
//   for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
//     ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
//   }
//   ASSERT_EQ(obj.getLockedJointIds(), sorted_locked_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerTest, checkLockedJointIdsWrong) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", wrong_controlled_joint_names_);
//   obj.loadEtras(nh_);
//   sorted_controlled_joint_names_.pop_back();

//   std::vector<long unsigned int> locked_joint_ids = obj.getLockedJointIds();
//   for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
//     ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
//   }
//   sorted_locked_joint_ids_.insert(sorted_locked_joint_ids_.begin(), sorted_controlled_joint_ids_.back());
//   ASSERT_EQ(obj.getLockedJointIds(), sorted_locked_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerTest, checkLockedJointIdsDuplicate) {
//   LinearFeedbackController obj;
//   nh_.setParam("controlled_joint_names", duplicate_controlled_joint_names_);
//   obj.loadEtras(nh_);

//   std::vector<long unsigned int> locked_joint_ids = obj.getLockedJointIds();
//   for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
//     ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
//   }
//   ASSERT_EQ(obj.getLockedJointIds(), sorted_locked_joint_ids_);
// }

// TEST_F(LinearFeedbackControllerTest, DISABLED_checkJointTorqueOffsets) {
//   ASSERT_TRUE(false);
// }
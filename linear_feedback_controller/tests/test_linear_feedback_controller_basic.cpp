#include <gtest/gtest.h>
#include <fstream>
#include <sstream>
#include <example-robot-data/path.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>

#include "linear_feedback_controller/linear_feedback_controller.hpp"

using namespace linear_feedback_controller;

class LinearFeedbackControllerTest : public ::testing::Test {
 protected:

  bool dirExists(const char *path)
  {
      struct stat info;

      if(stat( path, &info ) != 0)
          return false;
      else if(info.st_mode & S_IFDIR)
          return true;
      else
          return false;
  }
  
  std::string readFile(std::string filename)
  {
    std::ifstream fbin = std::ifstream(filename);
    if (!fbin) {
      throw std::runtime_error("File " + filename + " is not found.");
    }
    std::stringstream ss;
    ss << fbin.rdbuf();
    std::string out = ss.str();
    return out;
  }

  void SetUp() override
  {
    std::string data_dir = EXAMPLE_ROBOT_DATA_MODEL_DIR;
    if (!dirExists(data_dir.c_str()))
    {
      data_dir = "/opt/ros/melodic/share/example-robot-data/robots";
    }
    ASSERT_TRUE(dirExists(data_dir.c_str()));
    urdf_ = readFile(data_dir + "/talos_data/robots/talos_reduced.urdf");
    srdf_ = readFile(data_dir + "/talos_data/srdf/talos.srdf");

    // clang-format off
    // moving_joint_names
    sorted_moving_joint_names_ = {
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
    mixed_moving_joint_names_ = {
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
    wrong_moving_joint_names_ = {
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
    duplicate_moving_joint_names_ = {
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
    sorted_moving_joint_ids_ =
      { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
    sorted_locked_joint_ids_ =
      { 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33 };
    // clang-format on
    robot_has_free_flyer_ = true;

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
    nh_.setParam("robot_description", urdf_);
    nh_.setParam("robot_description_semantic", srdf_);
    nh_.setParam("moving_joint_names", sorted_moving_joint_names_);
    nh_.setParam("robot_has_free_flyer", robot_has_free_flyer_);
    for (std::size_t i = 0 ; i < torque_offset_names_.size() ; ++i)
    {
      nh_.setParam(torque_offset_names_[i], torque_offset_values_[i]);
    }
  }

  void TearDown() override {}

  std::string urdf_;
  std::string srdf_;
  std::vector<long unsigned int> sorted_moving_joint_ids_;
  std::vector<long unsigned int> sorted_locked_joint_ids_;
  std::vector<std::string> sorted_moving_joint_names_;
  std::vector<std::string> mixed_moving_joint_names_;
  std::vector<std::string> wrong_moving_joint_names_;
  std::vector<std::string> duplicate_moving_joint_names_;
  std::vector<std::string> torque_offset_names_;
  std::vector<double> torque_offset_values_;
  bool robot_has_free_flyer_;
  ros::NodeHandle nh_;
};
class DISABLED_LinearFeedbackControllerTest : public LinearFeedbackControllerTest {};

TEST_F(LinearFeedbackControllerTest, checkConstructor) { LinearFeedbackController obj; }

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_RobotModel) {
  LinearFeedbackController obj;
  bool ret = obj.loadEtras(nh_);
  ASSERT_FALSE(ret); // No hardware interface exists yet here...
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_MovingJointNamesSorted) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", sorted_moving_joint_names_);
  obj.loadEtras(nh_);
  ASSERT_EQ(obj.getMovingJointNames(), sorted_moving_joint_names_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_MovingJointNamesMixed) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", mixed_moving_joint_names_);
  obj.loadEtras(nh_);
  ASSERT_EQ(obj.getMovingJointNames(), sorted_moving_joint_names_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_MovingJointNamesWrong) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", wrong_moving_joint_names_);
  obj.loadEtras(nh_);
  sorted_moving_joint_names_.pop_back();
  ASSERT_EQ(obj.getMovingJointNames(), sorted_moving_joint_names_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_MovingJointNamesDuplicate) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", duplicate_moving_joint_names_);
  obj.loadEtras(nh_);
  ASSERT_EQ(obj.getMovingJointNames(), sorted_moving_joint_names_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_MovingJointIdsSorted) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", sorted_moving_joint_names_);
  obj.loadEtras(nh_);

  std::vector<long unsigned int> moving_joint_ids = obj.getMovingJointIds();
  for (std::size_t i = 1; i < moving_joint_ids.size(); ++i) {
    ASSERT_LE(moving_joint_ids[i - 1], moving_joint_ids[i]);
  }
  ASSERT_EQ(obj.getMovingJointIds(), sorted_moving_joint_ids_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_MovingJointIdsMixed) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", mixed_moving_joint_names_);
  obj.loadEtras(nh_);

  std::vector<long unsigned int> moving_joint_ids = obj.getMovingJointIds();
  for (std::size_t i = 1; i < moving_joint_ids.size(); ++i) {
    ASSERT_LE(moving_joint_ids[i - 1], moving_joint_ids[i]);
  }
  ASSERT_EQ(obj.getMovingJointIds(), sorted_moving_joint_ids_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_MovingJointIdsWrong) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", wrong_moving_joint_names_);
  obj.loadEtras(nh_);
  sorted_moving_joint_names_.pop_back();

  std::vector<long unsigned int> moving_joint_ids = obj.getMovingJointIds();
  for (std::size_t i = 1; i < moving_joint_ids.size(); ++i) {
    ASSERT_LE(moving_joint_ids[i - 1], moving_joint_ids[i]);
  }
  sorted_moving_joint_ids_.pop_back();
  ASSERT_EQ(obj.getMovingJointIds(), sorted_moving_joint_ids_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_MovingJointIdsDuplicate) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", duplicate_moving_joint_names_);
  obj.loadEtras(nh_);

  std::vector<long unsigned int> moving_joint_ids = obj.getMovingJointIds();
  for (std::size_t i = 1; i < moving_joint_ids.size(); ++i) {
    ASSERT_LE(moving_joint_ids[i - 1], moving_joint_ids[i]);
  }
  ASSERT_EQ(obj.getMovingJointIds(), sorted_moving_joint_ids_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_LockedJointIdsSorted) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", sorted_moving_joint_names_);
  obj.loadEtras(nh_);

  std::vector<long unsigned int> locked_joint_ids = obj.getLockedJointIds();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  ASSERT_EQ(obj.getLockedJointIds(), sorted_locked_joint_ids_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_LockedJointIdsMixed) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", mixed_moving_joint_names_);
  obj.loadEtras(nh_);

  std::vector<long unsigned int> locked_joint_ids = obj.getLockedJointIds();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  ASSERT_EQ(obj.getLockedJointIds(), sorted_locked_joint_ids_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_LockedJointIdsWrong) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", wrong_moving_joint_names_);
  obj.loadEtras(nh_);
  sorted_moving_joint_names_.pop_back();

  std::vector<long unsigned int> locked_joint_ids = obj.getLockedJointIds();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  sorted_locked_joint_ids_.insert(sorted_locked_joint_ids_.begin(), sorted_moving_joint_ids_.back());
  ASSERT_EQ(obj.getLockedJointIds(), sorted_locked_joint_ids_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_LockedJointIdsDuplicate) {
  LinearFeedbackController obj;
  nh_.setParam("moving_joint_names", duplicate_moving_joint_names_);
  obj.loadEtras(nh_);

  std::vector<long unsigned int> locked_joint_ids = obj.getLockedJointIds();
  for (std::size_t i = 1; i < locked_joint_ids.size(); ++i) {
    ASSERT_LE(locked_joint_ids[i - 1], locked_joint_ids[i]);
  }
  ASSERT_EQ(obj.getLockedJointIds(), sorted_locked_joint_ids_);
}

TEST_F(LinearFeedbackControllerTest, checkLoadEtras_JointTorqueOffsets) {
  ROS_WARN_STREAM("This test is only valid because no roscontrol hardware exists here.");
  LinearFeedbackController obj;
  obj.loadEtras(nh_);
  ASSERT_EQ(obj.getTorqueOffsets(), std::vector<double>());
}

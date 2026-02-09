#include "linear_feedback_controller/robot_model_builder.hpp"

#include <cmath>
#include <numeric>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace linear_feedback_controller {

const Eigen::Index RobotModelBuilder::free_flyer_nq_ = 7;

const Eigen::Index RobotModelBuilder::free_flyer_nv_ = 6;

RobotModelBuilder::RobotModelBuilder() {}

RobotModelBuilder::~RobotModelBuilder() {}

bool RobotModelBuilder::build_model(
    const std::string& urdf, const std::vector<std::string>& moving_joint_names,
    const std::vector<std::string>& controlled_joint_names,
    const bool robot_has_free_flyer) {
  // Copy the argument inside the class.
  moving_joint_names_ = moving_joint_names;
  robot_has_free_flyer_ = robot_has_free_flyer;
  controlled_joint_names_ = controlled_joint_names;

  pinocchio::Model pinocchio_model_complete;
  // Build the rigid body model of the robot.
  if (robot_has_free_flyer_) {
    pinocchio::urdf::buildModelFromXML(urdf, pinocchio::JointModelFreeFlyer(),
                                       pinocchio_model_complete);
  } else {
    pinocchio::urdf::buildModelFromXML(urdf, pinocchio_model_complete);
  }
  std::cout << "Robot model:\n" << pinocchio_model_complete << std::endl;

  // Reduce the rigid body model and set initial position.
  if (!parse_moving_joint_names(pinocchio_model_complete, moving_joint_names_,
                                controlled_joint_names_)) {
    return false;
  }

  q_default_complete_ = Eigen::VectorXd::Zero(pinocchio_model_complete.nq);
  pinocchio_model_ = pinocchio::buildReducedModel(
      pinocchio_model_complete, locked_joint_ids_, q_default_complete_);
  pinocchio_data_ = pinocchio::Data(pinocchio_model_);
  return true;
}

bool RobotModelBuilder::parse_moving_joint_names(
    const pinocchio::Model& pinocchio_model_complete,
    const std::vector<std::string>& moving_joint_names,
    const std::vector<std::string>& controlled_joint_names) {
  // Reset internal variables.
  moving_joint_ids_.clear();
  locked_joint_ids_.clear();
  pin_to_hwi_.clear();
  joint_nq_per_joint_.clear();
  joint_nv_per_joint_.clear();
  joint_categories_.clear();

  bool failure = false;

  for (const auto& joint_name : moving_joint_names) {
    pinocchio::JointIndex joint_id = 0;
    // Do not consider joint that are not in the model.
    if (pinocchio_model_complete.existJointName(joint_name)) {
      joint_id = pinocchio_model_complete.getJointId(joint_name);
      moving_joint_ids_.push_back(joint_id);
    } else {
      std::cerr << "joint_name='" << joint_name
                << "' does not belong to the model" << std::endl;
      failure = true;
    }
  }
  if (failure) {
    return false;
  }
  if (robot_has_free_flyer_) {
    moving_joint_ids_.push_back(
        pinocchio_model_complete.getJointId("root_joint"));
  }
  // Sort them to the pinocchio order (increasing number) and remove duplicates.
  std::sort(moving_joint_ids_.begin(), moving_joint_ids_.end());
  moving_joint_ids_.erase(
      unique(moving_joint_ids_.begin(), moving_joint_ids_.end()),
      moving_joint_ids_.end());

  // Remap the moving joint names to the Pinocchio order.
  moving_joint_names_.clear();
  moving_joint_names_.reserve(moving_joint_ids_.size());
  for (std::size_t i = 0; i < moving_joint_ids_.size(); ++i) {
    moving_joint_names_.push_back(
        pinocchio_model_complete.names[moving_joint_ids_[i]]);
  }

  // Locked joint ids in the Pinocchio order.
  locked_joint_ids_.clear();
  for (std::vector<std::string>::const_iterator it =
           pinocchio_model_complete.names.begin() + 1;
       it != pinocchio_model_complete.names.end(); ++it) {
    const std::string& joint_name = *it;
    if (std::find(moving_joint_names_.begin(), moving_joint_names_.end(),
                  joint_name) == moving_joint_names_.end()) {
      locked_joint_ids_.push_back(
          pinocchio_model_complete.getJointId(joint_name));
    }
  }

  // Remove the "root_joint" if it exists in the moving_joint_names_, we needed
  // it to create the above locked_joint_ids needed for the reduced model.
  auto root_name = std::find(moving_joint_names_.begin(),
                             moving_joint_names_.end(), "root_joint");
  if (root_name != moving_joint_names_.end()) {
    const std::size_t idx_root =
        static_cast<std::size_t>(root_name - moving_joint_names_.begin());

    // remove the name
    moving_joint_names_.erase(root_name);

    // also remove the id
    const auto root_joint_id =
        pinocchio_model_complete.getJointId("root_joint");
    auto root_id = std::find(moving_joint_ids_.begin(), moving_joint_ids_.end(),
                             root_joint_id);
    if (root_id != moving_joint_ids_.end()) {
      moving_joint_ids_.erase(root_id);
    }

    // also remove nq and nv entries
    if (idx_root < joint_nq_per_joint_.size()) {
      joint_nq_per_joint_.erase(joint_nq_per_joint_.begin() + idx_root);
      joint_nv_per_joint_.erase(joint_nv_per_joint_.begin() + idx_root);
    }
  }

  // Store joint type, nq and nv per moving joint in Pinocchio order
  joint_nq_per_joint_.reserve(moving_joint_ids_.size());
  joint_nv_per_joint_.reserve(moving_joint_ids_.size());
  joint_categories_.reserve(moving_joint_ids_.size());

  for (auto jid : moving_joint_ids_) {
    const auto& jmodel = pinocchio_model_complete.joints[jid];
    joint_nq_per_joint_.push_back(static_cast<int>(jmodel.nq()));
    joint_nv_per_joint_.push_back(static_cast<int>(jmodel.nv()));

    // Determine joint category based on joint type
    const std::string jtype = jmodel.shortname();
    if (jtype == "JointModelRX" || jtype == "JointModelRY" ||
        jtype == "JointModelRZ" || jtype == "JointModelPX" ||
        jtype == "JointModelPY" || jtype == "JointModelPZ") {
      joint_categories_.push_back(JointCategory::STANDARD_1DOF);
    } else if (jtype == "JointModelRUBX" || jtype == "JointModelRUBY" ||
               jtype == "JointModelRUBZ") {
      joint_categories_.push_back(JointCategory::CONTINUOUS_1DOF);
    } else {
      std::cerr << "RobotModelBuilder::parse_moving_joint_names(): "
                << "Unsupported joint type '" << jtype << "' for joint '"
                << pinocchio_model_complete.names[jid] << "'" << std::endl;
      return false;
    }
  }

  // Create the joint name joint id mapping of the hardware interface.
  for (std::size_t i = 0; i < moving_joint_names_.size(); ++i) {
    auto it = std::find(controlled_joint_names.begin(),
                        controlled_joint_names.end(), moving_joint_names_[i]);
    // verify that the found moving joints are part of the hardware interface.
    if (it == controlled_joint_names.end()) {
      std::cerr << "LinearFeedbackController::parseMovingJointNames(): "
                << "Moving joint " << moving_joint_names_[i]
                << " is not part of the current roscontrol hardware "
                << "interface." << std::endl;
      return false;
    }
    std::size_t it_dist =
        (size_t)std::distance(controlled_joint_names.begin(), it);
    pin_to_hwi_.emplace(std::make_pair(i, it_dist));
  }

  return true;
}

const std::vector<std::string>& RobotModelBuilder::get_moving_joint_names()
    const {
  return moving_joint_names_;
}

const std::vector<long unsigned int>& RobotModelBuilder::get_moving_joint_ids()
    const {
  return moving_joint_ids_;
}

const std::vector<long unsigned int>& RobotModelBuilder::get_locked_joint_ids()
    const {
  return locked_joint_ids_;
}

const pinocchio::Model& RobotModelBuilder::get_model() const {
  return pinocchio_model_;
}

pinocchio::Data& RobotModelBuilder::get_data() { return pinocchio_data_; }

bool RobotModelBuilder::get_robot_has_free_flyer() const {
  return robot_has_free_flyer_;
}

const std::map<int, int>&
RobotModelBuilder::get_pinocchio_to_hardware_interface_map() const {
  return pin_to_hwi_;
}

void RobotModelBuilder::construct_robot_state(
    const linear_feedback_controller_msgs::Eigen::Sensor& sensor,
    Eigen::VectorXd& robot_configuration, Eigen::VectorXd& robot_velocity) {
  assert(robot_configuration.size() == get_nq() &&
         "robot_configuration has the wrong size");
  assert(robot_velocity.size() == get_nv() &&
         "robot_velocity has the wrong size");

  robot_configuration.setZero();
  robot_velocity.setZero();

  // nq joints (Pinocchio, without free-flyer).
  const int joint_pin_nq = get_joint_pin_nq();

  // nv joints (Pinocchio, without free-flyer)
  const int joint_pin_nv = get_joint_nv();

  // Add free flyer to the state vector: x = [q, v]
  if (get_robot_has_free_flyer()) {
    robot_configuration.head<7>() = sensor.base_pose;
    robot_velocity.head<6>() = sensor.base_twist;
  }

  // Sanity checks
  assert(static_cast<int>(sensor.joint_state.position.size()) ==
             static_cast<int>(moving_joint_names_.size()) &&
         "sensor.joint_state.position size must equal number of moving joints");
  assert(static_cast<int>(sensor.joint_state.velocity.size()) ==
             static_cast<int>(moving_joint_names_.size()) &&
         "sensor.joint_state.velocity size must equal number of moving joints");

  assert(joint_categories_.size() == moving_joint_names_.size() &&
         "joint_categories_ size must equal number of moving joints");

  int iq = get_nq() - joint_pin_nq;
  int iv = get_nv() - joint_pin_nv;

  // add joints to the state vector: x = [q, v]
  for (std::size_t k = 0; k < moving_joint_names_.size(); ++k) {
    const int hwi_index = pin_to_hwi_.at(static_cast<int>(k));

    const double pos = sensor.joint_state.position[hwi_index];
    const double vel = sensor.joint_state.velocity[hwi_index];

    switch (joint_categories_[k]) {
      case JointCategory::STANDARD_1DOF:
        robot_configuration[iq] = pos;
        robot_velocity[iv] = vel;
        iq += 1;
        iv += 1;
        break;

      case JointCategory::CONTINUOUS_1DOF:
        robot_configuration[iq] = std::cos(pos);
        robot_configuration[iq + 1] = std::sin(pos);
        robot_velocity[iv] = vel;
        iq += 2;
        iv += 1;
        break;

      default:
        std::cerr << "RobotModelBuilder::construct_robot_state(): joint '"
                  << moving_joint_names_[k]
                  << "' has unknown category (should not happen)." << std::endl;
        assert(false && "Unknown joint category in construct_robot_state");
    }
  }

  // Sanity checks
  assert(iq == get_nq() &&
         "Did not fill the expected number of configuration DOFs");
  assert(iv == get_nv() && "Did not fill the expected number of velocity DOFs");
}

int RobotModelBuilder::get_nq() const { return pinocchio_model_.nq; }

int RobotModelBuilder::get_nv() const { return pinocchio_model_.nv; }

int RobotModelBuilder::get_joint_pin_nq() const {
  return std::accumulate(joint_nq_per_joint_.begin(), joint_nq_per_joint_.end(),
                         0);
}

int RobotModelBuilder::get_joint_hw_nq() const {
  // Each joint in moving_joint_names_ corresponds to exactly 1 hardware DOF
  // (whether it's a standard revolute/prismatic or a continuous joint)
  return static_cast<int>(moving_joint_names_.size());
}

int RobotModelBuilder::get_joint_nv() const {
  return std::accumulate(joint_nv_per_joint_.begin(), joint_nv_per_joint_.end(),
                         0);
}

}  // namespace linear_feedback_controller

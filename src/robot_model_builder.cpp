// #include <algorithm>
// #include <pinocchio/algorithm/compute-all-terms.hpp>
#include "linear_feedback_controller/robot_model_builder.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace linear_feedback_controller {

RobotModelBuilder::RobotModelBuilder() {}

RobotModelBuilder::~RobotModelBuilder() {}

bool RobotModelBuilder::buildModel(
    const std::string& urdf, const std::string& srdf,
    const std::vector<std::string>& moving_joint_names,
    const std::vector<std::string>& controlled_joint_names,
    bool robot_has_free_flyer) {
  // Copy the argument inside the class.
  urdf_ = urdf;
  srdf_ = srdf;
  moving_joint_names_ = moving_joint_names;
  robot_has_free_flyer_ = robot_has_free_flyer;
  controlled_joint_names_ = controlled_joint_names;

  // Build the rigid body model of the robot.
  if (robot_has_free_flyer_) {
    pinocchio::urdf::buildModelFromXML(urdf_, pinocchio::JointModelFreeFlyer(),
                                       pinocchio_model_complete_);
  } else {
    pinocchio::urdf::buildModelFromXML(urdf_, pinocchio_model_complete_);
  }
  std::istringstream iss_srdf(srdf_);
  pinocchio::srdf::loadReferenceConfigurationsFromXML(pinocchio_model_complete_,
                                                      iss_srdf, false);

  // Reduce the rigid body model and set initial position.
  if (!parseMovingJointNames(moving_joint_names_, controlled_joint_names_)) {
    return false;
  }

  q_default_complete_ =
      pinocchio_model_complete_.referenceConfigurations["half_sitting"];
  pinocchio_model_reduced_ = pinocchio::buildReducedModel(
      pinocchio_model_complete_, locked_joint_ids_, q_default_complete_);
  pinocchio_data_reduced_ = pinocchio::Data(pinocchio_model_reduced_);

  return true;
}

bool RobotModelBuilder::parseMovingJointNames(
    const std::vector<std::string>& moving_joint_names,
    const std::vector<std::string>& controlled_joint_names) {
  // Get moving joints ids
  moving_joint_ids_.clear();
  for (const auto& joint_name : moving_joint_names) {
    pinocchio::JointIndex joint_id = 0;
    // do not consider joint that are not in the model
    if (pinocchio_model_complete_.existJointName(joint_name)) {
      joint_id = pinocchio_model_complete_.getJointId(joint_name);
      moving_joint_ids_.push_back(joint_id);
    } else {
      std::cerr << "joint_name=" << joint_name
                << " does not belong to the model" << std::endl;
    }
  }
  if (robot_has_free_flyer_) {
    moving_joint_ids_.push_back(
        pinocchio_model_complete_.getJointId("root_joint"));
  }
  // Sort them to the pinocchio order (increasing number) and remove duplicates.
  std::sort(moving_joint_ids_.begin(), moving_joint_ids_.end());
  moving_joint_ids_.erase(
      unique(moving_joint_ids_.begin(), moving_joint_ids_.end()),
      moving_joint_ids_.end());

  // Remap the moving joint names to the Pinocchio order.
  moving_joint_names_.clear();
  for (std::size_t i = 0; i < moving_joint_ids_.size(); ++i) {
    moving_joint_names_.push_back(
        pinocchio_model_complete_.names[moving_joint_ids_[i]]);
  }

  // Locked joint ids in the Pinocchio order.
  locked_joint_ids_.clear();
  for (std::vector<std::string>::const_iterator it =
           pinocchio_model_complete_.names.begin() + 1;
       it != pinocchio_model_complete_.names.end(); ++it) {
    const std::string& joint_name = *it;
    if (std::find(moving_joint_names_.begin(), moving_joint_names_.end(),
                  joint_name) == moving_joint_names_.end()) {
      locked_joint_ids_.push_back(
          pinocchio_model_complete_.getJointId(joint_name));
    }
  }

  // remove the "root_joint" if it exists in the moving_joint_names_, we needed
  // it to create the above locked_joint_ids needed for the reduced model.
  auto root_name = std::remove(moving_joint_names_.begin(),
                               moving_joint_names_.end(), "root_joint");
  if (root_name != moving_joint_names_.end()) {
    moving_joint_names_.erase(root_name, moving_joint_names_.end());
    auto root_id =
        std::remove(moving_joint_ids_.begin(), moving_joint_ids_.end(),
                    pinocchio_model_complete_.getJointId("root_joint"));
    moving_joint_ids_.erase(root_id, moving_joint_ids_.end());
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
  // Double check what we just did
  for (std::size_t i = 0; i < moving_joint_names_.size(); ++i) {
    if (moving_joint_names_[i] != controlled_joint_names[pin_to_hwi_[i]]) {
      std::cerr << "LinearFeedbackController::parseMovingJointNames(): "
                << "The mapping from pinocchio to roscontrol is wrong. "
                << "moving_joint_names_[i] != "
                << "controlled_joint_names[pin_to_hwi_[i]] => "
                << moving_joint_names_[i]
                << " != " << controlled_joint_names[pin_to_hwi_[i]];
      return false;
    }
  }
  return true;
}

const std::vector<std::string>& RobotModelBuilder::getMovingJointNames() const {
  return moving_joint_names_;
}

const std::vector<long unsigned int>& RobotModelBuilder::getMovingJointIds()
    const {
  return moving_joint_ids_;
}

const std::vector<long unsigned int>& RobotModelBuilder::getLockedJointIds()
    const {
  return locked_joint_ids_;
}

const std::string& RobotModelBuilder::getUrdf() const { return urdf_; }

const std::string& RobotModelBuilder::getSrdf() const { return srdf_; }

const pinocchio::Model& RobotModelBuilder::getPinocchioModel() const {
  return pinocchio_model_reduced_;
}

pinocchio::Data& RobotModelBuilder::getPinocchioData() {
  return pinocchio_data_reduced_;
}

bool RobotModelBuilder::getRobotHasFreeFlyer() { return robot_has_free_flyer_; }

std::map<int, int> RobotModelBuilder::getPinocchioToHarwdareInterfaceMap() {
  return pin_to_hwi_;
}

}  // namespace linear_feedback_controller

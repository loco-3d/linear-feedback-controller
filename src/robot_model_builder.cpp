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

bool RobotModelBuilder::build_model(
    const std::string& in_urdf, const std::string& in_srdf,
    const std::vector<std::string>& in_moving_joint_names,
    bool in_robot_has_free_flyer,
    const std::vector<std::string>& in_controlled_joint_names) {
  // Copy the argument inside the class.
  in_urdf_ = in_urdf;
  in_srdf_ = in_srdf;
  in_moving_joint_names_ = in_moving_joint_names;
  in_robot_has_free_flyer_ = in_robot_has_free_flyer;
  in_controlled_joint_names_ = in_controlled_joint_names;

  // Build the rigid body model of the robot.
  if (in_robot_has_free_flyer_) {
    pinocchio::urdf::buildModelFromXML(
        in_urdf_, pinocchio::JointModelFreeFlyer(), pinocchio_model_complete_);
  } else {
    pinocchio::urdf::buildModelFromXML(in_urdf_, pinocchio_model_complete_);
  }
  std::istringstream iss_srdf(in_srdf_);
  pinocchio::srdf::loadReferenceConfigurationsFromXML(pinocchio_model_complete_,
                                                      iss_srdf, false);

  // Reduce the rigid body model and set initial position.
  if (!parseMovingJointNames(in_moving_joint_names_, in_controlled_joint_names_,
                             moving_joint_names_, moving_joint_ids_,
                             locked_joint_ids_)) {
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
    const std::vector<std::string>& in_moving_joint_names,
    const std::vector<std::string>& in_controlled_joint_names,
    std::vector<std::string>& moving_joint_names,
    std::vector<pinocchio::Index>& moving_joint_ids,
    std::vector<pinocchio::Index>& locked_joint_ids) {
  // Get moving joints ids
  moving_joint_ids.clear();
  for (std::vector<std::string>::const_iterator it =
           in_moving_joint_names.begin();
       it != in_moving_joint_names.end(); ++it) {
    const std::string& joint_name = *it;
    pinocchio::JointIndex joint_id = 0;
    // do not consider joint that are not in the model
    if (pinocchio_model_complete_.existJointName(joint_name)) {
      joint_id = pinocchio_model_complete_.getJointId(joint_name);
      moving_joint_ids.push_back(joint_id);
    } else {
      std::cerr << "joint_name=" << joint_name
                << " does not belong to the model" << std::endl;
    }
  }
  if (in_robot_has_free_flyer_) {
    moving_joint_ids.push_back(
        pinocchio_model_complete_.getJointId("root_joint"));
  }
  // Sort them to the pinocchio order (increasing number) and remove duplicates.
  std::sort(moving_joint_ids.begin(), moving_joint_ids.end());
  moving_joint_ids.erase(
      unique(moving_joint_ids.begin(), moving_joint_ids.end()),
      moving_joint_ids.end());

  // Remap the moving joint names to the Pinocchio order.
  moving_joint_names.clear();
  for (std::size_t i = 0; i < moving_joint_ids.size(); ++i) {
    moving_joint_names.push_back(
        pinocchio_model_complete_.names[moving_joint_ids[i]]);
  }

  // Locked joint ids in the Pinocchio order.
  locked_joint_ids.clear();
  for (std::vector<std::string>::const_iterator it =
           pinocchio_model_complete_.names.begin() + 1;
       it != pinocchio_model_complete_.names.end(); ++it) {
    const std::string& joint_name = *it;
    if (std::find(moving_joint_names.begin(), moving_joint_names.end(),
                  joint_name) == moving_joint_names.end()) {
      locked_joint_ids.push_back(
          pinocchio_model_complete_.getJointId(joint_name));
    }
  }

  // remove the "root_joint" if it exists in the moving_joint_names, we needed
  // it to create the above locked_joint_ids needed for the reduced model.
  auto root_name = std::remove(moving_joint_names.begin(),
                               moving_joint_names.end(), "root_joint");
  if (root_name != moving_joint_names.end()) {
    moving_joint_names.erase(root_name, moving_joint_names.end());
    auto root_id =
        std::remove(moving_joint_ids.begin(), moving_joint_ids.end(),
                    pinocchio_model_complete_.getJointId("root_joint"));
    moving_joint_ids.erase(root_id, moving_joint_ids.end());
  }

  // Create the joint name joint id mapping of the hardware interface.
  for (std::size_t i = 0; i < moving_joint_names.size(); ++i) {
    auto it = std::find(in_controlled_joint_names.begin(),
                        in_controlled_joint_names.end(), moving_joint_names[i]);
    // verify that the found moving joints are part of the hardware interface.
    if (it == in_controlled_joint_names.end()) {
      std::cerr << "LinearFeedbackController::parseMovingJointNames(): "
                << "Moving joint " << moving_joint_names[i]
                << " is not part of the current roscontrol hardware "
                << "interface." << std::endl;
      return false;
    }
    std::size_t it_dist =
        (size_t)std::distance(in_controlled_joint_names.begin(), it);
    pin_to_hwi_.emplace(std::make_pair(i, it_dist));
  }
  // Double check what we just did
  for (std::size_t i = 0; i < moving_joint_names.size(); ++i) {
    if (moving_joint_names[i] != in_controlled_joint_names[pin_to_hwi_[i]]) {
      std::cerr << "LinearFeedbackController::parseMovingJointNames(): "
                << "The mapping from pinocchio to roscontrol is wrong. "
                << "moving_joint_names[i] != "
                << "in_controlled_joint_names[pin_to_hwi_[i]] => "
                << moving_joint_names[i]
                << " != " << in_controlled_joint_names[pin_to_hwi_[i]];
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

const std::string& RobotModelBuilder::getUrdf() const { return in_urdf_; }

const pinocchio::Model& RobotModelBuilder::getPinocchioModel() const {
  return pinocchio_model_reduced_;
}

pinocchio::Data& RobotModelBuilder::getPinocchioData() {
  return pinocchio_data_reduced_;
}

}  // namespace linear_feedback_controller

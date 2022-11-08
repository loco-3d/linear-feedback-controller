#ifndef __free_flyer_model_utils_hpp__
#define __free_flyer_model_utils_hpp__

#include <math.h> 
#include <boost/algorithm/string.hpp>
#include <algorithm>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/parsers/srdf.hpp>

#include <crocoddyl/multibody/states/multibody.hpp>
#include <crocoddyl/multibody/actions/free-fwddyn.hpp>
#include <crocoddyl/multibody/actuations/full.hpp>
#include <crocoddyl/multibody/frames.hpp>
#include <crocoddyl/multibody/residuals/state.hpp>
#include <crocoddyl/multibody/residuals/frame-translation.hpp>
#include <crocoddyl/multibody/residuals/frame-placement.hpp>
#include <crocoddyl/multibody/residuals/contact-control-gravity.hpp>
#include <crocoddyl/multibody/contacts/multiple-contacts.hpp>
#include <crocoddyl/multibody/residuals/com-position.hpp>
#include <crocoddyl/multibody/residuals/frame-velocity.hpp>
#include <crocoddyl/multibody/residuals/contact-wrench-cone.hpp>
#include <crocoddyl/multibody/contacts/contact-3d.hpp>
#include <crocoddyl/multibody/contacts/contact-6d.hpp>
#include <crocoddyl/multibody/actuations/floating-base.hpp>
#include <crocoddyl/multibody/actions/contact-fwddyn.hpp>
#include <crocoddyl/multibody/residuals/frame-rotation.hpp>
#include <crocoddyl/multibody/residuals/contact-force.hpp>
#include <crocoddyl/multibody/actions/impulse-fwddyn.hpp>
#include <crocoddyl/multibody/impulses/impulse-6d.hpp>

#include <crocoddyl/core/fwd.hpp>
#include <crocoddyl/core/optctrl/shooting.hpp>
#include <crocoddyl/core/solver-base.hpp>
#include <crocoddyl/core/utils/exception.hpp>
#include <crocoddyl/core/activations/quadratic-barrier.hpp>
#include <crocoddyl/core/activations/quadratic-flat-log.hpp>
#include <crocoddyl/core/activations/weighted-quadratic.hpp>
#include <crocoddyl/core/integrator/euler.hpp>
#include <crocoddyl/core/action-base.hpp>
#include <crocoddyl/core/solvers/fddp.hpp>
#include <crocoddyl/core/residuals/control.hpp>
#include <crocoddyl/core/action-base.hpp>

#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <memmo_trajectory_controller/JointState.h>
#include <memmo_trajectory_controller/Config.h>


namespace model_utils{
  using namespace crocoddyl;
  class FreeFlyerModelUtils{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    public:
    FreeFlyerModelUtils(const std::vector<std::string> controlled_joints_names,
               const std::string urdf_filename,
               const std::string srdf_filename);
    FreeFlyerModelUtils();
    pinocchio::Model createPinocchioModel();
    void buildStateMap(sensor_msgs::JointStateConstPtr actual_joint_state);
    void mapToPinocchio(sensor_msgs::JointStateConstPtr actual_joint_state,
                        nav_msgs::OdometryConstPtr actual_base_state);
    void mapToTF(const Eigen::VectorXd us0, 
                 const Eigen::MatrixXd K0,
                 const Eigen::VectorXd ddq = Eigen::VectorXd());
    void mapToTF(const Eigen::VectorXd us0,
                 const Eigen::MatrixXd K0,
                 const Eigen::MatrixXd dKp,
                 const Eigen::VectorXd mocap_target_pos);
    
    pinocchio::Model pin_model_;
    std::vector<std::string> controlled_joints_names_;
    std::vector<Eigen::Index> controlled_joints_id_;
    std::map<std::string, size_t> actual_state_map_;
    std::string urdf_filename_;
    std::string srdf_filename_;
    std::string robot_description_;
    
    Eigen::Vector3d base_linear_vel_;
    Eigen::Vector3d base_angular_vel_;
    Eigen::Quaterniond base_orientation_;
    Eigen::Vector3d base_position_;
    Eigen::VectorXd x_current_;
    memmo_trajectory_controller::JointState joints_state_;
};

}

#endif


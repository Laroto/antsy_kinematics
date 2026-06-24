#include "antsy_kinematics/kinematics.hpp"

#include <cassert>
#include <cmath>
#include <stdexcept>
#include <thread>

#include "rclcpp/qos.hpp"
#include "kdl/chain.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/joint.hpp"
#include "kdl_parser/kdl_parser.hpp"

namespace antsy_kinematics
{
using std::placeholders::_1;

Kinematics::Kinematics(
  const std::vector<std::string>& feet_links,
  const std::string& base_link,
  const double position_weight,
  const double orientation_weight)
{
  if (position_weight <= 0.0 || orientation_weight < 0.0) {
    throw std::runtime_error(
      "IK task weights must have positive position weight and non-negative orientation weight.");
  }
  feet_links_ = feet_links;
  base_link_ = base_link;
  position_weight_ = position_weight;
  orientation_weight_ = orientation_weight;
}

void validate_limits(const rclcpp::Logger& logger, const std::vector<urdf::JointLimits> & limits)
{
  for (size_t i = 0; i < limits.size(); i++) {
    RCLCPP_INFO(logger,
      "IK: Joint %d has limits: [%f, %f].",
      static_cast<int>(i), limits[i].lower, limits[i].upper
    );
    assert(limits[i].upper >= limits[i].lower);
  }
}

void Kinematics::robotDescriptionCallback(const std_msgs::msg::String& msg)
{
  solvers_set_ = false;

  // Build KDL::Tree
  bool success = kdl_parser::treeFromString(msg.data, tree_);
  if (success) {
    RCLCPP_INFO(rclcpp::get_logger("antsy_kinematics"),
      "IK: Constructed KDL tree from URDF with %d joints and %d segments.",
      tree_.getNrOfJoints(), tree_.getNrOfSegments());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("antsy_kinematics"), "IK: Failed to construct KDL tree from URDF.");
    return;
  }

  // Extract chains from tree
  // TODO do this in a way that doesn't involve duplicating them from the tree?
  chains_.clear();
  chains_.resize(feet_links_.size());

  bool all_chains_extracted = true;
  for (size_t i = 0; i < feet_links_.size(); i++) {
    // Extract chain of interest from tree
    bool success = tree_.getChain(base_link_, feet_links_[i], std::ref(chains_[i]));
    if (success) {
      RCLCPP_INFO(rclcpp::get_logger("antsy_kinematics"),
        "IK: Extracted chain %d with %d joints and %d segments to link %s.",
        static_cast<int>(i), chains_[i].getNrOfJoints(),
        chains_[i].getNrOfSegments(), feet_links_[i].c_str());
    } else {
      all_chains_extracted = false;
      RCLCPP_ERROR(rclcpp::get_logger("antsy_kinematics"),
        "IK: Failed to extract chain %d: %s => %s.",
        static_cast<int>(i), base_link_.c_str(), feet_links_[i].c_str());
      continue;
    }
  }
  if (!all_chains_extracted) {
    return;
  }

  // Extract joint limits
  // not supported by KDL so we'll parse them manually
  // first extract joint names from KDL, then look up their limits
  // (to ensure we get the same sequence)
  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(msg.data);
  if (!urdf) {
    RCLCPP_ERROR(rclcpp::get_logger("antsy_kinematics"), "IK: Failed to parse URDF joint limits.");
    return;
  }

  joint_limits_.clear();
  joint_limits_.resize(chains_.size());

  for (size_t i = 0; i < chains_.size(); i++) {
    for (size_t j = 0; j < chains_[i].getNrOfSegments(); j++) {
      auto joint = chains_[i].getSegment(j).getJoint();
      
      if (joint.getType() != KDL::Joint::JointType::Fixed) {
        auto urdf_joint = urdf->getJoint(joint.getName());
        if (!urdf_joint || !urdf_joint->limits) {
          RCLCPP_ERROR(rclcpp::get_logger("antsy_kinematics"),
            "IK: Joint %s is missing limits in the URDF.",
            joint.getName().c_str());
          return;
        }

        auto lims = urdf_joint->limits;
        joint_limits_[i].push_back(*lims);
      }
    }
    joint_limits_[i].shrink_to_fit();
    if (joint_limits_[i].size() != chains_[i].getNrOfJoints()) {
      RCLCPP_ERROR(rclcpp::get_logger("antsy_kinematics"),
        "IK: Chain %d has %d joints but %d joint limits.",
        static_cast<int>(i),
        static_cast<int>(chains_[i].getNrOfJoints()),
        static_cast<int>(joint_limits_[i].size()));
      return;
    }
    validate_limits(rclcpp::get_logger("antsy_kinematics"), joint_limits_[i]);
  }
  // With all information gathered, initialize the solvers
  createSolvers();
}

void Kinematics::createSolvers()
{
  Eigen::Matrix<double, 6, 1> task_weights;
  task_weights << position_weight_, position_weight_, position_weight_,
    orientation_weight_, orientation_weight_, orientation_weight_;

  solvers_.clear();
  solvers_.reserve(feet_links_.size());
  for (size_t i = 0; i < feet_links_.size(); i++) {
    // Create IK solver
    solvers_.emplace_back(std::cref(chains_[i]), task_weights, 1e-5, 100, 1e-15);
  }
  solvers_set_ = true;
}

int Kinematics::cartToJnt(
  const size_t leg_index, const KDL::JntArray& q_init,
  const KDL::Frame& T_base_goal, KDL::JntArray& q_out)
{
  if (!solvers_set_) {
    RCLCPP_ERROR(rclcpp::get_logger("antsy_kinematics"),
      "IK: Solvers not yet set, have to wait until initialized!");
    return -1;
  }
  return solvers_[leg_index].CartToJnt(q_init, T_base_goal, q_out);
}

bool Kinematics::foldAndClampJointAnglesToLimits(
  const size_t leg_index, KDL::JntArray & q)
{
  bool clamping_applied = false;
  for (size_t i = 0; i < q.rows(); i++) {
    urdf::JointLimits jl = joint_limits_[leg_index][i];
    if (q(i) > jl.upper) {
      clamping_applied = true;
      q(i) = jl.upper;
    } else if (q(i) < jl.lower) {
      clamping_applied = true;
      q(i) = jl.lower;
    }
  }
  return clamping_applied;
}

bool Kinematics::isInitialized() const {
  return solvers_set_;
}

}  // namespace antsy_kinematics

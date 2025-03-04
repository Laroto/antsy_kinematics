#include "antsy_kinematics/kinematics.hpp"

#include <math.h>

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
  const std::string& base_link)
: Node("antsy_kinematics")
{
  feet_links_ = feet_links;
  base_link_ = base_link;
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&Kinematics::robotDescriptionCallback, this, _1));
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
  // Build KDL::Tree
  bool tree_was_not_empty = tree_.getNrOfSegments() > 0;
  bool success = kdl_parser::treeFromString(msg.data, tree_);
  if (success) {
    RCLCPP_INFO(this->get_logger(),
      "IK: Constructed KDL tree from URDF with %d joints and %d segments.",
      tree_.getNrOfJoints(), tree_.getNrOfSegments());
  } else {
    RCLCPP_ERROR(this->get_logger(), "IK: Failed to construct KDL tree from URDF.");
    return;
  }
  if (tree_was_not_empty) {
    createSolvers();
    RCLCPP_INFO(this->get_logger(),
      "IK: Received a new URDF, processed it and rebuilt the solvers.");
  }
  // Extract chains from tree
  // TODO do this in a way that doesn't involve duplicating them from the tree?
  chains_.clear();
  chains_.resize(feet_links_.size());
  
  for (size_t i = 0; i < feet_links_.size(); i++) {
    // Extract chain of interest from tree
    bool success = tree_.getChain(base_link_, feet_links_[i], std::ref(chains_[i]));
    if (success) {
      RCLCPP_INFO(this->get_logger(),
        "IK: Extracted chain %d with %d joints and %d segments to link %s.",
        static_cast<int>(i), chains_[i].getNrOfJoints(),
        chains_[i].getNrOfSegments(), feet_links_[i].c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(),
        "IK: Failed to extract chain %d: %s => %s.",
        static_cast<int>(i), base_link_.c_str(), feet_links_[i].c_str());
      continue;
    }
  }
  // Extract joint limits
  // not supported by KDL so we'll parse them manually
  // first extract joint names from KDL, then look up their limits
  // (to ensure we get the same sequence)
  urdf::ModelInterfaceSharedPtr urdf = urdf::parseURDF(msg.data);
  joint_limits_.clear();
  joint_limits_.resize(chains_.size());

  for (size_t i = 0; i < chains_.size(); i++) {
    for (size_t j = 0; j < chains_[i].getNrOfSegments(); j++) {
      auto joint = chains_[i].getSegment(j).getJoint();
      
      if (joint.getType() != KDL::Joint::JointType::Fixed) {
        // joint_limits_[i].push_back(*(urdf->getJoint(joint.getName())->limits));

        auto lims = urdf->getJoint(joint.getName())->limits;
        // RCLCPP_INFO(this->get_logger(),"IK: Joint %s has limits: [%f, %f].",joint.getName().c_str(),lims->lower, lims->upper);
        joint_limits_[i].push_back(*lims);

        // if (joint_limits_[i][j].upper < joint_limits_[i][j].lower) {
        //   RCLCPP_WARN(this->get_logger(),
        //     "IK: Joint %s has upper limit less than lower limit: %f < %f.",
        //     joint.getName().c_str(),
        //     joint_limits_[i][j].upper, joint_limits_[i][j].lower
        //     );
        // }
      }
    }
    joint_limits_[i].shrink_to_fit();
    validate_limits(this->get_logger(), joint_limits_[i]);
  }
  // With all information gathered, initialize the solvers
  createSolvers();
}

void Kinematics::createSolvers()
{
  solvers_.clear();
  solvers_.reserve(feet_links_.size());
  for (size_t i = 0; i < feet_links_.size(); i++) {
    // Create IK solver
    solvers_.emplace_back(std::cref(chains_[i]), 1e-5, 100, 1e-15);
  }
  solvers_set_ = true;
}

void Kinematics::spinUntilInitialized()
{
  // TODO do this more elegantly (without polling)?
  while (!solvers_set_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "IK: Waiting until URDF received and solvers initialized.");
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    rclcpp::spin_some(this->get_node_base_interface());
  }
}

int Kinematics::cartToJnt(
  const size_t leg_index, const KDL::JntArray& q_init,
  const KDL::Frame& T_base_goal, KDL::JntArray& q_out)
{
  if (!solvers_set_) {
    RCLCPP_ERROR(this->get_logger(),
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
    // Fold
    while (q(i) > jl.upper) q(i) -= 2*M_PI;
    while (q(i) < jl.lower) q(i) += 2*M_PI;
    // Check whether clamping is required, and if so, clamp to nearest
    if (q(i) > jl.upper) {
      clamping_applied = true;
      const float upper_distance = fmod(q(i) - jl.upper, 2*M_PI);
      const float lower_distance = fmod(jl.lower - q(i), 2*M_PI);
      q(i) = upper_distance > lower_distance ? jl.lower : jl.upper;
    }
  }
  return clamping_applied;
}
}  // namespace hexapod_kinematics

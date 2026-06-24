#ifndef ANTSY__KINEMATICS_HPP_
#define ANTSY__KINEMATICS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "urdf_parser/urdf_parser.h"
#include "kdl/tree.hpp"
#include "kdl/chainiksolverpos_lma.hpp"
#include "std_msgs/msg/string.hpp"


namespace antsy_kinematics
{
class Kinematics
{
public:
  Kinematics() = default;
  Kinematics(
    const std::vector<std::string>& feet_links,
    const std::string& base_link="base_link",
    double position_weight=1.0,
    double orientation_weight=0.0);
  // TODO use more suitable type for goal_pos?
  int cartToJnt(
    const size_t leg_index, const KDL::JntArray& q_init,
    const KDL::Frame& T_base_goal, KDL::JntArray& q_out);
  bool foldAndClampJointAnglesToLimits(
    const size_t leg_index, KDL::JntArray & q);
  // check whether solvers have been initialized after robot_description
  bool isInitialized() const;

 public:
  // Called by an external node subscription when robot_description arrives
  void robotDescriptionCallback(const std_msgs::msg::String& msg);
  
 private:
  void createSolvers();
  KDL::Tree tree_;
  std::vector<std::string> feet_links_;
  std::string base_link_;
  double position_weight_ = 1.0;
  double orientation_weight_ = 0.0;
  // TODO use smart pointers to handle chains and solvers?
  std::vector<KDL::Chain> chains_;
  std::vector<KDL::ChainIkSolverPos_LMA> solvers_;
  std::vector<std::vector<urdf::JointLimits>> joint_limits_;
  bool solvers_set_ = false;
};
}   // namespace antsy_kinematics

#endif  // ANTSY__KINEMATICS_HPP_

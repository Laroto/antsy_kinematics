#include "antsy_kinematics/kinematics.hpp"

#include "rclcpp/rclcpp.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::placeholders;

class ExampleNode : public rclcpp::Node
{
public:
  ExampleNode()
  : Node("example_node_kinematics_usage")
  {
  }
  void run()
  {
    // Start listening to URDF, create KDL tree,
    // extract chains and construct a solver for each
    antsy_kinematics::Kinematics kinematics(
      std::vector<std::string>{"foot_0", "foot_1", "foot_2"});
    // forward robot_description to kinematics helper and wait for init
    auto sub = this->create_subscription<std_msgs::msg::String>(
      "robot_description",
      rclcpp::QoS(rclcpp::KeepLast(1)).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&antsy_kinematics::Kinematics::robotDescriptionCallback, &kinematics, _1));
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "IK: Waiting until URDF received and solvers initialized.");
    while (!kinematics.isInitialized()) {
      rclcpp::spin_some(this->get_node_base_interface());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    // Run the solver once
    KDL::JntArray q_init(3);
    q_init(0) = 0;
    q_init(1) = 0.1;
    q_init(2) = -0.2;
    KDL::Frame p_in(KDL::Vector(0.16, 0.24, -0.03));
    KDL::JntArray q_out(3);
    int result = kinematics.cartToJnt(0, q_init, p_in, q_out);
    bool clamped = kinematics.foldAndClampJointAnglesToLimits(0, q_out);
    std::cout << "solver return: " << result << std::endl;
    std::cout << "clamped:       " << clamped << std::endl;
    std::cout << "joint angles [deg]: " << q_out(0)*180/M_PI << ", "
                                        << q_out(1)*180/M_PI << ", "
                                        << q_out(2)*180/M_PI << std::endl;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  node->run();
  rclcpp::shutdown();
  return 0;
}

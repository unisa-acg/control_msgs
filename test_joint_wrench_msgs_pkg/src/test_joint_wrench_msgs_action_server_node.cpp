#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/msg/joint_wrench_trajectory.hpp>
#include <control_msgs/msg/joint_wrench_trajectory_point.hpp>
#include <control_msgs/action/follow_joint_wrench_trajectory.hpp>

int main([[maybe_unused]] int argc, [[maybe_unused]] char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("test_joint_wrench_msgs_node", options);

  // Create an action server that receives JointWrenchTrajectory messages and logs them
  auto action_server = rclcpp_action::create_server<control_msgs::action::FollowJointWrenchTrajectory>(
    node,
    "joint_wrench_trajectory",
    [node](const rclcpp_action::GoalUUID & /*uuid*/,
           std::shared_ptr<const control_msgs::action::FollowJointWrenchTrajectory::Goal> goal) {
      RCLCPP_INFO(node->get_logger(), "Received goal with %zu joints and %zu points.",
                  goal->trajectory.joint_names.size(), goal->trajectory.points.size());
      for (size_t i = 0; i < goal->trajectory.points.size(); ++i) {
        const auto & point = goal->trajectory.points[i];
        RCLCPP_INFO(node->get_logger(), "Point %zu: time_from_start = %f, force.x = %.2f, torque.y = %.2f",
                    i,
                    point.point.time_from_start.sec + point.point.time_from_start.nanosec / 1e9,
                    point.wrench.force.x, point.wrench.torque.y);
        for (std::size_t j = 0; j < goal->trajectory.joint_names.size(); ++j) {
          RCLCPP_INFO(node->get_logger(), "%s, position: %f",goal->trajectory.joint_names[j].c_str(),
                      goal->trajectory.points[i].point.positions[j]);
        }
      }
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    },
    [node](const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointWrenchTrajectory>> /* goal_handle */) {
      RCLCPP_INFO(node->get_logger(), "Goal canceled");
      return rclcpp_action::CancelResponse::ACCEPT;
    },
    [node](const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointWrenchTrajectory>> goal_handle) {
      auto result = std::make_shared<control_msgs::action::FollowJointWrenchTrajectory::Result>();
      result->error_code = control_msgs::action::FollowJointWrenchTrajectory::Result::SUCCESSFUL;
      goal_handle->succeed(result);
      RCLCPP_INFO(node->get_logger(), "Goal executed with success");
      return rclcpp_action::ResultCode::SUCCEEDED;
    });
  RCLCPP_INFO(node->get_logger(), "Action server started, waiting for goals...");

  // Spin the node for 1 min, then shutdown
  rclcpp::WallRate rate(1);
  for (int i = 0; i < 60 && rclcpp::ok(); ++i) {
    rclcpp::spin_some(node);
    rate.sleep();
  }
  RCLCPP_INFO(node->get_logger(), "Shutting down action server...");
  rclcpp::shutdown();
  return 0;
}

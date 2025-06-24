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
  auto node = rclcpp::Node::make_shared("test_joint_wrench_msgs_node", options);

  RCLCPP_INFO(node->get_logger(), "Testing JointWrenchTrajectory message...");
  control_msgs::msg::JointWrenchTrajectory msg;
  msg.header.stamp = node->now();
  msg.header.frame_id = "base_link";
  msg.joint_names.push_back("joint1");
  msg.joint_names.push_back("joint2");
  msg.points.resize(2);
  msg.points[0].point.time_from_start = rclcpp::Duration(1, 0);
  msg.points[0].wrench.force.x = 1.0;
  msg.points[0].wrench.torque.y = 2.0;
  msg.points[0].point.positions.push_back(0.5);
  msg.points[0].point.positions.push_back(1.5);
  msg.points[1].point.time_from_start = rclcpp::Duration(2, 0);
  msg.points[1].wrench.force.x = 3.0;
  msg.points[1].wrench.torque.y = 4.0;
  msg.points[1].point.positions.push_back(2.5);
  msg.points[1].point.positions.push_back(3.5);

  // Create an action client to test the JointWrenchTrajectory message
  auto action_client = rclcpp_action::create_client<control_msgs::action::FollowJointWrenchTrajectory>(
    node, "joint_wrench_trajectory");
  if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Action server not available after waiting");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Action server available, sending goal...");
  auto goal_msg = control_msgs::action::FollowJointWrenchTrajectory::Goal();
  goal_msg.trajectory = msg;
  auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointWrenchTrajectory>::SendGoalOptions();
  send_goal_options.result_callback =
    [](const rclcpp_action::ClientGoalHandle<control_msgs::action::FollowJointWrenchTrajectory>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal succeeded");
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Goal failed with code: %d", static_cast<int>(result.code));
      }
    };
  auto goal_handle_future = action_client->async_send_goal(goal_msg, send_goal_options);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    return 1;
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected by the action server");
    return 1;
  }
  RCLCPP_INFO(node->get_logger(), "Goal accepted by the action server");
  // Wait for the result
  auto result_future = action_client->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to get result");
    return 1;
  }
  auto result = result_future.get();
  if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_INFO(node->get_logger(), "Action completed successfully");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Action failed with code: %d", static_cast<int>(result.code));
  }
  RCLCPP_INFO(node->get_logger(), "Test completed successfully.");
  rclcpp::shutdown();
  return 0;
}

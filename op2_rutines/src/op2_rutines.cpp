#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include "op2_msgs/action/op2_task.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <memory>


using namespace std::placeholders;

namespace op2_rutines
{
class Op2Task : public rclcpp::Node
{
public:
  explicit Op2Task(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("op2_rutines", options)
  {
    RCLCPP_INFO(get_logger(), "Starting the Server");
    action_server_ = rclcpp_action::create_server<op2_msgs::action::Op2Task>(
        this, "op2_rutines", std::bind(&Op2Task::goalCallback, this, _1, _2),
        std::bind(&Op2Task::cancelCallback, this, _1),
        std::bind(&Op2Task::acceptedCallback, this, _1));
  }

private:
  rclcpp_action::Server<op2_msgs::action::Op2Task>::SharedPtr action_server_;

  rclcpp_action::GoalResponse goalCallback(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const op2_msgs::action::Op2Task::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal request with id %d", goal->task_number);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancelCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<op2_msgs::action::Op2Task>> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(get_logger(), "Received request to cancel goal");
    auto head_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "head");
    auto r_arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "r_arm");
    auto l_arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "l_arm");

    head_move_group.stop();
    r_arm_move_group.stop();
    l_arm_move_group.stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void acceptedCallback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<op2_msgs::action::Op2Task>> goal_handle)
  {
    std::thread{ std::bind(&Op2Task::execute, this, _1), goal_handle }.detach();
  }

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<op2_msgs::action::Op2Task>> goal_handle)
{
  RCLCPP_INFO(get_logger(), "Executing goal");
  auto result = std::make_shared<op2_msgs::action::Op2Task::Result>();

  // MoveIt 2 Interface
  auto head_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "head");
  auto r_arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "r_arm");
  auto l_arm_move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "l_arm");

  std::vector<double> head_joint_goal;
  std::vector<double> r_arm_joint_goal;
  std::vector<double> l_arm_joint_goal;

  // Adjust joint goals according to your requirements
  // pi/2  25pi/180  pi/6  pi/12
  if (goal_handle->get_goal()->task_number == 0)
  {
    head_joint_goal = {0.0, 0.0};  // Head joint goals
    r_arm_joint_goal = {0.0, 0.0, 0.0};  // Right arm joint goals
    l_arm_joint_goal = {0.0, 0.0, 0.0};
  }
  else if (goal_handle->get_goal()->task_number == 1)
  {
    head_joint_goal = {0.0, 0.0};  // Head joint goals
    r_arm_joint_goal = {3.14 / 2.0, 0.0, 25.0 * 3.1416/180.0};  // Right arm joint goals
    l_arm_joint_goal = {0.0, 0.0, 0.0}; // Left arm joint goals
  }
  else if (goal_handle->get_goal()->task_number == 2)
  {
    head_joint_goal = {0.0, 0.0};  // Head joint goals
    r_arm_joint_goal = {3.14 / 2.0, 0.0, -25.0 * 3.1416/180.0};  // Right arm joint goals
    l_arm_joint_goal = {0.0, 0.0, 0.0};  // Left arm joint goals
  }
  else if (goal_handle->get_goal()->task_number == 3)
  {
    head_joint_goal = {0.0, 0.0};  // Head joint goals
    r_arm_joint_goal = {0.0, 0.0, 0.0};  // Right arm joint goals
    l_arm_joint_goal = {-3.14 / 2.0, 0.0, -25.0 * 3.1416/180.0}; // Left arm joint goals
  }
  else if (goal_handle->get_goal()->task_number == 4)
  {
    head_joint_goal = {0.0, 0.0};  // Head joint goals
    r_arm_joint_goal = {0.0, 0.0, 0.0};  // Right arm joint goals
    l_arm_joint_goal = {-3.14 / 2.0, 0.0, 25.0 * 3.1416/180.0};  // Left arm joint goals
  }
  else if (goal_handle->get_goal()->task_number == 5)
  {
    head_joint_goal = {0.0, 3.1416 / 6.0};  // Head joint goals
    r_arm_joint_goal = {0.0, 0.0, 0.0};  // Right arm joint goals
    l_arm_joint_goal = {0.0, 0.0, 0.0}; // Left arm joint goals
  }
  else if (goal_handle->get_goal()->task_number == 6)
  {
    head_joint_goal = {3.1416 / 12.0, 0.0};  // Head joint goals
    r_arm_joint_goal = {0.0, 0.0, 0.0};  // Right arm joint goals
    l_arm_joint_goal = {0.0, 0.0, 0.0};  // Left arm joint goals
  }
  else if (goal_handle->get_goal()->task_number == 7)
  {
    head_joint_goal = {-3.1416 / 12.0, 0.0}; // Head joint goals
    r_arm_joint_goal = {0.0, 0.0, 0.0};  // Right arm joint goals
    l_arm_joint_goal = {0.0, 0.0, 0.0};  // Left arm joint goals
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Invalid Task Number");
    return;
  }

  bool head_within_bounds = head_move_group.setJointValueTarget(head_joint_goal);
  bool r_arm_within_bounds = r_arm_move_group.setJointValueTarget(r_arm_joint_goal);
  bool l_arm_within_bounds = l_arm_move_group.setJointValueTarget(l_arm_joint_goal);

  if (!head_within_bounds || !r_arm_within_bounds || !l_arm_within_bounds)
  {
    RCLCPP_WARN(get_logger(),
                "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
    return;
  }

  moveit::planning_interface::MoveGroupInterface::Plan head_plan, r_arm_plan, l_arm_plan;
  bool head_plan_success = (head_move_group.plan(head_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  bool r_arm_plan_success = (r_arm_move_group.plan(r_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  bool l_arm_plan_success = (l_arm_move_group.plan(l_arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (head_plan_success && r_arm_plan_success && l_arm_plan_success)
  {
    RCLCPP_INFO(get_logger(), "Planner SUCCEED, moving the groups");
    head_move_group.move();
    r_arm_move_group.move();
    l_arm_move_group.move();
  }
  else
  {
    RCLCPP_ERROR(get_logger(), "Planner failed!");
    return;
  }

  result->success = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(get_logger(), "Goal succeeded");
}
};
}  // namespace op2_rutines

RCLCPP_COMPONENTS_REGISTER_NODE(op2_rutines::Op2Task)

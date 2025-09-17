#include <functional>
#include <memory>
#include <thread>
#include <moveit/move_group_interface/move_group_interface.h>

#include "kuka_motion_plan_action/action/motionplan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace kuka_motion
{
class MotionPlanServer : public rclcpp::Node
{
public:
  using Motionplan = kuka_motion_plan_action::action::Motionplan;
  using GoalHandleMotionplan = rclcpp_action::ServerGoalHandle<Motionplan>;

  // MotionPlanServer Node constructor
  explicit MotionPlanServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("motionplan_action_server", options), 
    kuka_arm_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "kuka_arm"),
    omnibase_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "omnibase"),
    agv_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "agv")
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Motionplan>(
      this,
      "motionplan",
      std::bind(&MotionPlanServer::handle_goal, this, _1, _2),
      std::bind(&MotionPlanServer::handle_cancel, this, _1),
      std::bind(&MotionPlanServer::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<Motionplan>::SharedPtr action_server_;
  moveit::planning_interface::MoveGroupInterface kuka_arm_group_;
  moveit::planning_interface::MoveGroupInterface omnibase_group_;
  moveit::planning_interface::MoveGroupInterface agv_group_;
  

  // goal response: echo received goal position before execution
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Motionplan::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose: Position(%.2f, %.2f, %.2f)",
                    goal->goal_pose.pose.position.x, goal->goal_pose.pose.position.y, goal->goal_pose.pose.position.z);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  // cancle response
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMotionplan> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // execute moveit motion planning for accepted goal
  void handle_accepted(const std::shared_ptr<GoalHandleMotionplan> goal_handle)
  {
    using namespace std::placeholders;
    std::thread{std::bind(&MotionPlanServer::execute, this, _1), goal_handle}.detach();
  }

  // motion planning function
  void execute(const std::shared_ptr<GoalHandleMotionplan> goal_handle)
  {
    bool success;
    auto result = std::make_shared<Motionplan::Result>();
    //feedback unused atm
    auto feedback = std::make_shared<Motionplan::Feedback>();

    auto goal = goal_handle->get_goal();
    RCLCPP_INFO(this->get_logger(), "Executing move to pose goal for move group %d", goal->planning_group);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (goal->planning_group > 5){ // filter for attaching/detaching/pause
      RCLCPP_INFO(this->get_logger(), "Detach/Attach Complete.");
      goal_handle->succeed(result);
      return;
    }
    switch (goal->planning_group)
    {
    case 1: //kuka_arm
      // set target pose and the moveit planner id (TODO: anderer kommentar!)
      kuka_arm_group_.setPoseTarget(goal->goal_pose);
      kuka_arm_group_.setPlannerId(goal->planner);
      if(goal->planner == "LIN") {
        kuka_arm_group_.setMaxVelocityScalingFactor(0.2);
        kuka_arm_group_.setGoalPositionTolerance(0.01);
      }
      else{
        kuka_arm_group_.setMaxVelocityScalingFactor(1.0);
      }
      // plan and execute
      success = (kuka_arm_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Motion Planning Failed! Retrying once!");
          success = (kuka_arm_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
          if (!success) {
              RCLCPP_ERROR(this->get_logger(), "Motion Planning Failed!");
              goal_handle->abort(result);
              return;
          }
      }
      kuka_arm_group_.execute(plan);
      break;
    case 2: //omnibase
      // set target pose and the moveit planner id (TODO: anderer kommentar!)
      omnibase_group_.setPoseTarget(goal->goal_pose);
      omnibase_group_.setPlannerId(goal->planner);

      success = (omnibase_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Motion Planning Failed!");
          goal_handle->abort(result);
          return;
      }
      omnibase_group_.execute(plan);
      break;
    case 3: //AGV
      // set target pose and the moveit planner id (TODO: anderer kommentar!)
      agv_group_.setPoseTarget(goal->goal_pose);
      agv_group_.setPlannerId(goal->planner);

      success = (agv_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Motion Planning Failed!");
          goal_handle->abort(result);
          return;
      }
      agv_group_.execute(plan);
      break;
    case 4: //omnibase named target
      // set target pose and the moveit planner id (TODO: anderer kommentar!)
      omnibase_group_.setNamedTarget(goal->named_target);
      omnibase_group_.setPlannerId(goal->planner);

      success = (omnibase_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Motion Planning Failed!");
          goal_handle->abort(result);
          return;
      }
      omnibase_group_.execute(plan);
      break;
      case 5: //KUKA named Target
      // set target pose and the moveit planner id (TODO: anderer kommentar!)
      kuka_arm_group_.setNamedTarget(goal->named_target);
      kuka_arm_group_.setPlannerId(goal->planner);
      RCLCPP_INFO(this->get_logger(), "Start planning for named Target");
      success = (kuka_arm_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
      if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Motion Planning Failed!");
          goal_handle->abort(result);
          return;
      }
      kuka_arm_group_.execute(plan);
      break;
    
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown planning group!%d", goal->planning_group);
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Motion Execution Complete.");

    goal_handle->succeed(result);

  }
};  // class MotionPlanServer

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(kuka_motion::MotionPlanServer)
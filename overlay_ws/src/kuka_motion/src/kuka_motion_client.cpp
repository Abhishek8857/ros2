//========================================================
//OUTDATED!
//========================================================
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <geometry_msgs/msg/pose_array.hpp>

#include "kuka_motion_plan_action/action/motionplan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace kuka_motion
{
class MotionPlanClient : public rclcpp::Node
{
public:
  using Motionplan = kuka_motion_plan_action::action::Motionplan;
  using GoalHandleMotionplan = rclcpp_action::ClientGoalHandle<Motionplan>;

  // MotionPlanClient constructor
  explicit MotionPlanClient(const rclcpp::NodeOptions & options)
  : Node("motionplan_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Motionplan>(
      this,
      "motionplan");

    // subscribe to topic (pose_vector) where goal poses are published
    this->subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "pose_vector", 10,
            std::bind(&MotionPlanClient::motion_plan_callback, this, std::placeholders::_1));
  }

private:
  rclcpp_action::Client<Motionplan>::SharedPtr client_ptr_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
  std::vector<geometry_msgs::msg::Pose> poses_;
  std::vector<geometry_msgs::msg::Pose> modified_poses_;
  std::vector<float> plug_poses_;
  std::vector<int> move_group_vector_;
  std::vector<std::string> planner_;
  size_t current_pose_index_;
  bool executing_ = false;
  std::vector<rclcpp::TimerBase::SharedPtr> active_timers_;
  std::vector<std::string> namedTarget_;
  
  // builds waypoints for the overall path from battery module poses
  void motion_plan_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
      if(!executing_){ // only accept new poses when no motion plan is being executed
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->poses.size());
        poses_ = msg->poses;
        current_pose_index_ = 0;

        // add a raised pose 30cm above the battery module that shall be approached before and after the modules position
        modified_poses_.push_back(poses_[0]);
        move_group_vector_.push_back(4);
        plug_poses_.push_back(0.0f);
        planner_.push_back("PTP");
        namedTarget_.push_back("battery_pack");

        RCLCPP_INFO(this->get_logger(), "Building Waypoint array.");
        for (const auto &pose : poses_) {
          geometry_msgs::msg::Pose raised_pose = pose;
          raised_pose.position.z += 0.15f;

          // move over battery
          modified_poses_.push_back(raised_pose);
          move_group_vector_.push_back(1);
          plug_poses_.push_back(0.0f);
          planner_.push_back("PTP");
          namedTarget_.push_back("");
          RCLCPP_INFO(this->get_logger(),
                    "Added Pose: Position(%.2f, %.2f, %.2f)",
                    raised_pose.position.x, raised_pose.position.y, raised_pose.position.z);

          // move onto battery
          modified_poses_.push_back(pose);
          move_group_vector_.push_back(1);
          plug_poses_.push_back(0.0f);
          planner_.push_back("LIN");
          namedTarget_.push_back("");
          RCLCPP_INFO(this->get_logger(),
                    "Added Pose: Position(%.2f, %.2f, %.2f)",
                    pose.position.x, pose.position.y, pose.position.z);
          // close plug 1
          modified_poses_.push_back(pose);
          move_group_vector_.push_back(2);
          plug_poses_.push_back(0.03f);
          planner_.push_back("PTP");
          namedTarget_.push_back("");
          RCLCPP_INFO(this->get_logger(),
                    "Added Pose: Close Plug 1");
          // close plug 2
          modified_poses_.push_back(pose);
          move_group_vector_.push_back(3);
          plug_poses_.push_back(0.03f);
          planner_.push_back("PTP");
          namedTarget_.push_back("");
          RCLCPP_INFO(this->get_logger(),
                    "Added Pose: Close Plug 2");
          // open plug 1
          modified_poses_.push_back(pose);
          move_group_vector_.push_back(2);
          plug_poses_.push_back(0.0f);
          planner_.push_back("PTP");
          namedTarget_.push_back("");
          RCLCPP_INFO(this->get_logger(),
                    "Added Pose: Open Plug 1");
          // open plug 2
          modified_poses_.push_back(pose);
          move_group_vector_.push_back(3);
          plug_poses_.push_back(0.0f);
          planner_.push_back("PTP");
          namedTarget_.push_back("");
          RCLCPP_INFO(this->get_logger(),
                    "Added Pose: Open Plug 2");
          // move over battery
          modified_poses_.push_back(raised_pose);
          move_group_vector_.push_back(1);
          plug_poses_.push_back(0.0f);
          planner_.push_back("LIN");
          namedTarget_.push_back("");
          RCLCPP_INFO(this->get_logger(),
                    "Added Pose: Position(%.2f, %.2f, %.2f)",
                    raised_pose.position.x, raised_pose.position.y, raised_pose.position.z);
        }
        modified_poses_.push_back(poses_[0]);
        move_group_vector_.push_back(5);
        plug_poses_.push_back(0.0f);
        planner_.push_back("PTP");
        namedTarget_.push_back("idle");

        modified_poses_.push_back(poses_[0]);
        move_group_vector_.push_back(4);
        plug_poses_.push_back(0.0f);
        planner_.push_back("PTP");
        namedTarget_.push_back("home");

        executing_ = true;
        send_goal();
      }
      else{
        RCLCPP_WARN(this->get_logger(), "Received PoseArray with %zu poses. Ignoring new poses until execution has finished.", msg->poses.size());
      }
      
  }

  // Send pose goal to action server
  void send_goal()
  {
    using namespace std::placeholders;

    // check if there is a new goal, else wait for new poses to be published
    if (current_pose_index_ >= modified_poses_.size()) {
            RCLCPP_INFO(this->get_logger(), "All poses have been processed");
            executing_ = false;
            return;
        }
    auto goal_msg = Motionplan::Goal();
    goal_msg.goal_pose = modified_poses_[current_pose_index_];
    goal_msg.planning_group = move_group_vector_[current_pose_index_];
    goal_msg.plug_pose = plug_poses_[current_pose_index_];
    goal_msg.planner = planner_[current_pose_index_];
    goal_msg.named_target = namedTarget_[current_pose_index_];
    switch (goal_msg.planning_group)
    {
    case 1: //kuka_arm
      // set target pose and the moveit planner id
      RCLCPP_INFO(this->get_logger(),
                "Sending Pose %zu: Position(%.2f, %.2f, %.2f)",
                current_pose_index_,
                goal_msg.goal_pose.position.x, goal_msg.goal_pose.position.y, goal_msg.goal_pose.position.z);
      break;
    case 2: //plug_1
      // set target pose and the moveit planner id
      RCLCPP_INFO(this->get_logger(),
                "Sending Pose %zu: Plug_1 s(%.2f)",
                current_pose_index_,
                goal_msg.plug_pose);
      break;
    case 3: //plug_2
          RCLCPP_INFO(this->get_logger(),
                "Sending Pose %zu: Plug_2 (%.2f)",
                current_pose_index_,
                goal_msg.plug_pose);
      break;
    case 4: //omnibase
            RCLCPP_INFO(this->get_logger(),
            "Sending Pose %zu: Omnibase (%s)",
            current_pose_index_,
            goal_msg.named_target);
      break;
    case 5: //idle
            RCLCPP_INFO(this->get_logger(),
            "Sending Pose %zu: Kuka Arm (%s)",
            current_pose_index_,
            goal_msg.named_target);
      break;
    
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown planning group!");
      return;
    }

    auto send_goal_options = rclcpp_action::Client<Motionplan>::SendGoalOptions();
    send_goal_options.result_callback = std::bind(&MotionPlanClient::result_callback, this, std::placeholders::_1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  // goal response callback
  void goal_response_callback(const GoalHandleMotionplan::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  // feedback callback, currently feedback is unused
  void feedback_callback(
    GoalHandleMotionplan::SharedPtr,
    const std::shared_ptr<const Motionplan::Feedback> feedback)
  {
    // Add feedback functionality if needed
  }

  // result callback
  void result_callback(const GoalHandleMotionplan::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Pose %zu completed successfully", current_pose_index_);
        if(move_group_vector_[current_pose_index_] == 3 && plug_poses_[current_pose_index_] > 0.01f){
          RCLCPP_INFO(this->get_logger(), "Waiting for EIS Measurement...");
          rclcpp::sleep_for(std::chrono::seconds(8));
        }
        else{
          RCLCPP_INFO(this->get_logger(), "Waiting for 3 seconds before sending position...");
          rclcpp::sleep_for(std::chrono::seconds(1));
        }
        current_pose_index_++;
        send_goal(); 
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }
};  // class MotionPlanClient

}  // namespace kuka_motion

RCLCPP_COMPONENTS_REGISTER_NODE(kuka_motion::MotionPlanClient)

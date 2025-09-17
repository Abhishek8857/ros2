#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/empty.hpp>

#include "kuka_motion_plan_action/action/motionplan.hpp"
#include "kuka_motion_plan_action/msg/motionstate.hpp"
#include "kuka_motion_plan_action/msg/motionstatearray.hpp"
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

    // subscribe to motion state topic where goal poses are published
    this->subscription_ = this->create_subscription<kuka_motion_plan_action::msg::Motionstatearray>(
            "state_vector", 10,
            std::bind(&MotionPlanClient::motion_plan_callback, this, std::placeholders::_1));
    
    // initialize publishers for handling of detachable joints. Could be saved (as pointers) in a map/vector to make this (a lot) nicer...
    this->eisKukaDetachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/eisKUKA/detach", 10);
    this->eisKukaAttachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/eisKUKA/attach", 10);
    this->eisTCSDetachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/eisTCS/detach", 10);
    this->eisTCSAttachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/eisTCS/attach", 10);

    this->gripperKukaDetachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/gripperKUKA/detach", 10);
    this->gripperKukaAttachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/gripperKUKA/attach", 10);
    this->gripperTCSDetachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/gripperTCS/detach", 10);
    this->gripperTCSAttachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/gripperTCS/attach", 10);

    this->vacuumKukaDetachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/vacuumKUKA/detach", 10);
    this->vacuumKukaAttachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/vacuumKUKA/attach", 10);
    this->vacuumTCSDetachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/vacuumTCS/detach", 10);
    this->vacuumTCSAttachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/vacuumTCS/attach", 10);
    this->vacuumDeckelDetachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/deckelVakuum/detach", 10);
    this->vacuumDeckelAttachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/deckelVakuum/attach", 10);

    this->deckelPackDetachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/deckel/detach", 10);
    this->deckelPackAttachPublisher_ = this->create_publisher<std_msgs::msg::Empty>("/model/deckel/attach", 10);

    for (int i = 1; i <= 15; ++i) {
            std::string topic_name_attach = "/model/module_" + std::to_string(i) + "Gripper/attach";
            std::string topic_name_detach = "/model/module_" + std::to_string(i) + "Gripper/detach";
            RCLCPP_INFO(this->get_logger(), std::to_string(i).c_str());
            // Attach publisher
            ModuleGripperAttachPublisher_[i - 1] = this->create_publisher<std_msgs::msg::Empty>("/model/module_" + std::to_string(i-1) + "Gripper/attach", 10);
            
            // Detach publisher
            ModuleGripperDetachPublisher_[i - 1] = this->create_publisher<std_msgs::msg::Empty>("/model/module_" + std::to_string(i-1) + "Gripper/detach", 10);

            // Attach publisher
            ModulePackAttachPublisher_[i - 1] = this->create_publisher<std_msgs::msg::Empty>("/model/module_" + std::to_string(i-1) + "Pack/attach", 10);
            
            // Detach publisher
            ModulePackDetachPublisher_[i - 1] = this->create_publisher<std_msgs::msg::Empty>("/model/module_" + std::to_string(i-1) + "Pack/detach", 10);
        }
  }

private:
  rclcpp_action::Client<Motionplan>::SharedPtr client_ptr_;
  rclcpp::Subscription<kuka_motion_plan_action::msg::Motionstatearray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr eisKukaDetachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr eisKukaAttachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr eisTCSDetachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr eisTCSAttachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripperKukaDetachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripperKukaAttachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripperTCSDetachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr gripperTCSAttachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuumKukaDetachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuumKukaAttachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuumTCSDetachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuumTCSAttachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuumDeckelDetachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vacuumDeckelAttachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr deckelPackDetachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr deckelPackAttachPublisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ModuleGripperDetachPublisher_[15];
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ModuleGripperAttachPublisher_[15];
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ModulePackDetachPublisher_[15];
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr ModulePackAttachPublisher_[15];
  std::vector<kuka_motion_plan_action::msg::Motionstate> states_;
  std::vector<geometry_msgs::msg::PoseStamped> modified_poses_;
  std::vector<float> plug_poses_;
  std::vector<int> move_group_vector_;
  std::vector<std::string> planner_;
  size_t current_pose_index_;
  bool executing_ = false;
  std::vector<rclcpp::TimerBase::SharedPtr> active_timers_;
  std::vector<std::string> namedTarget_;
  std_msgs::msg::Empty emptyMsg;
  
  // builds waypoints for the overall path from battery module poses
  void motion_plan_callback(const kuka_motion_plan_action::msg::Motionstatearray::SharedPtr msg) {
      if(!executing_){ // only accept new poses when no motion plan is being executed
        RCLCPP_INFO(this->get_logger(), "Received PoseArray with %zu poses", msg->statemachinearray.size());
        states_ = msg->statemachinearray;
        current_pose_index_ = 0;

        RCLCPP_INFO(this->get_logger(), "Building Waypoint array.");
        for (const auto &state : states_) {
          if (state.operation == "kuka_arm") { // handles PTP motion for KUKA arm
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(1);
            plug_poses_.push_back(0.0f);
            planner_.push_back("PTP");
            namedTarget_.push_back("");
            RCLCPP_INFO(this->get_logger(),
                      "Added Pose: Position(%.2f, %.2f, %.2f)",
                      state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z);
          }
          else if (state.operation == "kuka_arm_lin") { // handles linear motion for KUKA arm
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(1);
            plug_poses_.push_back(0.0f);
            planner_.push_back("LIN");
            namedTarget_.push_back("");
            RCLCPP_INFO(this->get_logger(),
                      "Added Pose: Position(%.2f, %.2f, %.2f)",
                      state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z);
          }
          else if (state.operation == "omnibase") { // handles omnibase motion
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(2);
            plug_poses_.push_back(0.0f);
            planner_.push_back("PTP");
            namedTarget_.push_back("");
            RCLCPP_INFO(this->get_logger(),
                      "Added Pose: Position(%.2f, %.2f, %.2f)",
                      state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z);
          }
          else if (state.operation == "agv") { // handles donkey motion
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(3);
            plug_poses_.push_back(0.0f);
            planner_.push_back("PTP");
            namedTarget_.push_back("");
            RCLCPP_INFO(this->get_logger(),
                      "Added Pose: Position(%.2f, %.2f, %.2f)",
                      state.pose.pose.position.x, state.pose.pose.position.y, state.pose.pose.position.z);
          }
          else if (state.operation == "KUKAidle") // moves kuka arm into idle position
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(5);
            plug_poses_.push_back(0.0f);
            planner_.push_back("PTP");
            namedTarget_.push_back("idle");
          }
          else if (state.operation == "KUKATCSgripper") // not used. Moves KUKA arm into neutral position at the right side of the base for tool changing
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(5);
            plug_poses_.push_back(0.0f);
            planner_.push_back("PTP");
            namedTarget_.push_back("TCSgripper");
          }
          else if (state.operation == "KUKATCSeis") // Moves KUKA arm into neutral position at the left side of the base for tool changing
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(5);
            plug_poses_.push_back(0.0f);
            planner_.push_back("PTP");
            namedTarget_.push_back("TCSeis");
          }
          else if (state.operation == "eisKUKAdetach") // following else if statements are used for attaching/detaching various detachable joints
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(6);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "eisKUKAattach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(7);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "eisTCSdetach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(8);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "eisTCSattach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(9);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "gripperKUKAdetach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(10);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "gripperKUKAattach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(11);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "gripperTCSdetach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(12);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "gripperTCSattach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(13);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "vacuumKUKAdetach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(14);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "vacuumKUKAattach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(15);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "vacuumTCSdetach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(16);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "vacuumTCSattach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(17);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "deckelVakuumdetach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(18);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "deckelVakuumattach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(19);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "deckeldetach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(20);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation == "deckelattach")
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(21);
            plug_poses_.push_back(0.0f);
            planner_.push_back("");
            namedTarget_.push_back("");
          }
          else if (state.operation.rfind("detachModule_", 0) == 0)
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(22);
            plug_poses_.push_back(0.0f);
            planner_.push_back(state.operation.substr(13)); // takes substring after _ to extract number of module
            namedTarget_.push_back("");
          }
          else if (state.operation.rfind("attachModule_", 0) == 0)
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(23);
            plug_poses_.push_back(0.0f);
            planner_.push_back(state.operation.substr(13));
            namedTarget_.push_back("");
          }
          else if (state.operation.rfind("detachPackModule_", 0) == 0)
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(24);
            plug_poses_.push_back(0.0f);
            planner_.push_back(state.operation.substr(17));
            namedTarget_.push_back("");
          }
          else if (state.operation.rfind("attachPackModule_", 0) == 0)
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(25);
            plug_poses_.push_back(0.0f);
            planner_.push_back(state.operation.substr(17));
            namedTarget_.push_back("");
          }
          else if (state.operation.rfind("pause_", 0) == 0) // pauses for the duration given after _
          {
            modified_poses_.push_back(state.pose);
            move_group_vector_.push_back(99);
            plug_poses_.push_back(0.0f);
            planner_.push_back(state.operation.substr(6));
            namedTarget_.push_back("");
          }
        }

        executing_ = true;
        send_goal();
      }
      else{
        RCLCPP_WARN(this->get_logger(), "Received PoseArray with %zu poses. Ignoring new poses until execution has finished.", msg->statemachinearray.size());
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
      RCLCPP_INFO(this->get_logger(),
                "Sending Pose %zu: Position(%.2f, %.2f, %.2f)",
                current_pose_index_,
                goal_msg.goal_pose.pose.position.x, goal_msg.goal_pose.pose.position.y, goal_msg.goal_pose.pose.position.z);
      break;
    case 2: //omnibase
          RCLCPP_INFO(this->get_logger(),
                "Sending Pose %zu: Omnibase Position(%.2f, %.2f, %.2f",
                current_pose_index_,
                goal_msg.goal_pose.pose.position.x, goal_msg.goal_pose.pose.position.y, goal_msg.goal_pose.pose.position.z);
      break;
    case 3: // donkey
          RCLCPP_INFO(this->get_logger(),
                "Sending Pose %zu: AGV Position(%.2f, %.2f, %.2f",
                current_pose_index_,
                goal_msg.goal_pose.pose.position.x, goal_msg.goal_pose.pose.position.y, goal_msg.goal_pose.pose.position.z);
      break;
    case 4: //omnibase named Target
            RCLCPP_INFO(this->get_logger(),
            "Sending Pose %zu: Omnibase ()",
            current_pose_index_/*,
            goal_msg.named_target*/);
      break;
    case 5: //kuka named Target (like idle or eisTCS)
            RCLCPP_INFO(this->get_logger(),
            "Sending Pose %zu: Kuka Arm ()",
            current_pose_index_/*,
            goal_msg.named_target*/);
      break;
    case 6: //detach EIS KUKA
      this->eisKukaDetachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Detaching EIS Endeffector from KUKA");
      break;
    case 7: //attach EIS KUKA
      this->eisKukaAttachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching EIS Endeffector to KUKA");
      break;
    case 8: //detach EIS TCS
      this->eisTCSDetachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Detaching EIS Endeffector from TCS");
      break;
    case 9: //attach EIS TCS
      this->eisTCSAttachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching EIS Endeffector to TCS");
      break;
    case 10: //detach gripper KUKA
      this->gripperKukaDetachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Detaching gripper Endeffector from KUKA");
      break;
    case 11: //attach gripper KUKA
      this->gripperKukaAttachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching gripper Endeffector to KUKA");
      break;
    case 12: //detach gripper TCS
      this->gripperTCSDetachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Detaching gripper Endeffector from TCS");
      break;
    case 13: //attach gripper TCS
      this->gripperTCSAttachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching gripper Endeffector to TCS");
      break;
        case 14: //detach vacuum KUKA
      this->vacuumKukaDetachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Detaching vacuum Endeffector from KUKA");
      break;
    case 15: //attach vacuum KUKA
      this->vacuumKukaAttachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching vacuum Endeffector to KUKA");
      break;
    case 16: //detach vacuum TCS
      this->vacuumTCSDetachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Detaching vacuum Endeffector from TCS");
      break;
    case 17: //attach vacuum TCS
      this->vacuumTCSAttachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching vacuum Endeffector to TCS");
      break;
    case 18: //detach vacuum Deckel
      this->vacuumDeckelDetachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Detaching vacuum Endeffector from Deckel");
      break;
    case 19: //attach vacuum Deckel
      this->vacuumDeckelAttachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching vacuum Endeffector to Deckel");
      break;
    case 20: //detach vacuum Deckel
      this->deckelPackDetachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Detaching Deckel from Pack");
      break;
    case 21: //attach vacuum Deckel
      this->deckelPackAttachPublisher_->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching Deckel from Pack");
      break;
    case 22: //detach module from gripper
      this->ModuleGripperDetachPublisher_[std::stoi(goal_msg.planner)]->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), ("Detaching Module " + goal_msg.planner + " from Gripper").c_str());
      break;
    case 23: //attach module to gripper
      this->ModuleGripperAttachPublisher_[std::stoi(goal_msg.planner)]->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching Module to Gripper");
      break;
    case 24: //detach module from pack
      this->ModulePackDetachPublisher_[std::stoi(goal_msg.planner)]->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), ("Detaching Module " + goal_msg.planner + " from Pack").c_str());
      break;
    case 25: //attach module to pack
      this->ModulePackAttachPublisher_[std::stoi(goal_msg.planner)]->publish(emptyMsg);
      RCLCPP_INFO(this->get_logger(), "Attaching Module to Pack");
      break;


    case 99: //pause
      RCLCPP_INFO(this->get_logger(), "Pausing for %s seconds", goal_msg.planner.c_str());
      rclcpp::sleep_for(std::chrono::seconds(std::stoi(goal_msg.planner)));
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown planning group!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Debug 1");
    auto send_goal_options = rclcpp_action::Client<Motionplan>::SendGoalOptions();
    RCLCPP_INFO(this->get_logger(), "Debug 2");
    send_goal_options.result_callback = std::bind(&MotionPlanClient::result_callback, this, std::placeholders::_1);
    RCLCPP_INFO(this->get_logger(), "Sending Goal");
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
        // the following can be used to include pases after the movement of certain movegroups
        // if(move_group_vector_[current_pose_index_] >= 6){
        //   // RCLCPP_INFO(this->get_logger(), "Detaching/Attaching...");
        //   // rclcpp::sleep_for(std::chrono::seconds(1));
        // }
        // else{
        //   // RCLCPP_INFO(this->get_logger(), "Waiting for 3 seconds before sending position...");
        //   // rclcpp::sleep_for(std::chrono::seconds(1));
        // }
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

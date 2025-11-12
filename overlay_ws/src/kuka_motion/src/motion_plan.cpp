#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <map>
#include <string>
#include <vector>
#include <chrono>

int main(int argc, char** argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("motion_plan_joint_sequence");

    // Give MoveIt and RViz time to start
    rclcpp::sleep_for(std::chrono::seconds(2));

    // Create MoveGroupInterface for manipulator
    static const std::string PLANNING_GROUP = "manipulator";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

    // Optional: choose a planner
    move_group.setPlannerId("RRTConnectkConfigDefault");

    // Define multiple joint positions (sequence)
    std::vector<std::map<std::string, double>> joint_sequence = {
        {{"joint_1", 0.0}, {"joint_2", -2.0}, {"joint_3", 2.0}, {"joint_4", 0.0}, {"joint_5", 1.5}, {"joint_6", 0.0}},
        {{"joint_1", 0.5}, {"joint_2", -1.5}, {"joint_3", 1.5}, {"joint_4", 0.0}, {"joint_5", 1.0}, {"joint_6", 0.0}},
        {{"joint_1", -0.5}, {"joint_2", -2.5}, {"joint_3", 2.0}, {"joint_4", 0.0}, {"joint_5", 1.5}, {"joint_6", 0.0}}
        // Add more positions as needed
    };

    // Iterate through the sequence
    for (size_t i = 0; i < joint_sequence.size(); ++i)
    {
        RCLCPP_INFO(node->get_logger(), "Moving to position %zu...", i+1);
        move_group.setJointValueTarget(joint_sequence[i]);

        bool success = (move_group.move() == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(node->get_logger(), "Finished position %zu!", i+1);
            rclcpp::sleep_for(std::chrono::seconds(2));  // short rest between motions
        }
        else
        {
            RCLCPP_ERROR(node->get_logger(), "Move failed for position %zu!", i+1);
        }
    }


    RCLCPP_INFO(node->get_logger(), "Finished all positions.");

    rclcpp::shutdown();
    return 0;
}

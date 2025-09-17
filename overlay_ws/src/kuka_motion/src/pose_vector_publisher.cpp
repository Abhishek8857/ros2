//========================================================
//OUTDATED!
//========================================================
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

class PoseVectorPublisherNode : public rclcpp::Node {
public:
    PoseVectorPublisherNode() : Node("pose_vector_publisher") {
        // Create a publisher for the PoseArray poseArray
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("pose_vector", 10);

        // Publish data at 0.05 Hz
        timer_ = this->create_wall_timer(
            std::chrono::seconds(20),
            std::bind(&PoseVectorPublisherNode::publishPoseVector, this)
        );

        RCLCPP_INFO(this->get_logger(), "Pose vector publisher node has started.");
    }

private:
    void publishPoseVector() {
        // Create the PoseArray poseArray
        geometry_msgs::msg::PoseArray poseArray;
        poseArray.header.stamp = this->now();
        poseArray.header.frame_id = "world";  // Set the reference frame

        geometry_msgs::msg::Pose pose1;
        pose1.position.x = 2.2657;
        pose1.position.y = -0.8485;
        pose1.position.z = 1.101;
        pose1.orientation.x = 0.0;
        pose1.orientation.y = 0.0;
        pose1.orientation.z = 0.0;
        pose1.orientation.w = 1.0;
        poseArray.poses.push_back(pose1);

        geometry_msgs::msg::Pose pose2;
        pose2.position.x = 2.2657;
        pose2.position.y = -0.6845;
        pose2.position.z = 1.1;
        pose2.orientation.x = 0.0;
        pose2.orientation.y = 0.0;
        pose2.orientation.z = 0.0;
        pose2.orientation.w = 1.0;
        poseArray.poses.push_back(pose2);

        geometry_msgs::msg::Pose pose3;
        pose3.position.x = 2.2657;
        pose3.position.y = -0.5205;
        pose3.position.z = 1.099;
        pose3.orientation.x = 0.0;
        pose3.orientation.y = 0.0;
        pose3.orientation.z = 0.0;
        pose3.orientation.w = 1.0;
        poseArray.poses.push_back(pose3);

        geometry_msgs::msg::Pose pose4;
        pose4.position.x = 2.2657;
        pose4.position.y = -0.3345;
        pose4.position.z = 1.099;
        pose4.orientation.x = 0.0;
        pose4.orientation.y = 0.0;
        pose4.orientation.z = 0.0;
        pose4.orientation.w = 1.0;
        poseArray.poses.push_back(pose4);

        geometry_msgs::msg::Pose pose5;
        pose5.position.x = 2.2657;
        pose5.position.y = -0.1705;
        pose5.position.z = 1.10;
        pose5.orientation.x = 0.0;
        pose5.orientation.y = 0.0;
        pose5.orientation.z = 0.0;
        pose5.orientation.w = 1.0;
        poseArray.poses.push_back(pose5);

        geometry_msgs::msg::Pose pose6;
        pose6.position.x = 2.2657;
        pose6.position.y = -0.0065;
        pose6.position.z = 1.10;
        pose6.orientation.x = 0.0;
        pose6.orientation.y = 0.0;
        pose6.orientation.z = 0.0;
        pose6.orientation.w = 1.0;
        poseArray.poses.push_back(pose6);

        geometry_msgs::msg::Pose pose7;
        pose7.position.x = 2.2657;
        pose7.position.y = 0.1575;
        pose7.position.z = 1.10;
        pose7.orientation.x = 0.0;
        pose7.orientation.y = 0.0;
        pose7.orientation.z = 0.0;
        pose7.orientation.w = 1.0;
        poseArray.poses.push_back(pose7);

        geometry_msgs::msg::Pose pose8;
        pose8.position.x = 2.2657;
        pose8.position.y = 0.3215;
        pose8.position.z = 1.099;
        pose8.orientation.x = 0.0;
        pose8.orientation.y = 0.0;
        pose8.orientation.z = 0.0;
        pose8.orientation.w = 1.0;
        poseArray.poses.push_back(pose8);

        geometry_msgs::msg::Pose pose9;
        pose9.position.x = 2.2657;
        pose9.position.y = 0.5075;
        pose9.position.z = 1.10;
        pose9.orientation.x = 0.0;
        pose9.orientation.y = 0.0;
        pose9.orientation.z = 0.0;
        pose9.orientation.w = 1.0;
        poseArray.poses.push_back(pose9);

        geometry_msgs::msg::Pose pose10;
        pose10.position.x = 2.2657;
        pose10.position.y = 0.6715;
        pose10.position.z = 1.10;
        pose10.orientation.x = 0.0;
        pose10.orientation.y = 0.0;
        pose10.orientation.z = 0.0;
        pose10.orientation.w = 1.0;
        poseArray.poses.push_back(pose10);

        geometry_msgs::msg::Pose pose11;
        pose11.position.x = 2.2657;
        pose11.position.y = 0.8355;
        pose11.position.z = 1.10;
        pose11.orientation.x = 0.0;
        pose11.orientation.y = 0.0;
        pose11.orientation.z = 0.0;
        pose11.orientation.w = 1.0;
        poseArray.poses.push_back(pose11);

        geometry_msgs::msg::Pose pose12;
        pose12.position.x = 1.9851;
        pose12.position.y = 0.6265;
        pose12.position.z = 1.10;
        pose12.orientation.x = 0.0;
        pose12.orientation.y = 0.0;
        pose12.orientation.z = 0.7071;
        pose12.orientation.w = 0.7071;
        poseArray.poses.push_back(pose12);

        geometry_msgs::msg::Pose pose13;
        pose13.position.x = 1.9851;
        pose13.position.y = 0.2025;
        pose13.position.z = 1.099;
        pose13.orientation.x = 0.0;
        pose13.orientation.y = 0.0;
        pose13.orientation.z = 0.7071;
        pose13.orientation.w = 0.7071;
        poseArray.poses.push_back(pose13);

        geometry_msgs::msg::Pose pose14;
        pose14.position.x = 1.9851;
        pose14.position.y = -0.2155;
        pose14.position.z = 1.10;
        pose14.orientation.x = 0.0;
        pose14.orientation.y = 0.0;
        pose14.orientation.z = 0.7071;
        pose14.orientation.w = 0.7071;
        poseArray.poses.push_back(pose14);

        geometry_msgs::msg::Pose pose15;
        pose15.position.x = 1.9851;
        pose15.position.y = -0.6395;
        pose15.position.z = 1.10;
        pose15.orientation.x = 0.0;
        pose15.orientation.y = 0.0;
        pose15.orientation.z = 0.7071;
        pose15.orientation.w = 0.7071;
        poseArray.poses.push_back(pose15);

        // Publish the poseArray
        publisher_->publish(poseArray);
        RCLCPP_INFO(this->get_logger(), "Published a PoseArray poseArray with %zu poses.", poseArray.poses.size());
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseVectorPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

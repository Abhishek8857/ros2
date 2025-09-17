#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "kuka_motion_plan_action/msg/motionstate.hpp"
#include "kuka_motion_plan_action/msg/motionstatearray.hpp"

class MotionStateGeneratorNode : public rclcpp::Node {
public:
    MotionStateGeneratorNode() : Node("pose_vector_publisher") {
        // Create a publisher for the motion state array
        publisher_ = this->create_publisher<kuka_motion_plan_action::msg::Motionstatearray>("state_vector", 10);

        // Publish data at 0.05 Hz
        timer_ = this->create_wall_timer(
            std::chrono::seconds(20),
            std::bind(&MotionStateGeneratorNode::publishStateVector, this)
        );

        RCLCPP_INFO(this->get_logger(), "Pose vector publisher node has started.");
    }

private:
    void publishStateVector() {
        // Create the Motion State Array
        kuka_motion_plan_action::msg::Motionstatearray MotionStateArray;

        //=====================================================================
        // Initialize Detachable Joints
        //=====================================================================

        // Detach EIS from KUKA
        kuka_motion_plan_action::msg::Motionstate EISKUKAdetach;
        EISKUKAdetach.operation = "eisKUKAdetach";
        EISKUKAdetach.pose.pose.position.x = 0.0;
        EISKUKAdetach.pose.pose.position.y = 0.0;
        EISKUKAdetach.pose.pose.position.z = 0.0;
        EISKUKAdetach.pose.pose.orientation.x = 0.0;
        EISKUKAdetach.pose.pose.orientation.y = 0.0;
        EISKUKAdetach.pose.pose.orientation.z = 0.0;
        EISKUKAdetach.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(EISKUKAdetach);

        // Detach gripper from KUKA
        kuka_motion_plan_action::msg::Motionstate gripperKUKAdetach;
        gripperKUKAdetach.operation = "gripperKUKAdetach";
        gripperKUKAdetach.pose.pose.position.x = 0.0;
        gripperKUKAdetach.pose.pose.position.y = 0.0;
        gripperKUKAdetach.pose.pose.position.z = 0.0;
        gripperKUKAdetach.pose.pose.orientation.x = 0.0;
        gripperKUKAdetach.pose.pose.orientation.y = 0.0;
        gripperKUKAdetach.pose.pose.orientation.z = 0.0;
        gripperKUKAdetach.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(gripperKUKAdetach);

        // Detach vacuumgripper from KUKA
        kuka_motion_plan_action::msg::Motionstate vacuumKUKAdetach;
        vacuumKUKAdetach.operation = "vacuumKUKAdetach";
        vacuumKUKAdetach.pose.pose.position.x = 0.0;
        vacuumKUKAdetach.pose.pose.position.y = 0.0;
        vacuumKUKAdetach.pose.pose.position.z = 0.0;
        vacuumKUKAdetach.pose.pose.orientation.x = 0.0;
        vacuumKUKAdetach.pose.pose.orientation.y = 0.0;
        vacuumKUKAdetach.pose.pose.orientation.z = 0.0;
        vacuumKUKAdetach.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(vacuumKUKAdetach);

        // Detach lid from vacuumgripper
        kuka_motion_plan_action::msg::Motionstate deckelVakuumDetach;
        deckelVakuumDetach.operation = "deckelVakuumdetach";
        deckelVakuumDetach.pose.header.frame_id = "base_link_footprint";
        deckelVakuumDetach.pose.pose.position.x = -1.885;
        deckelVakuumDetach.pose.pose.position.y = 0.02;
        deckelVakuumDetach.pose.pose.position.z = 1.75;
        deckelVakuumDetach.pose.pose.orientation.x = 0.0;
        deckelVakuumDetach.pose.pose.orientation.y = 0.7071;
        deckelVakuumDetach.pose.pose.orientation.z = 0.0;
        deckelVakuumDetach.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(deckelVakuumDetach);

        // Detach battery modules from gripper
        kuka_motion_plan_action::msg::Motionstate detachModule;
        detachModule.operation = "detachModule_0";
        detachModule.pose.header.frame_id = "base_link_footprint";
        detachModule.pose.pose.position.x = -1.885;
        detachModule.pose.pose.position.y = 0.02;
        detachModule.pose.pose.position.z = 1.75;
        detachModule.pose.pose.orientation.x = 0.0;
        detachModule.pose.pose.orientation.y = 0.7071;
        detachModule.pose.pose.orientation.z = 0.0;
        detachModule.pose.pose.orientation.w = 0.7071;

        // Detach battery modules from gripper
        for (int i = 0; i < 15; i++) {
            detachModule.operation = "detachModule_" + std::to_string(i);
            MotionStateArray.statemachinearray.push_back(detachModule);
        }

        // pause for 2 seconds
        kuka_motion_plan_action::msg::Motionstate pause;
        pause.operation = "pause_2";
        pause.pose.pose.position.x = 0.0;
        pause.pose.pose.position.y = 0.0;
        pause.pose.pose.position.z = 0.0;
        pause.pose.pose.orientation.x = 0.0;
        pause.pose.pose.orientation.y = 0.0;
        pause.pose.pose.orientation.z = 0.0;
        pause.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(pause);

        //=====================================================================
        // Step 1: Remove battery pack cover
        //=====================================================================

        // move KUKA to WS1
        kuka_motion_plan_action::msg::Motionstate omnibaseWS1;
        omnibaseWS1.operation = "omnibase";
        omnibaseWS1.pose.header.frame_id = "world";
        omnibaseWS1.pose.pose.position.x = 2.90433;
        omnibaseWS1.pose.pose.position.y = 4.99753;
        omnibaseWS1.pose.pose.position.z = 0.0;
        omnibaseWS1.pose.pose.orientation.x = 0.0;
        omnibaseWS1.pose.pose.orientation.y = 0.0;
        omnibaseWS1.pose.pose.orientation.z = 0.0;
        omnibaseWS1.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(omnibaseWS1);

        // Idle position for tool changing (needed for PIMP motion planner, else motion planning can fail or choose interesting paths)
        kuka_motion_plan_action::msg::Motionstate TCSeis;
        TCSeis.operation = "KUKATCSeis";
        TCSeis.pose.pose.position.x = 0.0;
        TCSeis.pose.pose.position.y = 0.0;
        TCSeis.pose.pose.position.z = 0.0;
        TCSeis.pose.pose.orientation.x = 0.0;
        TCSeis.pose.pose.orientation.y = 0.0;
        TCSeis.pose.pose.orientation.z = 0.0;
        TCSeis.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(TCSeis);

        // in front of vacuum TCS position
        kuka_motion_plan_action::msg::Motionstate frontVacuumTCS;
        frontVacuumTCS.operation = "kuka_arm";
        frontVacuumTCS.pose.header.frame_id = "base_link_footprint";
        frontVacuumTCS.pose.pose.position.x = -1.8;
        frontVacuumTCS.pose.pose.position.y = 0.02;
        frontVacuumTCS.pose.pose.position.z = 1.526;
        frontVacuumTCS.pose.pose.orientation.x = 0.0;
        frontVacuumTCS.pose.pose.orientation.y = 0.7071;
        frontVacuumTCS.pose.pose.orientation.z = 0.0;
        frontVacuumTCS.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(frontVacuumTCS);

        // vacuum TCS position
        kuka_motion_plan_action::msg::Motionstate VacuumTCS;
        VacuumTCS.operation = "kuka_arm_lin";
        VacuumTCS.pose.header.frame_id = "base_link_footprint";
        VacuumTCS.pose.pose.position.x = -1.885;
        VacuumTCS.pose.pose.position.y = 0.0198;
        VacuumTCS.pose.pose.position.z = 1.526;
        VacuumTCS.pose.pose.orientation.x = 0.0;
        VacuumTCS.pose.pose.orientation.y = 0.7071;
        VacuumTCS.pose.pose.orientation.z = 0.0;
        VacuumTCS.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(VacuumTCS);

        // Attach vacuumgripper to KUKA
        kuka_motion_plan_action::msg::Motionstate vacuumKUKAattach;
        vacuumKUKAattach.operation = "vacuumKUKAattach";
        vacuumKUKAattach.pose.header.frame_id = "base_link_footprint";
        vacuumKUKAattach.pose.pose.position.x = 0.0;
        vacuumKUKAattach.pose.pose.position.y = 0.0;
        vacuumKUKAattach.pose.pose.position.z = 0.0;
        vacuumKUKAattach.pose.pose.orientation.x = 0.0;
        vacuumKUKAattach.pose.pose.orientation.y = 0.0;
        vacuumKUKAattach.pose.pose.orientation.z = 0.0;
        vacuumKUKAattach.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(vacuumKUKAattach);

        // Detach vacuumgripper from TCS
        kuka_motion_plan_action::msg::Motionstate vacuumTCSdetach;
        vacuumTCSdetach.operation = "vacuumTCSdetach";
        vacuumTCSdetach.pose.header.frame_id = "base_link_footprint";
        vacuumTCSdetach.pose.pose.position.x = 0.0;
        vacuumTCSdetach.pose.pose.position.y = 0.0;
        vacuumTCSdetach.pose.pose.position.z = 0.0;
        vacuumTCSdetach.pose.pose.orientation.x = 0.0;
        vacuumTCSdetach.pose.pose.orientation.y = 0.0;
        vacuumTCSdetach.pose.pose.orientation.z = 0.0;
        vacuumTCSdetach.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(vacuumTCSdetach);

        // above vacuum TCS position
        kuka_motion_plan_action::msg::Motionstate aboveVacuumTCS;
        aboveVacuumTCS.operation = "kuka_arm_lin";
        aboveVacuumTCS.pose.header.frame_id = "base_link_footprint";
        aboveVacuumTCS.pose.pose.position.x = -1.885;
        aboveVacuumTCS.pose.pose.position.y = 0.02;
        aboveVacuumTCS.pose.pose.position.z = 2.0;
        aboveVacuumTCS.pose.pose.orientation.x = 0.0;
        aboveVacuumTCS.pose.pose.orientation.y = 0.7071;
        aboveVacuumTCS.pose.pose.orientation.z = 0.0;
        aboveVacuumTCS.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(aboveVacuumTCS);
        
        // move Kuka into idle position
        kuka_motion_plan_action::msg::Motionstate idle;
        idle.operation = "KUKAidle";
        idle.pose.pose.position.x = 0.0;
        idle.pose.pose.position.y = 0.0;
        idle.pose.pose.position.z = 0.0;
        idle.pose.pose.orientation.x = 0.0;
        idle.pose.pose.orientation.y = 0.0;
        idle.pose.pose.orientation.z = 0.0;
        idle.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(idle);

        // move vacuumgripper above lid
        kuka_motion_plan_action::msg::Motionstate abovePack;
        abovePack.operation = "kuka_arm";
        abovePack.pose.header.frame_id = "base_link_footprint";
        abovePack.pose.pose.position.x = 2.2;
        abovePack.pose.pose.position.y = 0.0;
        abovePack.pose.pose.position.z = 1.7;
        abovePack.pose.pose.orientation.x = 0.0;
        abovePack.pose.pose.orientation.y = 0.0;
        abovePack.pose.pose.orientation.z = 0.0;
        abovePack.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(abovePack);

        // move vacuumgripper onto lid
        kuka_motion_plan_action::msg::Motionstate onLid;
        onLid.operation = "kuka_arm_lin";
        onLid.pose.header.frame_id = "base_link_footprint";
        onLid.pose.pose.position.x = 2.2;
        onLid.pose.pose.position.y = 0.0;
        onLid.pose.pose.position.z = 1.08;
        onLid.pose.pose.orientation.x = 0.0;
        onLid.pose.pose.orientation.y = 0.0;
        onLid.pose.pose.orientation.z = 0.0;
        onLid.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(onLid);

        // attach lid to vacuumgripper 
        kuka_motion_plan_action::msg::Motionstate deckelVakuumAttach;
        deckelVakuumAttach.operation = "deckelVakuumattach";
        deckelVakuumAttach.pose.header.frame_id = "base_link_footprint";
        deckelVakuumAttach.pose.pose.position.x = -1.885;
        deckelVakuumAttach.pose.pose.position.y = 0.02;
        deckelVakuumAttach.pose.pose.position.z = 1.20;
        deckelVakuumAttach.pose.pose.orientation.x = 0.0;
        deckelVakuumAttach.pose.pose.orientation.y = 0.7071;
        deckelVakuumAttach.pose.pose.orientation.z = 0.0;
        deckelVakuumAttach.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(deckelVakuumAttach);

        // detach lid from pack
        kuka_motion_plan_action::msg::Motionstate deckelPackDetach;
        deckelPackDetach.operation = "deckeldetach";
        deckelPackDetach.pose.header.frame_id = "base_link_footprint";
        deckelPackDetach.pose.pose.position.x = -1.885;
        deckelPackDetach.pose.pose.position.y = 0.02;
        deckelPackDetach.pose.pose.position.z = 1.20;
        deckelPackDetach.pose.pose.orientation.x = 0.0;
        deckelPackDetach.pose.pose.orientation.y = 0.7071;
        deckelPackDetach.pose.pose.orientation.z = 0.0;
        deckelPackDetach.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(deckelPackDetach);

        // lift lid from pack
        abovePack.operation = "kuka_arm_lin";
        abovePack.pose.pose.position.z = 2.0;
        MotionStateArray.statemachinearray.push_back(abovePack);

        // move lid above table
        kuka_motion_plan_action::msg::Motionstate abovePallet;
        abovePallet.operation = "kuka_arm";
        abovePallet.pose.header.frame_id = "base_link_footprint";
        abovePallet.pose.pose.position.x = 0.23;
        abovePallet.pose.pose.position.y = 2.0;
        abovePallet.pose.pose.position.z = 2.0;
        abovePallet.pose.pose.orientation.x = 0.0;
        abovePallet.pose.pose.orientation.y = 0.0;
        abovePallet.pose.pose.orientation.z = 0.7071;
        abovePallet.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(abovePallet);

        // move lid onto table
        kuka_motion_plan_action::msg::Motionstate pallet;
        pallet.operation = "kuka_arm_lin";
        pallet.pose.header.frame_id = "base_link_footprint";
        pallet.pose.pose.position.x = 0.23;
        pallet.pose.pose.position.y = 2.0;
        pallet.pose.pose.position.z = 1.28;
        pallet.pose.pose.orientation.x = 0.0;
        pallet.pose.pose.orientation.y = 0.0;
        pallet.pose.pose.orientation.z = 0.7071;
        pallet.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(pallet);

        // detach lid from vacuumgripper
        MotionStateArray.statemachinearray.push_back(deckelVakuumDetach);

        // move vacuumgripper above palette
        abovePallet.pose.pose.position.z = 1.5;
        MotionStateArray.statemachinearray.push_back(abovePallet);

        //=====================================================================
        // Step 2: Move Battery Pack to WS2 for manual steps
        //         Move KUKA to WS3 and prepare for EIS measurement
        //=====================================================================

        //move AGV to WS2
        kuka_motion_plan_action::msg::Motionstate AGVWS2;
        AGVWS2.operation = "agv";
        AGVWS2.pose.header.frame_id = "world";
        AGVWS2.pose.pose.position.x = 6.0;
        AGVWS2.pose.pose.position.y = -5.0;
        AGVWS2.pose.pose.position.z = 0.0;
        AGVWS2.pose.pose.orientation.x = 0.0;
        AGVWS2.pose.pose.orientation.y = 0.0;
        AGVWS2.pose.pose.orientation.z = -0.7071;
        AGVWS2.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(AGVWS2);

        // initiate tool changing
        MotionStateArray.statemachinearray.push_back(TCSeis);

        aboveVacuumTCS.operation = "kuka_arm";
        MotionStateArray.statemachinearray.push_back(aboveVacuumTCS);

        MotionStateArray.statemachinearray.push_back(VacuumTCS);

        kuka_motion_plan_action::msg::Motionstate vacuumTCSattach;
        vacuumTCSattach.operation = "vacuumTCSattach";
        vacuumTCSattach.pose.header.frame_id = "base_link_footprint";
        vacuumTCSattach.pose.pose.position.x = 0.0;
        vacuumTCSattach.pose.pose.position.y = 0.0;
        vacuumTCSattach.pose.pose.position.z = 0.0;
        vacuumTCSattach.pose.pose.orientation.x = 0.0;
        vacuumTCSattach.pose.pose.orientation.y = 0.0;
        vacuumTCSattach.pose.pose.orientation.z = 0.0;
        vacuumTCSattach.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(vacuumTCSattach);

        MotionStateArray.statemachinearray.push_back(vacuumKUKAdetach);

        frontVacuumTCS.operation =  "kuka_arm_lin";
        MotionStateArray.statemachinearray.push_back(frontVacuumTCS);

        //above eis TCS position
        kuka_motion_plan_action::msg::Motionstate aboveEISTCS;
        aboveEISTCS.operation = "kuka_arm";
        aboveEISTCS.pose.header.frame_id = "base_link_footprint";
        aboveEISTCS.pose.pose.position.x = -1.79;
        aboveEISTCS.pose.pose.position.y = 0.43;
        aboveEISTCS.pose.pose.position.z = 1.55;
        aboveEISTCS.pose.pose.orientation.x = 0.0;
        aboveEISTCS.pose.pose.orientation.y = 0.0;
        aboveEISTCS.pose.pose.orientation.z = 0.0;
        aboveEISTCS.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(aboveEISTCS);

        //eis TCS position
        kuka_motion_plan_action::msg::Motionstate EISTCS;
        EISTCS.operation = "kuka_arm_lin";
        EISTCS.pose.header.frame_id = "base_link_footprint";
        EISTCS.pose.pose.position.x = -1.79;
        EISTCS.pose.pose.position.y = 0.43;
        EISTCS.pose.pose.position.z = 1.422;
        EISTCS.pose.pose.orientation.x = 0.0;
        EISTCS.pose.pose.orientation.y = 0.0;
        EISTCS.pose.pose.orientation.z = 0.0;
        EISTCS.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(EISTCS);

        kuka_motion_plan_action::msg::Motionstate EISKUKAattach;
        EISKUKAattach.operation = "eisKUKAattach";
        EISKUKAattach.pose = EISKUKAdetach.pose;
        MotionStateArray.statemachinearray.push_back(EISKUKAattach);

        kuka_motion_plan_action::msg::Motionstate EISTCSdetach;
        EISTCSdetach.operation = "eisTCSdetach";
        EISTCSdetach.pose = EISKUKAdetach.pose;
        MotionStateArray.statemachinearray.push_back(EISTCSdetach);

        //in front of eis TCS position
        kuka_motion_plan_action::msg::Motionstate frontEISTCS;
        frontEISTCS.operation = "kuka_arm_lin";
        frontEISTCS.pose.header.frame_id = "base_link_footprint";
        frontEISTCS.pose.pose.position.x = -1.49;
        frontEISTCS.pose.pose.position.y = 0.43;
        frontEISTCS.pose.pose.position.z = 1.421;
        frontEISTCS.pose.pose.orientation.x = 0.0;
        frontEISTCS.pose.pose.orientation.y = 0.0;
        frontEISTCS.pose.pose.orientation.z = 0.0;
        frontEISTCS.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(frontEISTCS);

        MotionStateArray.statemachinearray.push_back(idle);

        // move KUKA to WS3
        kuka_motion_plan_action::msg::Motionstate omnibaseWS3;
        omnibaseWS3.operation = "omnibase";
        omnibaseWS3.pose.header.frame_id = "world";
        omnibaseWS3.pose.pose.position.x = -6.89567;
        omnibaseWS3.pose.pose.position.y = 4.99753;
        omnibaseWS3.pose.pose.position.z = 0.0;
        omnibaseWS3.pose.pose.orientation.x = 0.0;
        omnibaseWS3.pose.pose.orientation.y = 0.0;
        omnibaseWS3.pose.pose.orientation.z = 0.0;
        omnibaseWS3.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(omnibaseWS3);

        //=====================================================================
        // Step 3: EIS measurement
        //=====================================================================

        //move AGV to WS3
        kuka_motion_plan_action::msg::Motionstate AGVWS3;
        AGVWS3.operation = "agv";
        AGVWS3.pose.header.frame_id = "world";
        AGVWS3.pose.pose.position.x = -3.8;
        AGVWS3.pose.pose.position.y = 4.0;
        AGVWS3.pose.pose.position.z = 0.0;
        AGVWS3.pose.pose.orientation.x = 0.0;
        AGVWS3.pose.pose.orientation.y = 0.0;
        AGVWS3.pose.pose.orientation.z = -0.7071;
        AGVWS3.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(AGVWS3);

        // move above battery module 0
        kuka_motion_plan_action::msg::Motionstate abovePose1;
        abovePose1.operation = "kuka_arm";
        abovePose1.pose.header.frame_id = "base_link_footprint";
        abovePose1.pose.pose.position.x = 2.2657;
        abovePose1.pose.pose.position.y = -0.8485;
        abovePose1.pose.pose.position.z = 1.1;
        abovePose1.pose.pose.orientation.x = 0.0;
        abovePose1.pose.pose.orientation.y = 0.0;
        abovePose1.pose.pose.orientation.z = 0.7071;
        abovePose1.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(abovePose1);

        // move onto battery module 0
        kuka_motion_plan_action::msg::Motionstate pose1;
        pose1.operation = "kuka_arm_lin";
        pose1.pose.header.frame_id = "base_link_footprint";
        pose1.pose.pose.position.x = 2.2657;
        pose1.pose.pose.position.y = -0.8485;
        pose1.pose.pose.position.z = 0.9767;
        pose1.pose.pose.orientation.x = 0.0;
        pose1.pose.pose.orientation.y = 0.0;
        pose1.pose.pose.orientation.z = 0.7071;
        pose1.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(pose1);

        // wait for EIS
        pause.operation = "pause_4";
        MotionStateArray.statemachinearray.push_back(pause);

        abovePose1.operation = "kuka_arm_lin";
        MotionStateArray.statemachinearray.push_back(abovePose1);

        // move above battery module 5
        kuka_motion_plan_action::msg::Motionstate abovePose6;
        abovePose6.operation = "kuka_arm";
        abovePose6.pose.header.frame_id = "base_link_footprint";
        abovePose6.pose.pose.position.x = 2.2657;
        abovePose6.pose.pose.position.y = -0.0065;
        abovePose6.pose.pose.position.z = 1.1;
        abovePose6.pose.pose.orientation.x = 0.0;
        abovePose6.pose.pose.orientation.y = 0.0;
        abovePose6.pose.pose.orientation.z = 0.7071;
        abovePose6.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(abovePose6);

        kuka_motion_plan_action::msg::Motionstate pose6;
        pose6.operation = "kuka_arm";
        pose6.pose.header.frame_id = "base_link_footprint";
        pose6.pose.pose.position.x = 2.2657;
        pose6.pose.pose.position.y = -0.0065;
        pose6.pose.pose.position.z = 0.9767;
        pose6.pose.pose.orientation.x = 0.0;
        pose6.pose.pose.orientation.y = 0.0;
        pose6.pose.pose.orientation.z = 0.7071;
        pose6.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(pose6);

        pause.operation = "pause_4";
        MotionStateArray.statemachinearray.push_back(pause);

        abovePose6.operation = "kuka_arm_lin";
        MotionStateArray.statemachinearray.push_back(abovePose6);

        // move above battery module 11
        kuka_motion_plan_action::msg::Motionstate abovePose12;
        abovePose12.operation = "kuka_arm";
        abovePose12.pose.header.frame_id = "base_link_footprint";
        abovePose12.pose.pose.position.x = 1.9851;
        abovePose12.pose.pose.position.y = 0.6265;
        abovePose12.pose.pose.position.z = 1.1;
        abovePose12.pose.pose.orientation.x = 0.0;
        abovePose12.pose.pose.orientation.y = 0.0;
        abovePose12.pose.pose.orientation.z = 1.0;
        abovePose12.pose.pose.orientation.w = 0.0;
        MotionStateArray.statemachinearray.push_back(abovePose12);

        kuka_motion_plan_action::msg::Motionstate pose12;
        pose12.operation = "kuka_arm";
        pose12.pose.header.frame_id = "base_link_footprint";
        pose12.pose.pose.position.x = 1.9851;
        pose12.pose.pose.position.y = 0.6265;
        pose12.pose.pose.position.z = 0.9767;
        pose12.pose.pose.orientation.x = 0.0;
        pose12.pose.pose.orientation.y = 0.0;
        pose12.pose.pose.orientation.z = 1.0;
        pose12.pose.pose.orientation.w = 0.0;
        MotionStateArray.statemachinearray.push_back(pose12);
        
        pause.operation = "pause_4";
        MotionStateArray.statemachinearray.push_back(pause);

        abovePose12.operation = "kuka_arm_lin";
        MotionStateArray.statemachinearray.push_back(abovePose12);

        // initiate tool changing to gripper
        MotionStateArray.statemachinearray.push_back(TCSeis);

        frontEISTCS.operation = "kuka_arm";
        MotionStateArray.statemachinearray.push_back(frontEISTCS);

        MotionStateArray.statemachinearray.push_back(EISTCS);

        kuka_motion_plan_action::msg::Motionstate EISTCSattach;
        EISTCSattach.operation = "eisTCSattach";
        EISTCSattach.pose = EISKUKAdetach.pose;
        MotionStateArray.statemachinearray.push_back(EISTCSattach);

        MotionStateArray.statemachinearray.push_back(EISKUKAdetach);

        aboveEISTCS.operation = "kuka_arm_lin";
        MotionStateArray.statemachinearray.push_back(aboveEISTCS);

        //=====================================================================
        // Step 4: Remove battery module
        //=====================================================================

        //above gripper TCS position
        kuka_motion_plan_action::msg::Motionstate aboveGripperTCS;
        aboveGripperTCS.operation = "kuka_arm";
        aboveGripperTCS.pose.header.frame_id = "base_link_footprint";
        aboveGripperTCS.pose.pose.position.x = -1.79;
        aboveGripperTCS.pose.pose.position.y = 0.02;
        aboveGripperTCS.pose.pose.position.z = 1.55;
        aboveGripperTCS.pose.pose.orientation.x = 0.0;
        aboveGripperTCS.pose.pose.orientation.y = 0.0;
        aboveGripperTCS.pose.pose.orientation.z = 0.0;
        aboveGripperTCS.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(aboveGripperTCS);

        //gripper TCS position
        kuka_motion_plan_action::msg::Motionstate gripperTCS;
        gripperTCS.operation = "kuka_arm_lin";
        gripperTCS.pose.header.frame_id = "base_link_footprint";
        gripperTCS.pose.pose.position.x = -1.79;
        gripperTCS.pose.pose.position.y = 0.02;
        gripperTCS.pose.pose.position.z = 1.422;
        gripperTCS.pose.pose.orientation.x = 0.0;
        gripperTCS.pose.pose.orientation.y = 0.0;
        gripperTCS.pose.pose.orientation.z = 0.0;
        gripperTCS.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(gripperTCS);

        kuka_motion_plan_action::msg::Motionstate gripperKUKAattach;
        gripperKUKAattach.operation = "gripperKUKAattach";
        gripperKUKAattach.pose = EISKUKAdetach.pose;
        MotionStateArray.statemachinearray.push_back(gripperKUKAattach);

        kuka_motion_plan_action::msg::Motionstate gripperTCSdetach;
        gripperTCSdetach.operation = "gripperTCSdetach";
        gripperTCSdetach.pose = EISKUKAdetach.pose;
        MotionStateArray.statemachinearray.push_back(gripperTCSdetach);

        //in front of gripper TCS position
        kuka_motion_plan_action::msg::Motionstate frontGripperTCS;
        frontGripperTCS.operation = "kuka_arm_lin";
        frontGripperTCS.pose.header.frame_id = "base_link_footprint";
        frontGripperTCS.pose.pose.position.x = -1.35;
        frontGripperTCS.pose.pose.position.y = 0.02;
        frontGripperTCS.pose.pose.position.z = 1.42;
        frontGripperTCS.pose.pose.orientation.x = 0.0;
        frontGripperTCS.pose.pose.orientation.y = 0.0;
        frontGripperTCS.pose.pose.orientation.z = 0.0;
        frontGripperTCS.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(frontGripperTCS);

        MotionStateArray.statemachinearray.push_back(idle);

        // move above battery module 0
        kuka_motion_plan_action::msg::Motionstate abovePose1Grip;
        abovePose1Grip.operation = "kuka_arm";
        abovePose1Grip.pose.header.frame_id = "base_link_footprint";
        abovePose1Grip.pose.pose.position.x = 2.2657;
        abovePose1Grip.pose.pose.position.y = -0.8485;
        abovePose1Grip.pose.pose.position.z = 1.5;
        abovePose1Grip.pose.pose.orientation.x = 0.0;
        abovePose1Grip.pose.pose.orientation.y = 0.0;
        abovePose1Grip.pose.pose.orientation.z = 0.0;
        abovePose1Grip.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(abovePose1Grip);

        // move onto battery module 0
        kuka_motion_plan_action::msg::Motionstate pose1Grip;
        pose1Grip.operation = "kuka_arm_lin";
        pose1Grip.pose.header.frame_id = "base_link_footprint";
        pose1Grip.pose.pose.position.x = 2.2657;
        pose1Grip.pose.pose.position.y = -0.8485;
        pose1Grip.pose.pose.position.z = 1.12;
        pose1Grip.pose.pose.orientation.x = 0.0;
        pose1Grip.pose.pose.orientation.y = 0.0;
        pose1Grip.pose.pose.orientation.z = 0.0;
        pose1Grip.pose.pose.orientation.w = 1.0;
        MotionStateArray.statemachinearray.push_back(pose1Grip);

        // Attach battery modules to gripper
        kuka_motion_plan_action::msg::Motionstate attachModule1;
        attachModule1.operation = "attachModule_0";
        attachModule1.pose.header.frame_id = "base_link_footprint";
        attachModule1.pose.pose.position.x = -1.885;
        attachModule1.pose.pose.position.y = 0.02;
        attachModule1.pose.pose.position.z = 1.75;
        attachModule1.pose.pose.orientation.x = 0.0;
        attachModule1.pose.pose.orientation.y = 0.7071;
        attachModule1.pose.pose.orientation.z = 0.0;
        attachModule1.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(attachModule1);

        // Detach battery modules from pack
        kuka_motion_plan_action::msg::Motionstate detachPackModule1;
        detachPackModule1.operation = "detachPackModule_0";
        detachPackModule1.pose.header.frame_id = "base_link_footprint";
        detachPackModule1.pose.pose.position.x = -1.885;
        detachPackModule1.pose.pose.position.y = 0.02;
        detachPackModule1.pose.pose.position.z = 1.75;
        detachPackModule1.pose.pose.orientation.x = 0.0;
        detachPackModule1.pose.pose.orientation.y = 0.7071;
        detachPackModule1.pose.pose.orientation.z = 0.0;
        detachPackModule1.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(detachPackModule1);

        abovePose1Grip.pose.pose.position.z = 2.0;
        MotionStateArray.statemachinearray.push_back(abovePose1Grip);

        // move module above palette
        kuka_motion_plan_action::msg::Motionstate abovepallet2;
        abovepallet2.operation = "kuka_arm";
        abovepallet2.pose.header.frame_id = "base_link_footprint";
        abovepallet2.pose.pose.position.x = 0.0;
        abovepallet2.pose.pose.position.y = 2.0;
        abovepallet2.pose.pose.position.z = 2.0;
        abovepallet2.pose.pose.orientation.x = 0.0;
        abovepallet2.pose.pose.orientation.y = 0.0;
        abovepallet2.pose.pose.orientation.z = 0.7071;
        abovepallet2.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(abovepallet2);

        // move module onto palette
        kuka_motion_plan_action::msg::Motionstate pallet2;
        pallet2.operation = "kuka_arm";
        pallet2.pose.header.frame_id = "base_link_footprint";
        pallet2.pose.pose.position.x = 0.0;
        pallet2.pose.pose.position.y = 2.0;
        pallet2.pose.pose.position.z = 1.49;
        pallet2.pose.pose.orientation.x = 0.0;
        pallet2.pose.pose.orientation.y = 0.0;
        pallet2.pose.pose.orientation.z = 0.7071;
        pallet2.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(pallet2);

        kuka_motion_plan_action::msg::Motionstate detachGripperModule1;
        detachGripperModule1.operation = "detachModule_0";
        detachGripperModule1.pose.header.frame_id = "base_link_footprint";
        detachGripperModule1.pose.pose.position.x = -1.885;
        detachGripperModule1.pose.pose.position.y = 0.02;
        detachGripperModule1.pose.pose.position.z = 1.75;
        detachGripperModule1.pose.pose.orientation.x = 0.0;
        detachGripperModule1.pose.pose.orientation.y = 0.7071;
        detachGripperModule1.pose.pose.orientation.z = 0.0;
        detachGripperModule1.pose.pose.orientation.w = 0.7071;
        MotionStateArray.statemachinearray.push_back(detachGripperModule1);

        MotionStateArray.statemachinearray.push_back(abovepallet2);

        // initiate tool changing to no tool
        MotionStateArray.statemachinearray.push_back(TCSeis);

        frontGripperTCS.operation = "kuka_arm";
        frontGripperTCS.pose.pose.position.z = 1.65;
        MotionStateArray.statemachinearray.push_back(frontGripperTCS);

        frontGripperTCS.pose.pose.position.z = 1.42;
        MotionStateArray.statemachinearray.push_back(frontGripperTCS);

        MotionStateArray.statemachinearray.push_back(gripperTCS);

        kuka_motion_plan_action::msg::Motionstate gripperTCSattach;
        gripperTCSattach.operation = "gripperTCSattach";
        gripperTCSattach.pose = EISKUKAdetach.pose;
        MotionStateArray.statemachinearray.push_back(gripperTCSattach);

        MotionStateArray.statemachinearray.push_back(gripperKUKAdetach);

        aboveGripperTCS.operation = "kuka_arm_lin";
        MotionStateArray.statemachinearray.push_back(aboveGripperTCS);

        MotionStateArray.statemachinearray.push_back(idle);

        // Publish the poseArray
        publisher_->publish(MotionStateArray);
        RCLCPP_INFO(this->get_logger(), "Published a motion sequence with %zu motions.", MotionStateArray.statemachinearray.size());
    }

    rclcpp::Publisher<kuka_motion_plan_action::msg::Motionstatearray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionStateGeneratorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

const double tau = 2 * M_PI;

int main(int argc, char **argv)
{
    // ROS2 Initialization
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("move_group_interface_node");
    auto logger = rclcpp::get_logger("move_group_interface");

    // Spinner for asynchronous handling
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinner_thread([&executor]() { executor.spin(); });

    rclcpp::sleep_for(std::chrono::seconds(2));

    using moveit::planning_interface::MoveGroupInterface;
    
    // Initialize both group interfaces
    MoveGroupInterface move_group(node, "arm");
    move_group.setPoseReferenceFrame("panda_link0");
    move_group.setPlanningTime(10.0);

    MoveGroupInterface gripper_group(node, "gripper");
    gripper_group.setPlanningTime(10.0);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // ==========================================
    // STEP 1: PLAN AND EXECUTE THE ARM PATH
    // ==========================================
    geometry_msgs::msg::Pose target_pose;
    tf2::Quaternion orientation;
    orientation.setRPY(3.14, 0, 0);
    target_pose.orientation = tf2::toMsg(orientation);
    target_pose.position.x = 0.4; // Brought closer slightly to stay within standard reaching workspace
    target_pose.position.y = 0.3;
    target_pose.position.z = 0.4;

    move_group.setPoseTarget(target_pose, "panda_link7");

    MoveGroupInterface::Plan arm_plan;
    bool arm_success = (move_group.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (arm_success)
    {
        RCLCPP_INFO(logger, "Arm Planning: SUCCESS. Moving arm to target pose...");
        move_group.move(); // Blocks until the arm reaches the location
        RCLCPP_INFO(logger, "Arm arrival confirmed.");
    }
    else
    {
        RCLCPP_ERROR(logger, "Arm Planning: FAILED! Skipping gripper operations.");
    }

    // ==========================================
    // STEP 2: PLAN AND EXECUTE THE GRIPPER PATH
    // ==========================================
    if (arm_success) 
    {
        gripper_group.setNamedTarget("oppen");

        MoveGroupInterface::Plan gripper_plan;
        bool gripper_success = (gripper_group.plan(gripper_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (gripper_success)
        {
            RCLCPP_INFO(logger, "Gripper Planning: SUCCESS. Closing gripper...");
            gripper_group.move(); // Blocks until the fingers complete their close sequence
            RCLCPP_INFO(logger, "Gripper action completed.");
        }
        else
        {
            RCLCPP_ERROR(logger, "Gripper Planning: FAILED!");
        }
    }

    // Shutdown processing safely
    rclcpp::shutdown();
    spinner_thread.join();
    return 0;
}
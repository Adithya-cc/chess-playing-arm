#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "franka_description/action/pick_and_place.hpp"

class PickAndPlaceExecutor
{
public:
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

    explicit PickAndPlaceExecutor(const std::shared_ptr<rclcpp::Node>& node)
        : node_(node),
          arm_group_(node_, "arm"),
          gripper_group_(node_, "gripper")
    {
        // Configure Arm Group
        arm_group_.setPoseReferenceFrame("panda_link0");
        arm_group_.setPlanningTime(5.0);
        arm_group_.setMaxVelocityScalingFactor(0.5);
        arm_group_.setMaxAccelerationScalingFactor(0.5);

        // Configure Gripper Group
        gripper_group_.setPlanningTime(5.0);

        // Orientation default (tool facing straight down)
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, 0); // Roll 180 deg for top-down grip
        default_orientation_ = tf2::toMsg(q);
    }

    void stop()
    {
        arm_group_.stop();
        gripper_group_.stop();
    }

    // Execute the complete Pick and Place state machine with Action Feedback
    bool executePickAndPlace(
        const geometry_msgs::msg::Pose& source_pose,
        const geometry_msgs::msg::Pose& target_pose,
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<franka_description::action::PickAndPlace>>& goal_handle,
        double clearance_offset = 0.18) // 15 cm approach clearance
    {
        auto feedback = std::make_shared<franka_description::action::PickAndPlace::Feedback>();
        auto result = std::make_shared<franka_description::action::PickAndPlace::Result>();

        auto update_stage = [&](const std::string& stage_name, float progress) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Goal Canceled during: " + stage_name;
                goal_handle->canceled(result);
                return false;
            }
            feedback->current_stage = stage_name;
            feedback->percentage_complete = progress;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(node_->get_logger(), "[Progress %.0f%%] %s", progress, stage_name.c_str());
            return true;
        };

        RCLCPP_INFO(node_->get_logger(), "Starting Pick & Place Sequence...");

        // 0. Ensure Gripper is Open
        if (!update_stage("Opening Gripper", 5.0) || !setGripperState("open")) goto execution_failed;

        // 1. Move ABOVE Source (Pre-Pick)
        {
            geometry_msgs::msg::Pose source_above = createPose(source_pose.position.x, source_pose.position.y, source_pose.position.z + clearance_offset);
            if (!update_stage("Step 1: Moving above source", 15.0) || !moveToPose(source_above)) goto execution_failed;

            // 2. Descend to Source (Pick Position)
            geometry_msgs::msg::Pose source_pick = createPose(source_pose.position.x, source_pose.position.y, source_pose.position.z);
            if (!update_stage("Step 2: Descending to pick position", 30.0) || !executeCartesianPath({source_above, source_pick})) goto execution_failed;
        }

        // 3. Close Gripper (Grasp Object)
        if (!update_stage("Step 3: Closing gripper", 45.0) || !setGripperState("close")) goto execution_failed;

        // 4. Lift Object (Post-Pick)
        {
            geometry_msgs::msg::Pose source_pick = createPose(source_pose.position.x, source_pose.position.y, source_pose.position.z);
            geometry_msgs::msg::Pose source_above = createPose(source_pose.position.x, source_pose.position.y, source_pose.position.z + clearance_offset);
            if (!update_stage("Step 4: Retracting upwards with object", 55.0) || !executeCartesianPath({source_pick, source_above})) goto execution_failed;

            // 5. Move ABOVE Target (Pre-Place)
            geometry_msgs::msg::Pose target_above = createPose(target_pose.position.x, target_pose.position.y, target_pose.position.z + clearance_offset);
            if (!update_stage("Step 5: Transporting to target overhead", 70.0) || !moveToPose(target_above)) goto execution_failed;

            // 6. Descend to Target (Place Position)
            geometry_msgs::msg::Pose target_place = createPose(target_pose.position.x, target_pose.position.y, target_pose.position.z);
            if (!update_stage("Step 6: Descending to place target", 80.0) || !executeCartesianPath({target_above, target_place})) goto execution_failed;
        }

        // 7. Open Gripper (Release Object)
        if (!update_stage("Step 7: Opening gripper", 88.0) || !setGripperState("open")) goto execution_failed;

        // 8. Retract Upward from Target
        {
            geometry_msgs::msg::Pose target_place = createPose(target_pose.position.x, target_pose.position.y, target_pose.position.z);
            geometry_msgs::msg::Pose target_above = createPose(target_pose.position.x, target_pose.position.y, target_pose.position.z + clearance_offset);
            if (!update_stage("Step 8: Retracting upward", 93.0) || !executeCartesianPath({target_place, target_above})) goto execution_failed;
        }

        // 9. Return to Home Position
        if (!update_stage("Step 9: Returning to Home position", 98.0) || !moveToNamedTarget("home")) goto execution_failed;

        result->success = true;
        result->message = "Pick and Place Sequence Completed Successfully!";
        goal_handle->succeed(result);
        RCLCPP_INFO(node_->get_logger(), "%s", result->message.c_str());
        return true;

    execution_failed:
        result->success = false;
        result->message = "Execution failed during stage: " + feedback->current_stage;
        goal_handle->abort(result);
        RCLCPP_ERROR(node_->get_logger(), "Goal Aborted: %s", result->message.c_str());
        return false;
    }

private:
    geometry_msgs::msg::Pose createPose(double x, double y, double z)
    {
        geometry_msgs::msg::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation = default_orientation_;
        return pose;
    }

    bool moveToPose(const geometry_msgs::msg::Pose& target_pose)
    {
        arm_group_.setPoseTarget(target_pose, "panda_link7");
        MoveGroupInterface::Plan plan;
        bool success = (arm_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            arm_group_.execute(plan);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Free-space planning failed!");
        }
        return success;
    }

    bool moveToNamedTarget(const std::string& target_name)
    {
        arm_group_.setNamedTarget(target_name);
        MoveGroupInterface::Plan plan;
        bool success = (arm_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            arm_group_.execute(plan);
        }
        return success;
    }

    bool executeCartesianPath(const std::vector<geometry_msgs::msg::Pose>& waypoints)
    {
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;

        double fraction = arm_group_.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        
        if (fraction >= 0.95) {
            arm_group_.execute(trajectory);
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning incomplete (Only %.2f%% planned)", fraction * 100.0);
            return false;
        }
    }

    bool setGripperState(const std::string& state)
    {
        std::string target = (state == "open") ? "open" : "close";
        gripper_group_.setNamedTarget(target);
        
        MoveGroupInterface::Plan plan;
        bool success = (gripper_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            gripper_group_.execute(plan);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Failed to plan gripper state: %s", target.c_str());
        }
        return success;
    }

    std::shared_ptr<rclcpp::Node> node_;
    MoveGroupInterface arm_group_;
    MoveGroupInterface gripper_group_;
    geometry_msgs::msg::Quaternion default_orientation_;
};

class PickAndPlaceActionServer : public rclcpp::Node
{
public:
    using PickAndPlace = franka_description::action::PickAndPlace;
    using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

    explicit PickAndPlaceActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("pick_and_place_action_server", options)
    {
    }

    void init()
    {
        action_server_ = rclcpp_action::create_server<PickAndPlace>(
            this,
            "pick_and_place",
            std::bind(&PickAndPlaceActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PickAndPlaceActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&PickAndPlaceActionServer::handle_accepted, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Pick and Place Action Server initialized and waiting for goals...");
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const PickAndPlace::Goal> goal)
    {
        (void)uuid;
        (void)goal;
        RCLCPP_INFO(this->get_logger(), "Received Pick and Place goal request.");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_WARN(this->get_logger(), "Received request to cancel goal execution!");
        if (runner_) {
            runner_->stop();
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        std::thread{std::bind(&PickAndPlaceActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        // Lazy-initialize the runner inside the thread to prevent ROS middleware startup timeouts
        if (!runner_) {
            runner_ = std::make_unique<PickAndPlaceExecutor>(shared_from_this());
        }

        const auto goal = goal_handle->get_goal();
        runner_->executePickAndPlace(goal->source_pose, goal->target_pose, goal_handle);
    }

    std::unique_ptr<PickAndPlaceExecutor> runner_;
    rclcpp_action::Server<PickAndPlace>::SharedPtr action_server_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlaceActionServer>();
    node->init();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
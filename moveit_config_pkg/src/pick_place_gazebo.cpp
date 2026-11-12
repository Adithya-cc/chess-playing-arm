#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <nlohmann/json.hpp>

// ROS 2 Link Attacher Service Headers
#include <linkattacher_msgs/srv/attach_link.hpp>
#include <linkattacher_msgs/srv/detach_link.hpp>

// Action Header
#include "franka_description/action/pick_and_place.hpp"

class PickAndPlaceExecutor
{
public:
    using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
    using PickAndPlace = franka_description::action::PickAndPlace;
    using GoalHandlePickAndPlace = rclcpp_action::ServerGoalHandle<PickAndPlace>;

    explicit PickAndPlaceExecutor(const std::shared_ptr<rclcpp::Node>& node)
        : node_(node),
          arm_group_(node_, "arm"),
          gripper_group_(node_, "gripper"),
          json_path_("/home/adithya-con006/arm_ws/src/moveit_config_pkg/config/chessboard_poses.json")
    {
        // Reference frame setup
        arm_group_.setPoseReferenceFrame("panda_link0");
        arm_group_.setEndEffectorLink("panda_hand");
        arm_group_.setPlanningTime(10.0);
        arm_group_.setNumPlanningAttempts(10);
        arm_group_.setMaxVelocityScalingFactor(0.3);
        arm_group_.setMaxAccelerationScalingFactor(0.3);

        // Configure Gripper Group
        gripper_group_.setPlanningTime(5.0);

        // Service clients for IFRA Link Attacher
        attach_client_ = node_->create_client<linkattacher_msgs::srv::AttachLink>("/ATTACHLINK");
        detach_client_ = node_->create_client<linkattacher_msgs::srv::DetachLink>("/DETACHLINK");

        // End-effector default orientation (tool facing straight down)
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, 0); // Roll 180 deg for top-down grip
        default_orientation_ = tf2::toMsg(q);

        // 1. Add Environment (Table & Board)
        addTableCollisionObject();

        // 2. Initialize ROS 2 Action Server
        action_server_ = rclcpp_action::create_server<PickAndPlace>(
            node_,
            "pick_and_place",
            std::bind(&PickAndPlaceExecutor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PickAndPlaceExecutor::handle_cancel, this, std::placeholders::_1),
            std::bind(&PickAndPlaceExecutor::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "Pick & Place Action Server Initialized and Ready!");
    }

    // Dynamic square-to-pose resolution from JSON or Graveyard offset
    bool getPoseFromSquare(const std::string& square_name, geometry_msgs::msg::Point& pt)
    {
        // -------------------------------------------------------------
        // DYNAMIC GRAVEYARD DISPATCH
        // -------------------------------------------------------------
        if (square_name == "GRAVEYARD") {
            // Compute dynamic drop location for captured piece
            pt.x = graveyard_start_x_ - (captured_count_ * graveyard_x_step_);
            pt.y = graveyard_y_;
            pt.z = 0.18; // Drop height matching piece placement

            RCLCPP_INFO(node_->get_logger(), 
                        "Resolved 'GRAVEYARD' (Capture #%d) -> Frame Pos: [%.4f, %.4f, %.4f]", 
                        captured_count_ + 1, pt.x, pt.y, pt.z);

            // Increment capture counter for next piece placement offset
            captured_count_++;
            return true;
        }

        // -------------------------------------------------------------
        // STANDARD CHESSBOARD LOOKUP
        // -------------------------------------------------------------
        std::ifstream file(json_path_);
        if (!file.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open board poses JSON: %s", json_path_.c_str());
            return false;
        }

        nlohmann::json board_poses;
        file >> board_poses;

        if (!board_poses.contains(square_name)) {
            RCLCPP_ERROR(node_->get_logger(), "Square '%s' not found in JSON file!", square_name.c_str());
            return false;
        }

        const double panda_base_x_offset = -0.50;

        double raw_x = board_poses[square_name][0].get<double>();
        double raw_y = board_poses[square_name][1].get<double>();

        pt.x = raw_x - panda_base_x_offset;
        pt.y = raw_y;
        pt.z = 0.18; // Fixed grasp height override

        RCLCPP_INFO(node_->get_logger(), "Resolved '%s' -> Frame Pos: [%.4f, %.4f, %.4f]", 
                    square_name.c_str(), pt.x, pt.y, pt.z);
        return true;
    }

    void addTableCollisionObject()
    {   
        std::vector<moveit_msgs::msg::CollisionObject> collision_objects;

        // Table Object
        moveit_msgs::msg::CollisionObject table;
        table.header.frame_id = "world";
        table.id = "table_surface";

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {1.524, 0.914, 1.011}; 

        geometry_msgs::msg::Pose table_pose;
        table_pose.orientation.w = 1.0;
        table_pose.position.x = 0.0;
        table_pose.position.y = 0.0;
        table_pose.position.z = 0.5055; 

        table.primitives.push_back(primitive);
        table.primitive_poses.push_back(table_pose);
        table.operation = table.ADD;
        collision_objects.push_back(table);

        // Chessboard Object
        moveit_msgs::msg::CollisionObject chessboard;
        chessboard.header.frame_id = "world";
        chessboard.id = "chessboard";

        shape_msgs::msg::SolidPrimitive board_primitive;
        board_primitive.type = board_primitive.BOX;
        board_primitive.dimensions = {0.50, 0.50, 0.02};

        geometry_msgs::msg::Pose chessboard_pose;
        chessboard_pose.orientation.w = 1.0;
        chessboard_pose.position.x = 0.0; 
        chessboard_pose.position.y = 0.0;
        chessboard_pose.position.z = 1.031; 

        chessboard.primitives.push_back(board_primitive);
        chessboard.primitive_poses.push_back(chessboard_pose);
        chessboard.operation = chessboard.ADD;
        collision_objects.push_back(chessboard);

        planning_scene_interface_.applyCollisionObjects(collision_objects);
        RCLCPP_INFO(node_->get_logger(), "Table and Chessboard added to Planning Scene!");
    }

    void attachObject(const std::string& object_id)
    {
        std::vector<std::string> touch_links = {
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger",
            "chessboard",
            "table_surface"
        };

        arm_group_.attachObject(object_id, "panda_hand", touch_links);
        RCLCPP_INFO(node_->get_logger(), "Attached '%s' to panda_hand in MoveIt.", object_id.c_str());
    }

    void detachObject(const std::string& object_id)
    {
        arm_group_.detachObject(object_id);
        RCLCPP_INFO(node_->get_logger(), "Detached '%s' from panda_hand in MoveIt.", object_id.c_str());
    }

    void addAllChessPieceCollisions(const std::string& json_file_path)
    {
        std::ifstream file(json_file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open piece JSON file: %s", json_file_path.c_str());
            return;
        }

        nlohmann::json board_poses;
        file >> board_poses;

        std::map<std::string, double> piece_heights = {
            {"pawn",   0.053},
            {"rook",   0.070},
            {"knight", 0.070},
            {"bishop", 0.070},
            {"king",   0.075},
            {"queen",  0.080}
        };

        std::map<std::string, std::string> initial_board_setup = {
            {"a8", "b_rook_1"},   {"b8", "b_knight_1"}, {"c8", "b_bishop_1"}, {"d8", "b_queen"},
            {"e8", "b_king"},     {"f8", "b_bishop_2"}, {"g8", "b_knight_2"}, {"h8", "b_rook_2"},
            {"a7", "b_pawn_1"},   {"b7", "b_pawn_2"},   {"c7", "b_pawn_3"},   {"d7", "b_pawn_4"},
            {"e7", "b_pawn_5"},   {"f7", "b_pawn_6"},   {"g7", "b_pawn_7"},   {"h7", "b_pawn_8"},

            {"a2", "w_pawn_1"},   {"b2", "w_pawn_2"},   {"c2", "w_pawn_3"},   {"d2", "w_pawn_4"},
            {"e2", "w_pawn_5"},   {"f2", "w_pawn_6"},   {"g2", "w_pawn_7"},   {"h2", "w_pawn_8"},
            {"a1", "w_rook_a1"},  {"b1", "w_knight_1"}, {"c1", "w_bishop_1"}, {"d1", "w_queen"},
            {"e1", "w_king"},     {"f1", "w_bishop_2"}, {"g1", "w_knight_2"}, {"h1", "w_rook_2"}
        };

        std::vector<moveit_msgs::msg::CollisionObject> piece_objects;
        const double surface_z = 1.033994;

        for (const auto& [square, piece_id] : initial_board_setup)
        {
            if (!board_poses.contains(square)) {
                RCLCPP_WARN(node_->get_logger(), "Square %s missing in JSON!", square.c_str());
                continue;
            }

            std::string piece_type = "pawn";
            for (const auto& [type, h] : piece_heights) {
                if (piece_id.find(type) != std::string::npos) {
                    piece_type = type;
                    break;
                }
            }
            double height = piece_heights[piece_type];

            double x = board_poses[square][0].get<double>();
            double y = board_poses[square][1].get<double>();

            moveit_msgs::msg::CollisionObject piece;
            piece.header.frame_id = "world";
            piece.id = piece_id;

            shape_msgs::msg::SolidPrimitive primitive;
            primitive.type = primitive.BOX;
            primitive.dimensions = {0.018, 0.018, height};

            geometry_msgs::msg::Pose piece_pose;
            piece_pose.orientation.w = 1.0;
            piece_pose.position.x = x;
            piece_pose.position.y = y;
            piece_pose.position.z = surface_z + (height / 2.0) + 0.02;

            piece.primitives.push_back(primitive);
            piece.primitive_poses.push_back(piece_pose);
            piece.operation = piece.ADD;

            piece_objects.push_back(piece);
        }

        planning_scene_interface_.applyCollisionObjects(piece_objects);
        RCLCPP_INFO(node_->get_logger(), "Added %zu piece collisions to Planning Scene!", piece_objects.size());
    }

    void setLinkAttacher(
        const std::string& model1, 
        const std::string& link1, 
        const std::string& model2, 
        const std::string& link2, 
        bool attach)
    {
        if (attach) {
            if (!attach_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(node_->get_logger(), "Service /ATTACHLINK not available!");
                return;
            }
            auto request = std::make_shared<linkattacher_msgs::srv::AttachLink::Request>();
            request->model1_name = model1;
            request->link1_name = link1;
            request->model2_name = model2;
            request->link2_name = link2;

            auto future = attach_client_->async_send_request(request);
            if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
                RCLCPP_INFO(node_->get_logger(), "Gazebo link attached successfully.");
            } else {
                RCLCPP_WARN(node_->get_logger(), "Gazebo attach request timed out or failed.");
            }
        } else {
            if (!detach_client_->wait_for_service(std::chrono::seconds(2))) {
                RCLCPP_ERROR(node_->get_logger(), "Service /DETACHLINK not available!");
                return;
            }
            auto request = std::make_shared<linkattacher_msgs::srv::DetachLink::Request>();
            request->model1_name = model1;
            request->link1_name = link1;
            request->model2_name = model2;
            request->link2_name = link2;

            auto future = detach_client_->async_send_request(request);
            if (future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
                RCLCPP_INFO(node_->get_logger(), "Gazebo link detached successfully.");
            } else {
                RCLCPP_WARN(node_->get_logger(), "Gazebo detach request timed out or failed.");
            }
        }
    }

    bool executePickAndPlace(
        const geometry_msgs::msg::Point& source,
        const geometry_msgs::msg::Point& target,
        const std::string& target_object_id = "",
        const std::shared_ptr<GoalHandlePickAndPlace>& goal_handle = nullptr,
        double clearance_offset = 0.25)
    {
        auto feedback = std::make_shared<PickAndPlace::Feedback>();

        auto publish_feedback = [&](const std::string& stage, float percentage) {
            if (goal_handle) {
                feedback->current_stage = stage;
                feedback->percentage_complete = percentage;
                goal_handle->publish_feedback(feedback);
            }
        };

        RCLCPP_INFO(node_->get_logger(), "Starting Pick & Place Sequence...");

        // 0. Open Gripper
        publish_feedback("Opening Gripper", 5.0);
        setGripperState("open");

        // 1. Move ABOVE Source
        publish_feedback("Moving Above Source", 15.0);
        geometry_msgs::msg::Pose source_above = createPose(source.x, source.y, source.z + clearance_offset);
        if (!moveToPose(source_above)) return false;

        // 2. Descend to Source
        publish_feedback("Descending to Pick", 30.0);
        geometry_msgs::msg::Pose source_pick = createPose(source.x, source.y, source.z);
        if (!executeCartesianPath({source_above, source_pick})) return false;

        // 3. Close Gripper & Attach Link
        publish_feedback("Attaching Object", 45.0);
        setGripperState("close");
        
        if (!target_object_id.empty()) {
            setLinkAttacher("cobot", "panda_leftfinger", target_object_id, "link", true);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        // 4. Lift Object
        publish_feedback("Lifting Piece", 55.0);
        if (!executeCartesianPath({source_pick, source_above})) return false;

        // 5. Move ABOVE Target
        publish_feedback("Transporting Overhead", 70.0);
        geometry_msgs::msg::Pose target_above = createPose(target.x, target.y, target.z + clearance_offset);
        if (!moveToPose(target_above)) return false;

        // 6. Descend to Target
        publish_feedback("Descending to Place", 80.0);
        geometry_msgs::msg::Pose target_place = createPose(target.x, target.y, target.z);
        if (!executeCartesianPath({target_above, target_place})) return false;

        // 7. Open Gripper & Detach Link
        publish_feedback("Detaching Object", 90.0);
        if (!target_object_id.empty()) {
            setLinkAttacher("cobot", "panda_leftfinger", target_object_id, "link", false);
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        setGripperState("open");
        

        }

        // 8. Retract Upward
        publish_feedback("Retracting", 95.0);
        if (!executeCartesianPath({target_place, target_above})) return false;

        // 9. Return to Home Position
        publish_feedback("Returning Ready", 100.0);
        if (!moveToNamedTarget("ready")) return false;

        RCLCPP_INFO(node_->get_logger(), "Pick and Place Sequence Completed Successfully!");
        return true;
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PickAndPlace::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(node_->get_logger(), "Received Request -> Source: %s | Target: %s | Piece: %s", 
                    goal->source_sqr.c_str(), goal->target_sqr.c_str(), goal->piece_id.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Goal Cancel Request Received");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        std::thread{std::bind(&PickAndPlaceExecutor::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePickAndPlace> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PickAndPlace::Result>();

        geometry_msgs::msg::Point source_pt;
        geometry_msgs::msg::Point target_pt;

        // Parse JSON coordinates using modular function
        if (!getPoseFromSquare(goal->source_sqr, source_pt) || 
            !getPoseFromSquare(goal->target_sqr, target_pt)) 
        {
            result->success = false;
            result->message = "Failed to resolve square coordinates from JSON.";
            goal_handle->abort(result);
            return;
        }

        bool success = executePickAndPlace(source_pt, target_pt, goal->piece_id, goal_handle);

        if (success) {
            result->success = true;
            result->message = "Execution completed successfully.";
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "Execution failed during trajectory planning/movement.";
            goal_handle->abort(result);
        }
    }

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
        arm_group_.setPoseTarget(target_pose);
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
        
        if (fraction >= 0.80) {
            arm_group_.execute(trajectory);
            return true;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Cartesian path planning incomplete (Only %.2f%% planned)", fraction * 100.0);
            return false;
        }
    }

    bool setGripperState(const std::string& state)
    {
        if (state == "open") {
            gripper_group_.setNamedTarget("open");
        } else {
            gripper_group_.setNamedTarget("close");
        }

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (gripper_group_.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (success) {
            gripper_group_.execute(plan);
        } else {
            RCLCPP_WARN(node_->get_logger(), "Gripper state planning skipped or already in target state.");
        }
        return true;
    }

    std::shared_ptr<rclcpp::Node> node_;
    MoveGroupInterface arm_group_;
    MoveGroupInterface gripper_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    geometry_msgs::msg::Quaternion default_orientation_;

    std::string json_path_;
    
    // Graveyard Location & Counter Tracking
    int captured_count_{0};
    const double graveyard_start_x_{0.209654+0.5};
    const double graveyard_y_{-0.288512};
    const double graveyard_x_step_{0.057965};

    rclcpp::Client<linkattacher_msgs::srv::AttachLink>::SharedPtr attach_client_;
    rclcpp::Client<linkattacher_msgs::srv::DetachLink>::SharedPtr detach_client_;
    rclcpp_action::Server<PickAndPlace>::SharedPtr action_server_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = std::make_shared<rclcpp::Node>("pick_and_place_server_node", options);

    // 1. Create multi-threaded executor and attach the node
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    // 2. Instantiate the Action Server
    PickAndPlaceExecutor server(node);

    // 3. Spin using the MultiThreadedExecutor directly on the main thread
    RCLCPP_INFO(node->get_logger(), "Spinning Pick & Place Server Node...");
    executor.spin();

    // 4. Clean shutdown when Ctrl+C is pressed
    rclcpp::shutdown();
    return 0;
}
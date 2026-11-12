#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>

#include "franka_description/action/pick_and_place.hpp"

using json = nlohmann::json;

class PickAndPlaceClient : public rclcpp::Node
{
public:
    using PickAndPlace = franka_description::action::PickAndPlace;
    using GoalHandlePickAndPlace = rclcpp_action::ClientGoalHandle<PickAndPlace>;

    explicit PickAndPlaceClient() : Node("pick_and_place_client")
    {
        client_ = rclcpp_action::create_client<PickAndPlace>(this, "pick_and_place");
        loadChessboardPoses("/home/adithya-si3510/arm_ws/src/moveit_config_pkg/config/chessboard_poses.json");
    }

    bool loadChessboardPoses(const std::string& json_path)
    {
        std::ifstream file(json_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open JSON file: %s", json_path.c_str());
            return false;
        }

        try {
            file >> json_data_;
            RCLCPP_INFO(this->get_logger(), "Successfully loaded chessboard poses from JSON.");
            return true;
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing JSON file: %s", e.what());
            return false;
        }
    }

    // Handles the JSON array format: "a2": [x, y, z]
    bool getSquarePose(const std::string& square, geometry_msgs::msg::Point& pt)
    {
        if (!json_data_.contains(square)) {
            RCLCPP_ERROR(this->get_logger(), "Square '%s' not found in chessboard_poses.json!", square.c_str());
            return false;
        }

        auto square_info = json_data_[square];

        if (square_info.is_array() && square_info.size() >= 2) {
            pt.x = square_info[0].get<double>();
            pt.y = square_info[1].get<double>();
            pt.z = 1.065; // Overriding JSON height with standard board surface Z
            return true;
        }

        RCLCPP_ERROR(this->get_logger(), "Square '%s' in JSON is not a valid array!", square.c_str());
        return false;
    }

    void sendGoal(const geometry_msgs::msg::Pose& source_pose, const geometry_msgs::msg::Pose& target_pose)
    {
        if (!client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Action server 'pick_and_place' not available!");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = PickAndPlace::Goal();
        goal_msg.source_pose = source_pose;
        goal_msg.target_pose = target_pose;

        RCLCPP_INFO(this->get_logger(), "Sending Goal:");
        RCLCPP_INFO(this->get_logger(), " -> Pick  : (X: %.3f, Y: %.3f, Z: %.3f)", 
                    source_pose.position.x, source_pose.position.y, source_pose.position.z);
        RCLCPP_INFO(this->get_logger(), " -> Target: (X: %.3f, Y: %.3f, Z: %.3f)", 
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);

        auto send_goal_options = rclcpp_action::Client<PickAndPlace>::SendGoalOptions();
        
        send_goal_options.feedback_callback =
            [this](GoalHandlePickAndPlace::SharedPtr, const std::shared_ptr<const PickAndPlace::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "[Feedback %.0f%%] %s",
                            feedback->percentage_complete, feedback->current_stage.c_str());
            };

        send_goal_options.result_callback =
            [this](const GoalHandlePickAndPlace::WrappedResult& result) {
                switch (result.code) {
                    case rclcpp_action::ResultCode::SUCCEEDED:
                        RCLCPP_INFO(this->get_logger(), "Goal Succeeded! Message: %s", result.result->message.c_str());
                        break;
                    case rclcpp_action::ResultCode::ABORTED:
                        RCLCPP_ERROR(this->get_logger(), "Goal Aborted! Message: %s", result.result->message.c_str());
                        break;
                    case rclcpp_action::ResultCode::CANCELED:
                        RCLCPP_WARN(this->get_logger(), "Goal Canceled!");
                        break;
                    default:
                        RCLCPP_ERROR(this->get_logger(), "Unknown result code!");
                        break;
                }
                rclcpp::shutdown();
            };

        client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<PickAndPlace>::SharedPtr client_;
    json json_data_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<PickAndPlaceClient>();

    if (argc != 3) {
        std::cout << "\n--- Chessboard Pick and Place Command ---\n";
        std::cout << "Usage: ros2 run moveit_config_pkg pick_and_place_client <source_square> <target_square>\n";
        std::cout << "Example: ros2 run moveit_config_pkg pick_and_place_client a2 e4\n\n";
        return 1;
    }

    std::string source_sq = argv[1];
    std::string target_sq = argv[2];

    geometry_msgs::msg::Pose source_pose;
    geometry_msgs::msg::Pose target_pose;

    if (!client_node->getSquarePose(source_sq, source_pose.position) ||
        !client_node->getSquarePose(target_sq, target_pose.position)) {
        RCLCPP_ERROR(client_node->get_logger(), "Failed to look up square positions from JSON.");
        return 1;
    }

    client_node->sendGoal(source_pose, target_pose);
    rclcpp::spin(client_node);
    return 0;
}
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Action Header
#include "franka_description/action/pick_and_place.hpp"

class PickAndPlaceClient : public rclcpp::Node
{
public:
    using PickAndPlace = franka_description::action::PickAndPlace;
    using GoalHandlePickAndPlace = rclcpp_action::ClientGoalHandle<PickAndPlace>;

    explicit PickAndPlaceClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("pick_and_place_client_node", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<PickAndPlace>(
            this,
            "pick_and_place");
    }

    void sendGoal(const std::string& source_sqr, const std::string& target_sqr, const std::string& piece_id)
    {
        using namespace std::chrono_literals;

        if (!this->client_ptr_->wait_for_action_server(10s)) {
            RCLCPP_ERROR(this->get_logger(), "Action server '/pick_and_place' not available after waiting.");
            return;
        }

        auto goal_msg = PickAndPlace::Goal();
        goal_msg.source_sqr = source_sqr;
        goal_msg.target_sqr = target_sqr;
        goal_msg.piece_id   = piece_id;

        RCLCPP_INFO(this->get_logger(), "Sending goal -> Source: '%s' | Target: '%s' | Piece: '%s'",
                    source_sqr.c_str(), target_sqr.c_str(), piece_id.c_str());

        auto send_goal_options = rclcpp_action::Client<PickAndPlace>::SendGoalOptions();
        
        // 1. Goal Response Callback
        send_goal_options.goal_response_callback =
            std::bind(&PickAndPlaceClient::goalResponseCallback, this, std::placeholders::_1);

        // 2. Feedback Callback
        send_goal_options.feedback_callback =
            std::bind(&PickAndPlaceClient::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);

        // 3. Final Result Callback
        send_goal_options.result_callback =
            std::bind(&PickAndPlaceClient::resultCallback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<PickAndPlace>::SharedPtr client_ptr_;

    void goalResponseCallback(const GoalHandlePickAndPlace::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by action server!");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, execution started.");
        }
    }

    void feedbackCallback(
        GoalHandlePickAndPlace::SharedPtr,
        const std::shared_ptr<const PickAndPlace::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Feedback -> Stage: '%s' | Progress: %.1f%%",
                    feedback->current_stage.c_str(),
                    feedback->percentage_complete);
    }

    void resultCallback(const GoalHandlePickAndPlace::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal Succeeded! Message: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted! Message: %s", result.result->message.c_str());
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "Goal was canceled!");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code!");
                break;
        }
        
        // Shutdown the node once execution finishes
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto client_node = std::make_shared<PickAndPlaceClient>();

    // Parse command line args if provided, otherwise default to e2 -> e4 move
    std::string source_sqr = "e2";
    std::string target_sqr = "e4";
    std::string piece_id   = "w_pawn_5";

    if (argc >= 4) {
        source_sqr = argv[1];
        target_sqr = argv[2];
        piece_id   = argv[3];
    } else {
        RCLCPP_INFO(client_node->get_logger(), 
                    "No arguments provided. Using defaults: %s %s %s", 
                    source_sqr.c_str(), target_sqr.c_str(), piece_id.c_str());
        RCLCPP_INFO(client_node->get_logger(), 
                    "Usage: ros2 run <your_pkg> pick_and_place_client <source_sqr> <target_sqr> <piece_id>");
    }

    client_node->sendGoal(source_sqr, target_sqr, piece_id);

    rclcpp::spin(client_node);
    return 0;
}
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_kdl_package/action/follow_trajectory.hpp"

using namespace std::chrono_literals;
using FollowTrajectory = ros2_kdl_package::action::FollowTrajectory;
using GoalHandleFollowTrajectory = rclcpp_action::ClientGoalHandle<FollowTrajectory>;

/*
 * Punto 1c: Action client di test (commenti in italiano)
 * - Si connette all'action server follow_trajectory
 * - Invia un goal di esempio e stampa feedback e result.
 */
class TrajectoryActionClient : public rclcpp::Node
{
public:
    TrajectoryActionClient() : Node("trajectory_action_client")
    {
        this->client_ = rclcpp_action::create_client<FollowTrajectory>(this, "follow_trajectory");
        
        // Attesa del server
        while (!this->client_->wait_for_action_server(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }
        
        this->send_goal();
    }

private:
    rclcpp_action::Client<FollowTrajectory>::SharedPtr client_;

    void send_goal()
    {
        auto goal_msg = FollowTrajectory::Goal();
        goal_msg.traj_duration = 1.5;
        goal_msg.acc_duration = 0.5;

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<FollowTrajectory>::SendGoalOptions();
        
        send_goal_options.goal_response_callback =
            std::bind(&TrajectoryActionClient::goal_response_callback, this, std::placeholders::_1);
        
        send_goal_options.feedback_callback =
            std::bind(&TrajectoryActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        
        send_goal_options.result_callback =
            std::bind(&TrajectoryActionClient::result_callback, this, std::placeholders::_1);

        this->client_->async_send_goal(goal_msg, send_goal_options);
    }

    void goal_response_callback(const GoalHandleFollowTrajectory::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleFollowTrajectory::SharedPtr,
        const std::shared_ptr<const FollowTrajectory::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Current position error: %f", feedback->current_error);
    }

    void result_callback(const GoalHandleFollowTrajectory::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded! Final error: %f", result.result->final_error);
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
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "ros2_kdl_package/action/follow_trajectory.hpp"
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"

using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
using FollowTrajectory = ros2_kdl_package::action::FollowTrajectory;
using GoalHandleFollowTrajectory = rclcpp_action::ServerGoalHandle<FollowTrajectory>;

/*
 * Punto 1c: Action server "follow_trajectory"
 * - Esegue una traiettoria lineare con profilo trapezoidale.
 * - Pubblica feedback con l'errore di posizione corrente.
 * - Restituisce il valore dell'errore finale nel result.
 */
class TrajectoryActionServer : public rclcpp::Node
{
public:
    TrajectoryActionServer()
    : Node("trajectory_action_server"),
      node_handle_(std::shared_ptr<TrajectoryActionServer>(this))
    {
        using namespace std::placeholders;

        // Action server
        this->action_server_ = rclcpp_action::create_server<FollowTrajectory>(
            this,
            "follow_trajectory",
            std::bind(&TrajectoryActionServer::handle_goal, this, _1, _2),
            std::bind(&TrajectoryActionServer::handle_cancel, this, _1),
            std::bind(&TrajectoryActionServer::handle_accepted, this, _1));

        // Publisher per comandi di giunto (modalità posizione)
        cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);

        // Sottoscrizione a /joint_states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&TrajectoryActionServer::joint_state_subscriber, this, _1));

        // Attesa del primo messaggio di stato giunti
        while (!joint_state_available_) {
            RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
            rclcpp::spin_some(node_handle_);
            std::this_thread::sleep_for(100ms);
        }

        // Inizializzazione modello KDL da robot_description
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for robot_state_publisher...");
        }

        auto parameter = parameters_client->get_parameters({"robot_description"});
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to retrieve robot_description!");
            return;
        }

        robot_ = std::make_shared<KDLRobot>(robot_tree);
        unsigned int nj = robot_->getNrJnts();

        // Limiti di giunto (valori indicativi)
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
        q_max.data <<  2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96;
        robot_->setJntLimits(q_min, q_max);

        // Buffer
        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);

        // EE e stato iniziale
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        init_cart_pose_ = robot_->getEEFrame();

        RCLCPP_INFO(this->get_logger(), "Action server ready!");
    }

private:
    rclcpp_action::Server<FollowTrajectory>::SharedPtr action_server_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;

    std::shared_ptr<KDLRobot> robot_;
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    KDL::JntArray joint_positions_cmd_;
    KDL::Frame init_cart_pose_;
    bool joint_state_available_ = false;
    rclcpp::Node::SharedPtr node_handle_;

    // Callback /joint_states (robusta)
    void joint_state_subscriber(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.empty()) {
            return;
        }
        const std::size_t n = msg->position.size();
        if (static_cast<std::size_t>(joint_positions_.data.size()) != n) {
            joint_positions_.resize(n);
        }
        if (static_cast<std::size_t>(joint_velocities_.data.size()) != n) {
            joint_velocities_.resize(n);
        }
        for (std::size_t i = 0; i < n; ++i) {
            joint_positions_.data[static_cast<long>(i)] = msg->position[i];
            double v = (msg->velocity.size() == n) ? msg->velocity[i] : 0.0;
            joint_velocities_.data[static_cast<long>(i)] = v;
        }
        joint_state_available_ = true;
    }

    // Gestione goal/cancel
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const FollowTrajectory::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with traj_duration: %f", goal->traj_duration);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        // Esecuzione su thread dedicato
        std::thread{std::bind(&TrajectoryActionServer::execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    // Esecuzione del goal: pianifica e invia comandi in posizione
    void execute(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowTrajectory::Feedback>();
        auto result = std::make_shared<FollowTrajectory::Result>();

        // Traiettoria lineare da posa iniziale (offset su Z) a posa con Y specchiata
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0, 0, 0.1));
        Eigen::Vector3d end_position;
        end_position << init_position[0], -init_position[1], init_position[2];
        KDLPlanner planner(goal->traj_duration, goal->acc_duration, init_position, end_position);

        // Timing
        double t = 0.0;
        int trajectory_len = 150;
        double total_time = goal->traj_duration;
        double loop_rate = static_cast<double>(trajectory_len) / total_time;
        double dt = 1.0 / loop_rate;
        int Kp = 5;
        rclcpp::Rate rate(loop_rate);

        while (t < total_time && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->final_error = 0.0;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Campionamento traiettoria
            trajectory_point p = planner.linear_traj_trapezoidal(t);

            // Stato robot ed errore cartesiano
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame cartpos = robot_->getEEFrame();
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));

            // Feedback
            feedback->current_error = error.norm();
            goal_handle->publish_feedback(feedback);

            // Passo successivo in posizione (integrazione esplicita)
            KDL::Frame nextFrame;
            nextFrame.M = cartpos.M;
            nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(Kp * error)) * dt;

            // IK e pubblicazione comandi
            joint_positions_cmd_ = joint_positions_;
            robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.resize(joint_positions_cmd_.data.size());
            for (long int i = 0; i < joint_positions_cmd_.data.size(); ++i) {
                cmd_msg.data[i] = joint_positions_cmd_(i);
            }
            cmdPublisher_->publish(cmd_msg);

            t += dt;
            rate.sleep();
        }

        // Result finale
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame final_cartpos = robot_->getEEFrame();
        Eigen::Vector3d final_error = computeLinearError(end_position, Eigen::Vector3d(final_cartpos.p.data));
        result->final_error = final_error.norm();

        if (rclcpp::ok()) {
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded with final error: %f", result->final_error);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

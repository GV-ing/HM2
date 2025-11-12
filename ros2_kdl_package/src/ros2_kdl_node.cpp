// ROS 2 KDL Node - Homework 1 & 2
// Punto 1a: Parametri ROS 2 per pianificazione traiettoria
// Punto 1b: Controllo in velocità con null-space optimization
// Punto 1c: Action server integrato per esecuzione traiettoria
// Punto 2b: Controllo visivo basato su ArUco marker
 
#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>
#include <array>
#include <algorithm>

#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Action type (FollowTrajectory)
#include "ros2_kdl_package/action/follow_trajectory.hpp"

#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

using FollowTrajectory = ros2_kdl_package::action::FollowTrajectory; // azione integrata (Punto 1c)
using GoalHandleFollowTrajectory = rclcpp_action::ServerGoalHandle<FollowTrajectory>;

class Iiwa_pub_sub : public rclcpp::Node
{
public:
    Iiwa_pub_sub()
    : Node("ros2_kdl_node"),
      node_handle_(std::shared_ptr<Iiwa_pub_sub>(this))
    {
        // Parametri di base
        declare_parameter("cmd_interface", "position");
        get_parameter("cmd_interface", cmd_interface_);
        RCLCPP_INFO(get_logger(), "Current cmd interface is: '%s'", cmd_interface_.c_str());
        if (!(cmd_interface_ == "position" || cmd_interface_ == "velocity" || cmd_interface_ == "effort")) {
            RCLCPP_ERROR(get_logger(), "Selected cmd interface is not valid!");
            return;
        }

        declare_parameter("traj_type", "linear");
        get_parameter("traj_type", traj_type_);
        RCLCPP_INFO(get_logger(), "Current trajectory type is: '%s'", traj_type_.c_str());

        declare_parameter("s_type", "trapezoidal");
        get_parameter("s_type", s_type_);
        RCLCPP_INFO(get_logger(), "Current s type is: '%s'", s_type_.c_str());

        declare_parameter("ctrl", "velocity_ctrl");
        get_parameter("ctrl", ctrl_type_);
        RCLCPP_INFO(get_logger(), "Current velocity controller type is: '%s'", ctrl_type_.c_str());
        const std::array<std::string, 3> valid_ctrls{"velocity_ctrl", "velocity_ctrl_null", "vision"};
        if (std::find(valid_ctrls.begin(), valid_ctrls.end(), ctrl_type_) == valid_ctrls.end()) {
            RCLCPP_ERROR(get_logger(), "Selected velocity controller is not valid!");
            return;
        }

        declare_parameter("lambda", 1.0);
        get_parameter("lambda", lambda_);

        if (ctrl_type_ == "vision" && cmd_interface_ != "velocity") {
            RCLCPP_WARN(get_logger(), "Vision controller requires 'velocity' cmd_interface. Forcing velocity mode.");
            cmd_interface_ = "velocity";
            this->set_parameter(rclcpp::Parameter("cmd_interface", cmd_interface_));
        }

        declare_parameter("aruco_topic", aruco_topic_);
        get_parameter("aruco_topic", aruco_topic_);

        declare_parameter("vision_gain", std::vector<double>{1.0, 1.0, 1.0});
        std::vector<double> gain_vec;
        get_parameter("vision_gain", gain_vec);
        if (gain_vec.size() == 3) {
            vision_gain_ << gain_vec[0], gain_vec[1], gain_vec[2];
        } else {
            vision_gain_ = Eigen::Vector3d::Ones();
        }

        declare_parameter("vision_velocity_limit", 0.0);
        get_parameter("vision_velocity_limit", vision_velocity_limit_);
        vision_velocity_limit_ = std::max(0.0, vision_velocity_limit_);

        // Con il frame ottico, l'offset è gestito da TF
        // Manteniamo il parametro per compatibilità, ma lo impostiamo a zero
        declare_parameter("camera_offset", std::vector<double>{0.0, 0.0, 0.0});
        std::vector<double> camera_offset_vec;
        get_parameter("camera_offset", camera_offset_vec);
        if (camera_offset_vec.size() == 3) {
            camera_offset_ << camera_offset_vec[0], camera_offset_vec[1], camera_offset_vec[2];
        } else {
            camera_offset_ << 0.0, 0.0, 0.0;  // Zero offset con frame ottico
        }

        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;

        // Parametri di traiettoria
        declare_parameter("traj_duration", 1.5);
        declare_parameter("acc_duration", 0.5);
        declare_parameter("total_time", 1.5);
        declare_parameter("trajectory_len", 150);
        declare_parameter("Kp", 5.0);
        declare_parameter("end_position", std::vector<double>{0.0, 0.0, 0.0});

        get_parameter("traj_duration", traj_duration_);
        get_parameter("acc_duration", acc_duration_);
        get_parameter("total_time", total_time_);
        get_parameter("trajectory_len", trajectory_len_);
        get_parameter("Kp", Kp_);

        // Attendi robot_description da robot_state_publisher
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
        while (!parameters_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                rclcpp::shutdown();
            }
            RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
        }
        auto parameter = parameters_client->get_parameters({"robot_description"});

        // Crea modello KDL
        KDL::Tree robot_tree;
        if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)) {
            std::cout << "Failed to retrieve robot_description param!";
        }
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        // Limiti giunti (TODO: leggere da URDF)
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96, -2.09, -2.96, -2.09, -2.96, -2.09, -2.96;
        q_max.data <<  2.96,  2.09,  2.96,  2.09,  2.96,  2.09,  2.96;
        robot_->setJntLimits(q_min, q_max);
        joint_positions_.resize(nj);
        joint_velocities_.resize(nj);
        joint_positions_cmd_.resize(nj);
        joint_velocities_cmd_.resize(nj);
        joint_efforts_cmd_.resize(nj);
        joint_efforts_cmd_.data.setZero();

        // Sottoscrizione a /joint_states
        jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Iiwa_pub_sub::joint_state_subscriber, this, std::placeholders::_1));

        if (ctrl_type_ == "vision") {
            arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                aruco_topic_, 10, std::bind(&Iiwa_pub_sub::aruco_pose_subscriber, this, std::placeholders::_1));
            RCLCPP_INFO(get_logger(), "Vision controller waiting for ArUco pose on topic '%s'", aruco_topic_.c_str());
        }

        // Attendi il primo messaggio
        while (!joint_state_available_) {
            RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
            rclcpp::spin_some(node_handle_);
        }

        // Inizializzazione KDLRobot
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        init_cart_pose_ = robot_->getEEFrame();

        // Validazione IK iniziale
        KDL::JntArray q(nj);
        robot_->getInverseKinematics(init_cart_pose_, q);

        controller_ = std::make_shared<KDLController>(*robot_);

        // Pianificazione traiettoria
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0, 0, 0.1));
        Eigen::Vector3d end_position;
        std::vector<double> end_pos_param;
        get_parameter("end_position", end_pos_param);
        if (end_pos_param.size() == 3 && !(end_pos_param[0] == 0.0 && end_pos_param[1] == 0.0 && end_pos_param[2] == 0.0)) {
            end_position << end_pos_param[0], end_pos_param[1], end_pos_param[2];
        } else {
            end_position << init_position[0], -init_position[1], init_position[2];
        }

        double traj_radius = 0.15;
        if (traj_type_ == "linear") {
            planner_ = KDLPlanner(traj_duration_, acc_duration_, init_position, end_position);
            if (s_type_ == "trapezoidal") {
                p_ = planner_.linear_traj_trapezoidal(t_);
            } else if (s_type_ == "cubic") {
                p_ = planner_.linear_traj_cubic(t_);
            }
        } else if (traj_type_ == "circular") {
            planner_ = KDLPlanner(traj_duration_, init_position, traj_radius, acc_duration_);
            if (s_type_ == "trapezoidal") {
                p_ = planner_.circular_traj_trapezoidal(t_);
            } else if (s_type_ == "cubic") {
                p_ = planner_.circular_traj_cubic(t_);
            }
        }

        // Publisher comandi e timer
        posCmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);

        if (cmd_interface_ == "position") {
            cmdPublisher_ = posCmdPublisher_;
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                desired_commands_[i] = joint_positions_(i);
            }
        } else if (cmd_interface_ == "velocity") {
            cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_(i);
            }
        } else if (cmd_interface_ == "effort") {
            cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
            timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Iiwa_pub_sub::cmd_publisher, this));
            for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                desired_commands_[i] = joint_efforts_cmd_(i);
            }
        }

        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);

        // Action server integrato (Punto 1c)
        declare_parameter("enable_action_server", false);
        bool enable_action = false;
        get_parameter("enable_action_server", enable_action);
        if (enable_action) {
            create_action_server();
            RCLCPP_INFO(this->get_logger(), "Action server abilitato e creato");
        } else {
            RCLCPP_INFO(this->get_logger(), "Action server disabilitato - usare parametro 'enable_action_server:=true' per abilitare");
        }

        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution (timer loop) ...");
    }

private:
    void cmd_publisher()
    {
        iteration_ = iteration_ + 1;
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

        // Se è attiva un'azione, esegui traiettoria dell'azione
        if (action_active_) {
            execute_action_cycle();
            return;
        }

        if (ctrl_type_ == "vision") {
            if (!aruco_pose_available_) {
                joint_velocities_cmd_.data.setZero();
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for ArUco pose..." );
            } else {
                KDL::Frame cartpos = robot_->getEEFrame();
                Eigen::Matrix3d R_base_camera = toEigen(cartpos.M);
                Eigen::Matrix<double, 6, Eigen::Dynamic> J_tool = robot_->getEEJacobian().data;
                Eigen::Vector3d p_tc_base = R_base_camera * camera_offset_;
                Eigen::Matrix3d skew_p = skew(p_tc_base);
                Eigen::Matrix<double, 6, Eigen::Dynamic> J_camera = J_tool;
                J_camera.block(0, 0, 3, J_tool.cols()) = J_tool.block(0, 0, 3, J_tool.cols()) - skew_p * J_tool.block(3, 0, 3, J_tool.cols());
                joint_velocities_cmd_.data = controller_->vision_ctrl(
                    aruco_position_cam_,
                    J_camera,
                    R_base_camera,
                    vision_gain_,
                    lambda_
                );
            }

            if (vision_velocity_limit_ > 0.0) {
                for (long int i = 0; i < joint_velocities_cmd_.data.size(); ++i) {
                    joint_velocities_cmd_(i) = std::clamp(
                        joint_velocities_cmd_(i),
                        -vision_velocity_limit_,
                        vision_velocity_limit_);
                }
            }

            for (long int i = 0; i < joint_velocities_cmd_.data.size(); ++i) {
                desired_commands_[i] = joint_velocities_cmd_(i);
            }

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
            return;
        }

        // Traiettoria temporizzata
        double loop_rate = static_cast<double>(trajectory_len_) / total_time_;
        double dt = 1.0 / loop_rate;
        t_ += dt;

        if (t_ < total_time_) {
            if (traj_type_ == "linear") {
                if (s_type_ == "trapezoidal") {
                    p_ = planner_.linear_traj_trapezoidal(t_);
                } else if (s_type_ == "cubic") {
                    p_ = planner_.linear_traj_cubic(t_);
                }
            } else if (traj_type_ == "circular") {
                if (s_type_ == "trapezoidal") {
                    p_ = planner_.circular_traj_trapezoidal(t_);
                } else if (s_type_ == "cubic") {
                    p_ = planner_.circular_traj_cubic(t_);
                }
            }

            KDL::Frame cartpos = robot_->getEEFrame();
            Eigen::Vector3d error = computeLinearError(p_.pos, Eigen::Vector3d(cartpos.p.data));
            Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));

            if (cmd_interface_ == "position") {
                KDL::Frame nextFrame;
                nextFrame.M = cartpos.M;
                nextFrame.p = cartpos.p + (toKDL(p_.vel) + toKDL(Kp_ * error)) * dt;
                joint_positions_cmd_ = joint_positions_;
                robot_->getInverseKinematics(nextFrame, joint_positions_cmd_);
            } else if (cmd_interface_ == "velocity") {
                if (ctrl_type_ == "velocity_ctrl") {
                    Vector6d cartvel;
                    cartvel << p_.vel + Kp_ * error, o_error;
                    joint_velocities_cmd_.data = pseudoinverse(robot_->getEEJacobian().data) * cartvel;
                } else if (ctrl_type_ == "velocity_ctrl_null") {
                    joint_velocities_cmd_.data = controller_->velocity_ctrl_null(
                        p_.pos,
                        Eigen::Vector3d(cartpos.p.data),
                        lambda_,
                        Kp_
                    );
                }
            } else if (cmd_interface_ == "effort") {
                joint_efforts_cmd_.data[0] = 0.1 * std::sin(2 * M_PI * t_ / total_time_);
            }

            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));

            if (cmd_interface_ == "position") {
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            } else if (cmd_interface_ == "velocity") {
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_cmd_(i);
                }
            } else if (cmd_interface_ == "effort") {
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");

            if (cmd_interface_ == "position") {
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_cmd_(i);
                }
            } else if (cmd_interface_ == "velocity") {
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }
            } else if (cmd_interface_ == "effort") {
                for (long int i = 0; i < joint_efforts_cmd_.data.size(); ++i) {
                    desired_commands_[i] = joint_efforts_cmd_(i);
                }
            }

            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
        }
    }
    
    void create_action_server() {   
        using namespace std::placeholders;
        action_server_ = rclcpp_action::create_server<FollowTrajectory>(
            this,
            "follow_trajectory",
            std::bind(&Iiwa_pub_sub::handle_goal, this, _1, _2),
            std::bind(&Iiwa_pub_sub::handle_cancel, this, _1),
            std::bind(&Iiwa_pub_sub::handle_accepted, this, _1)
        );
    }

    void joint_state_subscriber(const sensor_msgs::msg::JointState & sensor_msg)
    {
        if (sensor_msg.position.empty()) {
            return;
        }
        const std::size_t n = sensor_msg.position.size();
        if (static_cast<std::size_t>(joint_positions_.data.size()) != n) {
            joint_positions_.resize(n);
        }
        if (static_cast<std::size_t>(joint_velocities_.data.size()) != n) {
            joint_velocities_.resize(n);
        }
        for (std::size_t i = 0; i < n; ++i) {
            joint_positions_.data[static_cast<long>(i)] = sensor_msg.position[i];
            double v = (sensor_msg.velocity.size() == n) ? sensor_msg.velocity[i] : 0.0;
            joint_velocities_.data[static_cast<long>(i)] = v;
        }
        joint_state_available_ = true;
    }

    void aruco_pose_subscriber(const geometry_msgs::msg::PoseStamped & msg)
    {
        aruco_position_cam_ << msg.pose.position.x, msg.pose.position.y, msg.pose.position.z;
        aruco_pose_available_ = true;
    }

    // Gestione Action: goal/cancel/accepted
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                                            std::shared_ptr<const FollowTrajectory::Goal> goal) {
        RCLCPP_INFO(get_logger(), "[Action] Goal request traj_duration=%.3f acc_duration=%.3f", goal->traj_duration, goal->acc_duration);
        (void)uuid;
        if (action_active_) {
            RCLCPP_WARN(get_logger(), "[Action] Un goal è già attivo: rifiutato");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (goal->traj_duration <= 0.0 || goal->acc_duration < 0.0 || goal->acc_duration > goal->traj_duration) {
            RCLCPP_ERROR(get_logger(), "[Action] Parametri non validi");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle) {
        RCLCPP_INFO(get_logger(), "[Action] Richiesta di cancel ricevuta");
        (void)goal_handle;
        action_active_ = false;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleFollowTrajectory> goal_handle) {
        RCLCPP_INFO(get_logger(), "[Action] Goal accettato: avvio esecuzione integrata nel timer");
        active_goal_handle_ = goal_handle;
        const auto goal = goal_handle->get_goal();
        robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
        action_init_cart_pose_ = robot_->getEEFrame();
        Eigen::Vector3d init_position(Eigen::Vector3d(action_init_cart_pose_.p.data) - Eigen::Vector3d(0, 0, 0.1));
        Eigen::Vector3d end_position(init_position[0], -init_position[1], init_position[2]);
        action_traj_duration_ = goal->traj_duration;
        action_acc_duration_ = goal->acc_duration;
        action_trajectory_len_ = 150;
        action_planner_ = KDLPlanner(action_traj_duration_, action_acc_duration_, init_position, end_position);
        action_joint_positions_cmd_.resize(robot_->getNrJnts());
        action_t_ = 0.0;
        action_active_ = true;
    }

    void execute_action_cycle() {
        if (!active_goal_handle_ || !action_active_) return;

        double loop_rate = static_cast<double>(action_trajectory_len_) / action_traj_duration_;
        double dt = 1.0 / loop_rate;
        if (action_t_ < action_traj_duration_) {
            action_p_ = action_planner_.linear_traj_trapezoidal(action_t_);
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame cartpos = robot_->getEEFrame();
            Eigen::Vector3d error = computeLinearError(action_p_.pos, Eigen::Vector3d(cartpos.p.data));

            auto feedback = std::make_shared<FollowTrajectory::Feedback>();
            feedback->current_error = error.norm();
            active_goal_handle_->publish_feedback(feedback);

            KDL::Frame nextFrame;
            nextFrame.M = cartpos.M;
            double Kp_action = Kp_;
            nextFrame.p = cartpos.p + (toKDL(action_p_.vel) + toKDL(Kp_action * error)) * dt;

            action_joint_positions_cmd_ = joint_positions_;
            robot_->getInverseKinematics(nextFrame, action_joint_positions_cmd_);
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data.resize(action_joint_positions_cmd_.data.size());
            for (long int i = 0; i < action_joint_positions_cmd_.data.size(); ++i) {
                cmd_msg.data[i] = action_joint_positions_cmd_(i);
            }
            posCmdPublisher_->publish(cmd_msg);
            action_t_ += dt;
        } else {
            robot_->update(toStdVector(joint_positions_.data), toStdVector(joint_velocities_.data));
            KDL::Frame cartpos = robot_->getEEFrame();
            Eigen::Vector3d final_error = computeLinearError(action_p_.pos, Eigen::Vector3d(cartpos.p.data));
            auto result = std::make_shared<FollowTrajectory::Result>();
            result->final_error = final_error.norm();
            if (active_goal_handle_->is_active()) {
                active_goal_handle_->succeed(result);
                RCLCPP_INFO(get_logger(), "[Action] Goal completato con errore finale=%.6f", result->final_error);
            }
            action_active_ = false;
            active_goal_handle_.reset();
        }
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::Publisher<FloatArray>::SharedPtr posCmdPublisher_; // dedicato all'action (position controller)
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr subTimer_;
    rclcpp::Node::SharedPtr node_handle_;

    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;

    KDL::JntArray joint_positions_cmd_;
    KDL::JntArray joint_velocities_cmd_;
    KDL::JntArray joint_efforts_cmd_;

    std::shared_ptr<KDLController> controller_;

    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;

    trajectory_point p_;

    int iteration_;
    bool joint_state_available_;
    double t_;

    // Punto 1a: parametri caricati da ROS 2
    double traj_duration_;
    double acc_duration_;
    double total_time_;
    int trajectory_len_;
    double Kp_;

    std::string cmd_interface_;
    std::string traj_type_;
    std::string s_type_;
    std::string ctrl_type_;
    double lambda_; // Punto 1b: scala ottimizzazione spazio nullo

    KDL::Frame init_cart_pose_;

    // Punto 2b: stato del controller visivo
    bool aruco_pose_available_{false};
    Eigen::Vector3d aruco_position_cam_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d vision_gain_{Eigen::Vector3d::Ones()};
    double vision_velocity_limit_{0.0};
    std::string aruco_topic_{"/aruco_single/pose"};
    Eigen::Vector3d camera_offset_{0.0, 0.0, 0.12};

    // Stato Action integrata (Punto 1c)
    rclcpp_action::Server<FollowTrajectory>::SharedPtr action_server_;
    bool action_active_{false};
    double action_t_{0.0};
    double action_traj_duration_{0.0};
    double action_acc_duration_{0.0};
    int action_trajectory_len_{150};
    KDLPlanner action_planner_;
    trajectory_point action_p_;
    KDL::JntArray action_joint_positions_cmd_;
    KDL::Frame action_init_cart_pose_;
    std::shared_ptr<GoalHandleFollowTrajectory> active_goal_handle_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Iiwa_pub_sub>());
    rclcpp::shutdown();
    return 1;
}

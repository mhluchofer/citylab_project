// ============================ HEADER ============================
#include <memory>
#include <cmath>
#include <chrono>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "robot_interfaces/action/go_to_pose.hpp"

using namespace std::chrono_literals;

class GoToPoseAction : public rclcpp::Node
{
public:
    using GoToPose = robot_interfaces::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;

    GoToPoseAction() : Node("go_to_pose_action")
    {
        // Publisher para mover el robot
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Suscripción a odometría
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&GoToPoseAction::odom_callback, this, std::placeholders::_1));

        // Servidor de acción
        action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "/go_to_pose",
            std::bind(&GoToPoseAction::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GoToPoseAction::handle_cancel, this, std::placeholders::_1),
            std::bind(&GoToPoseAction::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "GoToPose Action Server Ready");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

    geometry_msgs::msg::Pose2D current_pos_;
    geometry_msgs::msg::Pose2D desired_pos_;

    // ============================ CALLBACKS ============================
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;

        // Convertir quaternion a yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_pos_.theta = yaw; // Internamente en radianes
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const GoToPose::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received new goal: x=%.2f, y=%.2f, theta=%.2f (deg)",
                    goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);

        desired_pos_ = goal->goal_pos;
        // Convertir theta de grados a radianes para cálculos internos
        desired_pos_.theta = goal->goal_pos.theta * M_PI / 180.0;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        RCLCPP_WARN(this->get_logger(), "Goal canceled");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        std::thread{std::bind(&GoToPoseAction::execute, this, goal_handle)}.detach();
    }

    // ============================ CONTROL LOOP ============================
    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle)
    {
        rclcpp::Rate loop_rate(10); // 10 Hz
        const double max_linear_speed = 0.1; // m/s
        const double max_angular_speed = 1.0; // rad/s
        const double angular_kp = 1.65;   // Ganancia proporcional
        const double distance_tolerance = 0.05; //  5 cm
        const double angle_tolerance = 0.05;    // ~2.8 deg

        auto result = std::make_shared<GoToPose::Result>();

        while (rclcpp::ok())
        {
            // Cancelación
            if (goal_handle->is_canceling())
            {
                cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); // detener
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }

            // Diferencias
            double dx = desired_pos_.x - current_pos_.x;
            double dy = desired_pos_.y - current_pos_.y;
            double distance = std::sqrt(dx * dx + dy * dy);
            double angle_to_goal = std::atan2(dy, dx);
            double angle_diff = angle_to_goal - current_pos_.theta;

            // Normalizar ángulo a [-pi, pi]
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
            while (angle_diff < -M_PI) angle_diff += 2 * M_PI;

            geometry_msgs::msg::Twist cmd;

            if (distance > distance_tolerance)
            {
                // Factor de desaceleración basado en la distancia
                double distance_factor = std::clamp(distance / 0.2, 0.0, 1.0);

                cmd.linear.x = max_linear_speed * distance_factor;

                // Control proporcional angular con suavizado por distancia
                double angular_speed = angular_kp * angle_diff * distance_factor;
                cmd.angular.z = std::clamp(angular_speed, -max_angular_speed, max_angular_speed);
            }
            else
            {
                // Ajustar orientación final
                double angle_error = desired_pos_.theta - current_pos_.theta;
                while (angle_error > M_PI) angle_error -= 2 * M_PI;
                while (angle_error < -M_PI) angle_error += 2 * M_PI;

                if (std::fabs(angle_error) > angle_tolerance)
                {
                    cmd.angular.z = std::clamp(angular_kp * angle_error, -max_angular_speed, max_angular_speed);
                    cmd.linear.x = 0.0;
                }
                else
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    cmd_vel_pub_->publish(cmd);
                    result->status = true;
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Goal reached!");
                    return;
                }
            }

            // Publicar comando
            cmd_vel_pub_->publish(cmd);

            // Enviar feedback (theta en grados)
            auto feedback = std::make_shared<GoToPose::Feedback>();
            feedback->current_pos = current_pos_;
            feedback->current_pos.theta = current_pos_.theta * 180.0 / M_PI;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
    }
};

// ============================ MAIN ============================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPoseAction>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

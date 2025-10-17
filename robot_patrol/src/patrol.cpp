// NOTE: During shutdown (Ctrl+C), some ROS 2 warnings may appear 
// because publishers are destroyed before the last stop commands are sent. 
// This does not affect functionality — the robot stops safely and 
// the node can be restarted normally.

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <mutex>

#include <csignal>
#include <memory>

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("robot_patrol")
    {
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;

        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
            sub_options
        );

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Patrol::odom_callback, this, std::placeholders::_1),
            sub_options
        );

        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this),
            callback_group_
        );

        direction_ = 0.0;
        obstacle_detected_ = false;
        yaw_ = 0.0;
    }

    void stop_robot_safe()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (timer_) timer_->cancel();

        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;

        for (int i = 0; i < 10; ++i)
        {
            pub_->publish(stop);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::cout << "[INFO] Robot detenido (Ctrl+C detectado)" << std::endl;
    }

private:
    // --------------------------- LASER CALLBACK ---------------------------
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        // Rango que vamos a analizar (−90° a +90°)
        int start_idx = (int)((-M_PI_2 - msg->angle_min) / msg->angle_increment);
        int end_idx   = (int)(( M_PI_2 - msg->angle_min) / msg->angle_increment);

        start_idx = std::max(0, start_idx);
        end_idx   = std::min((int)msg->ranges.size() - 1, end_idx);

        int center_idx = (start_idx + end_idx) / 2;

        // ===========================================================
        // Nueva forma de calcular la ventana frontal 
        // ===========================================================
        double window_angle = 10.0 * M_PI / 180.0;     // radianes
        int window = static_cast<int>(window_angle / msg->angle_increment);

        // Detección: analizar ventana frontal
        float min_front = msg->range_max;
        for (int i = center_idx - window; i <= center_idx + window; i++)
        {
            if (i >= 0 && i < (int)msg->ranges.size() && std::isfinite(msg->ranges[i]))
            {
                min_front = std::min(min_front, msg->ranges[i]);
            }
        }

        obstacle_detected_ = min_front < 0.35;


        if (obstacle_detected_)
        {
            float max_range = 0.0;
            int safest_idx = center_idx;

            for (int i = start_idx; i <= end_idx; i++)
            {
                float r = msg->ranges[i];
                if (std::isfinite(r) && r > max_range && r < msg->range_max)
                {
                    max_range = r;
                    safest_idx = i;
                }
            }

            direction_ = msg->angle_min + safest_idx * msg->angle_increment;
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Safest direction: %.2f rad", direction_);
        }
        else
        {
            direction_ = 0.0;
        }
    }
    // --------------------------- ODOMETRY CALLBACK ---------------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        double qx = msg->pose.pose.orientation.x;
        double qy = msg->pose.pose.orientation.y;
        double qz = msg->pose.pose.orientation.z;
        double qw = msg->pose.pose.orientation.w;

        double siny_cosp = 2.0 * (qw * qz + qx * qy);
        double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
        yaw_ = std::atan2(siny_cosp, cosy_cosp);
    }

    // --------------------------- CONTROL LOOP ---------------------------
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0*M_PI;
        while (angle < -M_PI) angle += 2.0*M_PI;
        return angle;
    }

    void control_loop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1;  // Velocidad base hacia adelante

        static bool turning = false;
        static double target_yaw = 0.0;

        // Iniciar giro si se detecta obstáculo y no se está girando
        if (!turning && obstacle_detected_)
        {
            // Apuntar hacia la dirección más libre
            double angle_offset = direction_;
            target_yaw = normalize_angle(yaw_ + angle_offset);
            turning = true;

            RCLCPP_WARN(this->get_logger(),
                "Obstacle detected → turning toward free direction %.2f rad (target yaw: %.2f)",
                angle_offset, target_yaw);
        }

        if (turning)
        {
            double error = normalize_angle(target_yaw - yaw_);

            if (std::fabs(error) < 0.05)  // tolerancia ~3°
            {
                turning = false;
                cmd.linear.x = 0.1;
                cmd.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Rotation completed → resuming forward.");
            }
            else
            {
                // Control proporcional: giro suave mientras avanza
                double k_p = 1.5;
                double angular_speed = k_p * error;
                angular_speed = std::clamp(angular_speed, -4.0, 4.0);

                cmd.linear.x = 0.1;
                cmd.angular.z = angular_speed;

                RCLCPP_INFO(this->get_logger(), 
                    "Turning... yaw: %.3f, target: %.3f, ω=%.2f", yaw_, target_yaw, angular_speed);
            }
        }
        else
        {
            // Movimiento libre: ligera preferencia direccional (CW o CCW)
            cmd.linear.x = 0.1;
            cmd.angular.z = 0;
        }

        pub_->publish(cmd);
    }

    // --------------------------- VARIABLES ---------------------------
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
    double direction_;
    bool obstacle_detected_;
    double yaw_;
};

// ============================================================
// =============== MANEJO DE CTRL+C ============================
// ============================================================

std::shared_ptr<Patrol> g_node;

void sigint_handler(int)
{
    if (g_node) g_node->stop_robot_safe();
    rclcpp::shutdown();
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<Patrol>();

    std::signal(SIGINT, sigint_handler);

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(g_node);
    exec.spin();

    return 0;
}

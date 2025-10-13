#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <mutex>

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("robot_patrol")
    {
        // Crear callback group Reentrant
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;

        // Suscripción al LIDAR usando el callback group reentrant
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
            sub_options
        );

        //Suscripción a la odometría
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(),
            std::bind(&Patrol::odom_callback, this, std::placeholders::_1),
            sub_options
        );

        // Publisher a /cmd_vel
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer 10 Hz para control_loop usando el mismo callback group
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this),
            callback_group_
        );

        direction_ = 0.0;
        obstacle_detected_ = false;
        yaw_ = 0.0;
    }

private:
    // --------------------------- LASER CALLBACK ---------------------------
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        int start_idx = (int)((-M_PI_2 - msg->angle_min) / msg->angle_increment);
        int end_idx   = (int)(( M_PI_2 - msg->angle_min) / msg->angle_increment);
        
        start_idx = std::max(0, start_idx);
        end_idx   = std::min((int)msg->ranges.size() - 1, end_idx);

        int center_idx = (start_idx + end_idx) / 2;
        obstacle_detected_ = msg->ranges[center_idx] < 0.35;

        if (obstacle_detected_)
        {
            RCLCPP_WARN_STREAM(this->get_logger(), "Obstacle detected!");
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

        //RCLCPP_INFO(this->get_logger(), "Yaw (heading): %.3f rad", yaw_);
    }

    // --------------------------- CONTROL LOOP  ---------------------------
    
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

        // Variables estáticas para controlar el progreso del giro
        static bool turning = false;
        static double target_yaw = 0.0;

        // Iniciar giro si se detecta obstáculo y no se está girando
        if (!turning && obstacle_detected_)
        {
            // Calcular sentido del giro: derecha = -90°, izquierda = +90°
            double angle_offset = (direction_ >= 0.0) ? M_PI_2 : -M_PI_2;
            target_yaw = normalize_angle(yaw_ + angle_offset);

            turning = true;
            RCLCPP_WARN(this->get_logger(),
                        "Obstacle detected → starting turn. Target yaw: %.3f rad", target_yaw);
        }

        // Si está girando → controlar giro hasta alcanzar target_yaw
        if (turning)
        {
            double error = normalize_angle(target_yaw - yaw_);

            if (std::fabs(error) < 0.05)  // tolerancia ~3°
            {
                turning = false;
                cmd.linear.x = 0.0;
                cmd.angular.z = 0.0;
                RCLCPP_INFO(this->get_logger(), "Rotation completed. Resuming forward motion.");
            }
            else
            {
                // Velocidad angular proporcional al signo de 'direction_'
                double base_turn = direction_ / 2.0;

                // Asegurar velocidad mínima de giro
                if (std::fabs(base_turn) < 0.3)
                    base_turn = (base_turn >= 0.0) ? -0.3 : 0.3;

                cmd.linear.x = 0.0;
                cmd.angular.z = base_turn;
                
                RCLCPP_INFO(this->get_logger(), "Turning... current yaw: %.3f rad", yaw_);
            }
        }
        else
        {
            // Sin obstáculo o giro completado → avanzar normalmente
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
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
    float direction_;
    bool obstacle_detected_;
    double yaw_;  // orientación del robot (radianes)
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Patrol>();
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}

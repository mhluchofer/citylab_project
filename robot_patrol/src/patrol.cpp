#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
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

    // --------------------------- CONTROL LOOP  ---------------------------
    
    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1;  // avance por defecto

        rclcpp::Time now = this->now();

        if (obstacle_detected_)
        {
            if (!turning_)
            {
                // Inicializar giro temporal
                turning_ = true;
                turn_start_ = now;
                double desired_angle = M_PI/2;        // giro de 90 grados
                double omega = std::abs(direction_ / 2.0); // velocidad angular basada en la calificación
                turn_duration_ = desired_angle / omega;

                cmd.linear.x = 0.0; // detener avance mientras gira
                cmd.angular.z = (direction_ >= 0) ? omega : -omega;
            }
            else
            {
                double elapsed = (now - turn_start_).seconds();
                if (elapsed >= turn_duration_)
                {
                    // Giro terminado, reanudar avance
                    turning_ = false;
                    cmd.linear.x = 0.1;
                    cmd.angular.z = 0.0;
                }
                else
                {
                    // Mantener giro
                    cmd.linear.x = 0.0;
                    double omega = std::abs(direction_ / 2.0);
                    cmd.angular.z = (direction_ >= 0) ? omega : -omega;
                }
            }
        }
        else
        {
            // Movimiento normal
            turning_ = false;
            cmd.angular.z = 0.0;
        }

        pub_->publish(cmd);
    }

    // --------------------------- VARIABLES ---------------------------
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::mutex mutex_;
    float direction_;
    bool obstacle_detected_;
    bool turning_ = false;
    rclcpp::Time turn_start_;
    double turn_duration_;  // segundos que dura el giro
    double target_angle_;   // ángulo total a girar (rad) 
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

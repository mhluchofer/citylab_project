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

        ranges_ = msg->ranges;
        angle_min_ = msg->angle_min;
        angle_increment_ = msg->angle_increment;

        int start_idx = (int)((-M_PI_2 - angle_min_) / angle_increment_);
        int end_idx   = (int)(( M_PI_2 - angle_min_) / angle_increment_);
        start_idx = std::max(0, start_idx);
        end_idx   = std::min((int)ranges_.size() - 1, end_idx);

        int center_idx = (start_idx + end_idx) / 2;
        obstacle_detected_ = ranges_[center_idx] < 0.35;

        if (obstacle_detected_)
        {
            float max_range = 0.0;
            int safest_idx = center_idx;

            for (int i = start_idx; i <= end_idx; i++)
            {
                float r = ranges_[i];
                if (std::isfinite(r) && r > max_range && r < msg->range_max)
                {
                    max_range = r;
                    safest_idx = i;
                }
            }

            direction_ = angle_min_ + safest_idx * angle_increment_;
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
        cmd.linear.x = 0.1;  // Velocidad base hacia adelante

        std::lock_guard<std::mutex> lock(mutex_);

        // Variables auxiliares para laterales
        bool left_clear = true;
        bool right_clear = true;

        if (obstacle_detected_ || direction_ != 0.0)
        {
            double giro_signo = (direction_ >= 0) ? 1.0 : -1.0;

            // Determinar indices de los rayos extremos según lateral
            int start_idx, end_idx;
            if (giro_signo > 0)
            {
                // Giro a la derecha → revisar lateral izquierdo (0 a +90°)
                start_idx = (int)((0.0 - angle_min_) / angle_increment_);
                end_idx   = (int)((M_PI_2 - angle_min_) / angle_increment_);

                start_idx = std::max(0, start_idx);
                end_idx   = std::min((int)ranges_.size() - 1, end_idx);

                left_clear = true;
                for (int i = start_idx; i <= end_idx; ++i)
                {
                    double r = ranges_[i];
                    if (!std::isfinite(r) || r < 0.35)
                    {
                        left_clear = false;
                        break;
                    }
                }

                if (!left_clear)
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = direction_ / 2.0;
                    RCLCPP_WARN(this->get_logger(), "Obstacle detected on left! Rotating...");
                }
                else
                {
                    cmd.linear.x = 0.1;
                    cmd.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Left lateral free. Moving forward.");
                }
            }
            else
            {
                // Giro a la izquierda → revisar lateral derecho (-90° a 0)
                start_idx = (int)((-M_PI_2 - angle_min_) / angle_increment_);
                end_idx   = (int)((0.0 - angle_min_) / angle_increment_);

                start_idx = std::max(0, start_idx);
                end_idx   = std::min((int)ranges_.size() - 1, end_idx);

                right_clear = true;
                for (int i = start_idx; i <= end_idx; ++i)
                {
                    double r = ranges_[i];
                    if (!std::isfinite(r) || r < 0.35)
                    {
                        right_clear = false;
                        break;
                    }
                }

                if (!right_clear)
                {
                    cmd.linear.x = 0.0;
                    cmd.angular.z = direction_ / 2.0;
                    RCLCPP_WARN(this->get_logger(), "Obstacle detected on right! Rotating...");
                }
                else
                {
                    cmd.linear.x = 0.1;
                    cmd.angular.z = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Right lateral free. Moving forward.");
                }
            }
        }
        else
        {
            // Sin obstáculo frontal → avanzar normalmente
            cmd.linear.x = 0.1;
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
    std::vector<float> ranges_;
    float angle_min_;
    float angle_increment_;
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

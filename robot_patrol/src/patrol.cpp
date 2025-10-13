#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node
{
public:
    Patrol() : Node("patrol_node")
    {
        // Subscriber al láser
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Patrol::laser_callback, this, std::placeholders::_1));

        // Publisher a cmd_vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer para control loop 10 Hz
        control_timer_ = this->create_wall_timer(
            100ms, std::bind(&Patrol::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Patrol Node Ready");
    }

private:
    // Variables
    std::vector<float> laser_ranges_;
    float obstacle_threshold_ = 0.35; // 35 cm

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Callback del láser
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        laser_ranges_ = msg->ranges;

        // Opcional: imprimir la distancia mínima frontal para debug
        if (!laser_ranges_.empty())
        {
            size_t start_idx = laser_ranges_.size() / 4;
            size_t end_idx = 3 * laser_ranges_.size() / 4;
            float min_front = *std::min_element(laser_ranges_.begin() + start_idx,
                                                laser_ranges_.begin() + end_idx);
            RCLCPP_INFO(this->get_logger(), "Min front distance: %.2f m", min_front);
        }
    }

    // Loop de control
    void control_loop()
    {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = 0.1; // Velocidad hacia adelante

        // Verificar obstáculos frontales
        if (!laser_ranges_.empty())
        {
            size_t start_idx = laser_ranges_.size() / 4;
            size_t end_idx = 3 * laser_ranges_.size() / 4;
            float min_front = *std::min_element(laser_ranges_.begin() + start_idx,
                                                laser_ranges_.begin() + end_idx);

            if (min_front < obstacle_threshold_)
            {
                cmd_msg.linear.x = 0.0; // Detener el robot
                RCLCPP_WARN(this->get_logger(), "Obstacle detected! Stopping.");
            }
        }

        // Publicar comando
        cmd_vel_pub_->publish(cmd_msg);
    }
};

// Main
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

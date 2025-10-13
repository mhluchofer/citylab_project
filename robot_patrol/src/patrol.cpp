#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class PatrolTestIO : public rclcpp::Node
{
public:
    PatrolTestIO() : Node("patrol_test_io")
    {
        RCLCPP_INFO(this->get_logger(), "🔹 Iniciando nodo de prueba I/O...");

        // Publisher: enviamos comandos de velocidad
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Subscriber: recibimos datos del LIDAR
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&PatrolTestIO::laser_callback, this, _1));

        // Timer: publicamos una velocidad cada 0.5 s
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PatrolTestIO::publish_test_cmd, this));
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        size_t num_rays = msg->ranges.size();
        int center_idx = num_rays / 2;
        float center_range = msg->ranges[center_idx];

        RCLCPP_INFO(this->get_logger(),
            "LIDAR recibido -> rays: %zu, rango_frontal: %.2f m, ángulo_min: %.2f, ángulo_max: %.2f",
            num_rays, center_range, msg->angle_min, msg->angle_max);
    }

    void publish_test_cmd()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1;   // Avanza lentamente
        cmd.angular.z = 0.0;  // Sin giro

        pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "📤 Publicando velocidad: linear=%.2f, angular=%.2f",
                    cmd.linear.x, cmd.angular.z);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PatrolTestIO>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

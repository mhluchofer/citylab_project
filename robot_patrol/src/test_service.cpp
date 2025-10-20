#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "robot_interfaces/srv/get_direction.hpp"  // Cambia según tu paquete de interfaces

using std::placeholders::_1;

class ServiceTester : public rclcpp::Node
{
public:
    ServiceTester() : Node("service_tester")
    {
        // Crear cliente del servicio
        client_ = this->create_client<robot_interfaces::srv::GetDirection>("/direction_service");

        // Subscribir al tópico de láser
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&ServiceTester::laser_callback, this, _1)
        );
    }

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(this->get_logger(), "Service not available yet.");
            return;
        }

        auto request = std::make_shared<robot_interfaces::srv::GetDirection::Request>();
        request->laser_data = *msg;

        auto future = client_->async_send_request(request,
            [this](rclcpp::Client<robot_interfaces::srv::GetDirection>::SharedFuture response) {
                RCLCPP_INFO(this->get_logger(), "Direction: %s", response.get()->direction.c_str());
            }
        );
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Client<robot_interfaces::srv::GetDirection>::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ServiceTester>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

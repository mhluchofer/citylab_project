// ============================ HEADER ============================
#include <memory>
#include <mutex>
#include <csignal>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_interfaces/srv/get_direction.hpp"

using namespace std::chrono_literals;

// ============================ PATROL NODE ========================
class PatrolWithService : public rclcpp::Node
{
public:
    PatrolWithService() : Node("patrol_with_service")
    {
        // Mutex-safe callback group
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;

        // Subscribe to LaserScan
        sub_scan_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&PatrolWithService::laser_callback, this, std::placeholders::_1),
            sub_options
        );

        // Publisher for cmd_vel
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Service client
        client_ = this->create_client<robot_interfaces::srv::GetDirection>("/direction_service");

        // Control loop timer
        timer_ = this->create_wall_timer(
            100ms, std::bind(&PatrolWithService::control_loop, this)
        );
    }

    void stop_robot_safe()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (timer_) timer_->cancel();

        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.angular.z = 0.0;

        for (int i = 0; i < 10; ++i) {
            pub_->publish(stop);
            std::this_thread::sleep_for(50ms);
        }

        RCLCPP_INFO(this->get_logger(), "Robot stopped safely (Ctrl+C).");
    }

private:
    // --------------------------- LASER CALLBACK ---------------------------
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_scan_ = msg;
    }

    // --------------------------- CONTROL LOOP ---------------------------
    void control_loop()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (!last_scan_) return;

        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Direction service not available.");
            return;
        }

        auto request = std::make_shared<robot_interfaces::srv::GetDirection::Request>();
        request->laser_data = *last_scan_;

        // Asynchronous call to service
        auto future = client_->async_send_request(request,
            [this](rclcpp::Client<robot_interfaces::srv::GetDirection>::SharedFuture future_response)
            {
                auto response = future_response.get();
                process_service_response(response->direction);
            });
    }

    void process_service_response(const std::string & direction)
    {
        geometry_msgs::msg::Twist cmd;

        if (direction == "forward") {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.0;
        } else if (direction == "left") {
            cmd.linear.x = 0.1;
            cmd.angular.z = 0.5;
        } else if (direction == "right") {
            cmd.linear.x = 0.1;
            cmd.angular.z = -0.5;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Direction: %s → cmd_vel linear: %.2f angular: %.2f",
                    direction.c_str(), cmd.linear.x, cmd.angular.z);

        pub_->publish(cmd);
    }

    // --------------------------- MEMBERS ---------------------------
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<robot_interfaces::srv::GetDirection>::SharedPtr client_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    std::mutex mutex_;
};

// ============================================================
// =============== CTRL+C HANDLER ==============================
std::shared_ptr<PatrolWithService> g_node;

void sigint_handler(int)
{
    if (g_node) g_node->stop_robot_safe();
    rclcpp::shutdown();
}

// ============================================================

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    g_node = std::make_shared<PatrolWithService>();
    std::signal(SIGINT, sigint_handler);

    // Spin the node normally
    rclcpp::spin(g_node);

    return 0;
}

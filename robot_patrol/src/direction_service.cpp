// ============================ HEADER ============================
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "robot_interfaces/srv/get_direction.hpp"  // Ajustado a tu nuevo paquete

using std::placeholders::_1;
using std::placeholders::_2;

// ============================ SERVICE ===========================
class DirectionService : public rclcpp::Node
{
public:
    DirectionService() : Node("direction_service"), last_direction_("forward"), lock_cycles_(3)
    {
        std::string service_name = "/direction_service";
        service_ = this->create_service<robot_interfaces::srv::GetDirection>(
            service_name,
            std::bind(&DirectionService::handle_service, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "%s Service Server Ready", service_name.c_str());
    }

private:
    rclcpp::Service<robot_interfaces::srv::GetDirection>::SharedPtr service_;

    // ============================ LOCK TEMPORAL ============================
    std::string last_direction_;  // Memoriza la última dirección tomada
    int lock_cycles_;             // Cantidad mínima de ciclos antes de permitir cambio
    int current_lock_ = 0;        // Contador de ciclos en la misma dirección

    // ============================ DECISION ===========================
    void handle_service(
        const std::shared_ptr<robot_interfaces::srv::GetDirection::Request> request,
        std::shared_ptr<robot_interfaces::srv::GetDirection::Response> response)
    {
        auto ranges = request->laser_data.ranges;
        size_t n = ranges.size();

        if (n == 0)
        {
            RCLCPP_WARN(this->get_logger(), "No laser data received!");
            response->direction = "forward";
            return;
        }

        // ============================ DIVIDE SECTIONS ===========================
        int sec_rays = n / 3;
        auto right_section = std::vector<float>(ranges.begin(), ranges.begin() + sec_rays);
        auto front_section = std::vector<float>(ranges.begin() + sec_rays, ranges.begin() + 2 * sec_rays);
        auto left_section = std::vector<float>(ranges.begin() + 2 * sec_rays, ranges.end());

        auto sum_valid = [](const std::vector<float> &sec)
        {
            double sum = 0.0;
            for (auto r : sec)
                if (std::isfinite(r))
                    sum += r;
            return sum;
        };

        double total_dist_sec_right = sum_valid(right_section);
        double total_dist_sec_front = sum_valid(front_section);
        double total_dist_sec_left = sum_valid(left_section);

        auto min_front = *std::min_element(front_section.begin(), front_section.end());
        if (!std::isfinite(min_front))
            min_front = 0.0;

        // ============================ PORCENTAJE LIBRE FRONTAL ============================
        int free_count = 0;
        for (auto r : front_section)
            if (std::isfinite(r) && r >= 0.35)  // mismo umbral frontal
                free_count++;

        double free_percentage = static_cast<double>(free_count) / front_section.size();

        // ============================ LÓGICA CON LOCK TEMPORAL Y % LIBRE ============================
        std::string new_direction;

        // Si hay suficiente espacio libre adelante, seguimos "forward" aunque min_front esté bajo
        if (free_percentage >= 0.7)  // por ejemplo, 60% de frontal libre
        {
            new_direction = "forward";
        }
        else
        {
            if (total_dist_sec_left >= total_dist_sec_right)
                new_direction = "left";
            else
                new_direction = "right";
        }

        // Aplicar lock temporal
        if (new_direction != last_direction_)
        {
            if (current_lock_ < lock_cycles_)
            {
                response->direction = last_direction_;
                current_lock_++;
                RCLCPP_WARN(this->get_logger(), "Direction locked: keeping '%s' to avoid oscillation", last_direction_.c_str());
            }
            else
            {
                response->direction = new_direction;
                last_direction_ = new_direction;
                current_lock_ = 0;
            }
        }
        else
        {
            response->direction = new_direction;
            current_lock_ = 0;
        }

        // ============================ LOGS ============================
        RCLCPP_INFO(
            this->get_logger(),
            "Front min: %.2f, Free%%: %.2f → Direction: %s (Totals: F=%.2f L=%.2f R=%.2f)",
            min_front,
            free_percentage,
            response->direction.c_str(),
            total_dist_sec_front,
            total_dist_sec_left,
            total_dist_sec_right);
    }
};

// ============================ MAIN ===============================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DirectionService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

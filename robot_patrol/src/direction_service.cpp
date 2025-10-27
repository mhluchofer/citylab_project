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

        // Ajuste para el robot real (FastBot)
        // Si el escaneo empieza en 0 rad y va hasta 2π, rotamos para centrar el frente
        if (request->laser_data.angle_min >= 0.0 && request->laser_data.angle_max > 6.0)
        {
            size_t half = n / 2;
            std::rotate(ranges.begin(), ranges.begin() + half, ranges.end());
            //RCLCPP_INFO(this->get_logger(), "Laser ranges rotated to align front center (0-2π detected).");
        }


        // ============================ DIVIDE SECTIONS ===========================
        int sec_rays = n / 3;
        auto right_section = std::vector<float>(ranges.begin(), ranges.begin() + sec_rays);
        auto front_section = std::vector<float>(ranges.begin() + sec_rays, ranges.begin() + 2 * sec_rays);
        auto left_section = std::vector<float>(ranges.begin() + 2 * sec_rays, ranges.end());

        auto filter_finite = [](const std::vector<float>& sec) {
            std::vector<float> result;
            for (float v : sec)
                if (std::isfinite(v))
                    result.push_back(v);
            return result;
        };

        auto finite_front = filter_finite(front_section);
        auto finite_left  = filter_finite(left_section);
        auto finite_right = filter_finite(right_section);

    // ============================ CALCULAR SUMATORIAS Y MIN FRONT ===========================
        auto sum_section = [](const std::vector<float>& sec){
            double sum = 0.0;
            for (float v : sec)
                sum += v;
            return sum;
        };

        double total_dist_sec_front = sum_section(finite_front);
        double total_dist_sec_left  = sum_section(finite_left);
        double total_dist_sec_right = sum_section(finite_right);

        double min_front = finite_front.empty() ? 0.0 : *std::min_element(finite_front.begin(), finite_front.end());

        // ============================ PORCENTAJE LIBRE FRONTAL ============================
        int free_count = 0;
        for (auto r : front_section)
            if (std::isfinite(r) && r >= 0.35)  // mismo umbral frontal
                free_count++;

        double free_percentage = static_cast<double>(free_count) / front_section.size();

        // ============================ LÓGICA CON LOCK TEMPORAL Y % LIBRE ============================
        std::string new_direction;

        // Si hay suficiente espacio libre adelante, seguimos "forward" aunque min_front esté bajo
        if (free_percentage >= 0.6)  // por ejemplo, 70% de frontal libre
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

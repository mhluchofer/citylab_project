// ============================ HEADER ============================
#include <memory>
#include <vector>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "robot_interfaces/srv/get_direction.hpp" // Ajustado a tu nuevo paquete

using std::placeholders::_1;
using std::placeholders::_2;

// ============================ SERVICE ===========================
class DirectionService : public rclcpp::Node
{
public:
    DirectionService() : Node("direction_service")
    {
        // Crear el servicio
        std::string service_name = "/direction_service";
        service_ = this->create_service<robot_interfaces::srv::GetDirection>(
            service_name,
            std::bind(&DirectionService::handle_service, this, _1, _2)
        );

        RCLCPP_INFO(this->get_logger(), "%s Service Server Ready", service_name.c_str());
    }

private:
    rclcpp::Service<robot_interfaces::srv::GetDirection>::SharedPtr service_;

    // ============================ DECISION ===========================
    void handle_service(
        const std::shared_ptr<robot_interfaces::srv::GetDirection::Request> request,
        std::shared_ptr<robot_interfaces::srv::GetDirection::Response> response)
    {
        auto ranges = request->laser_data.ranges;
        size_t total_rays = ranges.size();

        int section_size = total_rays / 3; // Aproximadamente 60º por sección

        // Dividir el LIDAR en 3 secciones
        auto right_section = std::vector<float>(ranges.begin(), ranges.begin() + section_size);
        auto front_section = std::vector<float>(ranges.begin() + section_size, ranges.begin() + 2 * section_size);
        auto left_section  = std::vector<float>(ranges.begin() + 2 * section_size, ranges.end());

        // Función para sumar distancias válidas (ignorar NaN o infinito)
        auto sum_valid = [](const std::vector<float>& sec) {
            double sum = 0.0;
            for (auto r : sec) if (std::isfinite(r)) sum += r;
            return sum;
        };

        double total_right = sum_valid(right_section);
        double total_front = sum_valid(front_section);
        double total_left  = sum_valid(left_section);

        // ============================ LOGIC ============================
        auto min_front = *std::min_element(front_section.begin(), front_section.end());
        if (!std::isfinite(min_front)) min_front = 0.0;

        if (min_front >= 0.35) {
            response->direction = "forward";
        } else {
            // Elegir la sección más libre (izquierda o derecha)
            response->direction = (total_left >= total_right) ? "left" : "right";
        }

        // ============================ LOGS =============================
        RCLCPP_INFO(this->get_logger(),
            "Front min: %.2f → Direction: %s (Totals: F=%.2f L=%.2f R=%.2f)",
            min_front, response->direction.c_str(), total_front, total_left, total_right);
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

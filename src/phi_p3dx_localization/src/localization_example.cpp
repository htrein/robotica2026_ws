#include "phi_p3dx_localization/localization_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>

/**
 * @class LocalizationExample
 * @brief Exemplo de uso de LocalizationNode.
 * 
 * Demonstra como estender a classe base para implementar comportamento customizado.
 * Neste exemplo, as partículas rotacionam continuamente, permitindo observar:
 * - Como a pose estimada acompanha a rotação
 * - Como a covariância se comporta com movimento
 * - Como sobrescrever update_particles() para adicionar lógica
 * 
 * **Parâmetros:**
 * - Herda todos os parâmetros de LocalizationNode
 * - `rotation_speed` (double): Velocidade de rotação em rad/s (default: 0.5)
 * 
 * **Como usar:**
 * ```bash
 * ros2 run phi_p3dx_localization localization_example_cpp --ros-args -p num_particles:=500
 * ```
 */
class LocalizationExample : public LocalizationNode
{
public:
  /**
   * @brief Construtor do exemplo de localização.
   * 
   * @param node_name Nome do nó no ROS 2 (default: "localization_example_cpp")
   */
  explicit LocalizationExample(const std::string &node_name = "localization_example_cpp")
    : LocalizationNode(node_name),
      rotation_speed_(0.5),
      last_update_time_(this->now())
  {
    // Declarar e obter parâmetro de velocidade de rotação
    this->declare_parameter<double>("rotation_speed", 0.5);
    rotation_speed_ = this->get_parameter("rotation_speed").as_double();

    RCLCPP_INFO(this->get_logger(), 
      "[%s] Exemplo de localização com rotação inicializado.",
      node_name.c_str());
    RCLCPP_INFO(this->get_logger(), 
      "Velocidade de rotação: %.2f rad/s", rotation_speed_);
  }

  virtual ~LocalizationExample() = default;

private:
  /**
   * @brief Sobrescreve o método de atualização de partículas para rotacioná-las.
   * 
   * A cada chamada, incrementa o ângulo (theta) de cada partícula com base
   * na velocidade de rotação e no tempo decorrido desde a última atualização.
   */
  void update_particles() override
  {
    // Calcula delta de tempo desde a última atualização
    rclcpp::Time now = this->now();
    double dt = (now - last_update_time_).seconds();
    last_update_time_ = now;

    // Incremento de ângulo
    double delta_theta = rotation_speed_ * dt;

    // Rotaciona todas as partículas
    for (auto &p : particles_) {
      p.theta += delta_theta;
      
      // Normaliza theta para [-π, π]
      while (p.theta > M_PI) {
        p.theta -= 2.0 * M_PI;
      }
      while (p.theta < -M_PI) {
        p.theta += 2.0 * M_PI;
      }
    }
  }

private:
  double rotation_speed_;       // Velocidade de rotação em rad/s
  rclcpp::Time last_update_time_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LocalizationExample>("localization_example");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

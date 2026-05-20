#include "phi_p3dx_mapping/exploration_node.hpp"
#include <cstdlib>

/**
 * @brief Exemplo simples de nodo de exploração.
 *
 * Realiza uma caminhada aleatória: anda para frente até encontrar 
 * um obstáculo usando os dados do laser de varredura. Ao detectar,
 * gira para evitar colisão e escolhe uma direção aleatória por tempo.
 * O mapa é recebido via callback apenas para demonstrar o consumo,
 * mas não é ativamente usado para decidir o caminho na lógica básica.
 */
class ExplorationExample : public ExplorationNode
{
public:
  ExplorationExample() : ExplorationNode("exploration_example_cpp") {
    // Semente para direção aleatória
    srand(time(NULL));
  }

protected:
  void control_loop() override
  {
    if (laser_ranges_.empty()) {
      return; // Aguarda dados do laser
    }

    double front_dist = get_front_distance(20.0); // 20 graus para cada lado

    // Distância de segurança (parar e girar se menor que isso)
    const double MIN_DIST = 1.0;

    if (front_dist < MIN_DIST) {
      // Obstáculo detectado, girar
      RCLCPP_INFO(this->get_logger(), "Obstáculo detectado a frente (%.2fm). Virando...", front_dist);
      publish_target_pose(0.0, 0.0, 1.57, "base_link"); // Intenção: virar à esquerda
      publish_velocity(0.0, 0.4);
    } else {
      // Caminho livre, avançar
      publish_target_pose(0.0, 0.0, 0.0, "base_link"); // Intenção: ir em frente
      publish_velocity(0.25, 0.0);
    }
  }

  void on_map() override
  {
    // Apenas para ilustrar a leitura do mapa gerado pelo outro nodo
    if (map_msg_) {
      RCLCPP_INFO_ONCE(this->get_logger(), "Mapa recebido! Dimensões: %dx%d, Resolução: %.2f", 
                       map_msg_->info.width, map_msg_->info.height, map_msg_->info.resolution);

      // Imprimir o valor e indice da célula do robo
      int robot_x = map_msg_->info.width / 2;
      int robot_y = map_msg_->info.height / 2;
      int robot_index = robot_x + robot_y * map_msg_->info.width;
      RCLCPP_INFO(this->get_logger(), "Valor da célula onde está o robô: %d", map_msg_->data[robot_index]);
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationExample>());
  rclcpp::shutdown();
  return 0;
}

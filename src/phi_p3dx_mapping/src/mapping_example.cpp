#include "phi_p3dx_mapping/mapping_node.hpp"
#include <cmath>
#include <algorithm>

/**
 * @brief Exemplo simples de mapeamento iterativo.
 *
 * Registra as posições já visitadas como área livre.
 */
class MappingExample : public MappingNode
{
public:
  MappingExample() : MappingNode("mapping_example_cpp") {}

protected:
  void on_odom() override
  {
    // Toda vez que a odometria chega, marca um círculo ao redor da posição como livre
    mark_free_space(x_, y_, 0.5); // Raio de 0.5 m
    publish_map();
  }

private:
  void mark_free_space(double wx, double wy, double radius_m)
  {
    int r_cells = (int)(radius_m / resolution_);
    auto [mx, my] = meters_to_cells(wx, wy);

    int width_cells = map_msg_.info.width;
    int height_cells = map_msg_.info.height;

    // Percorre uma janela ao redor da posição do robô
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        int nx = mx + dx;
        int ny = my + dy;

        // Verifica se a célula está dentro dos limites da grade
        if (nx >= 0 && nx < width_cells && ny >= 0 && ny < height_cells) {
          // Verifica se está dentro do raio do círculo
          if (dx*dx + dy*dy <= r_cells*r_cells) {
            int idx = ny * width_cells + nx;
            map_msg_.data[idx] = 0; // 0 significa espaço livre
          }
        }
      }
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MappingExample>());
  rclcpp::shutdown();
  return 0;
}

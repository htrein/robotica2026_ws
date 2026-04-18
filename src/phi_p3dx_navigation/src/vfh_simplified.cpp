#include "navigation_node.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class VFHSimplified : public NavigationNode {
public:
  VFHSimplified() : NavigationNode("vfh_simplified") {}

private:
  void control_loop() override {

    const double goal_tolerance = 0.2;
    const double sector_threshold = 0.5;  
    double v_linear = 0.15; 

    // 0. VERIFICAÇÕES INICIAIS
    if (!has_goal() || !has_laser_data()){
      stop(); 
      return; 
    }
    if (distance_to_goal() < goal_tolerance){ 
      clear_goal(); 
      stop(); 
      return; 
    }

    // 1. DISCRETIZAR O ESPAÇO ANGULAR EM SETORES
    int num_sectors = 36; 
    const double sector_size = M_PI / num_sectors;
    const double d_max = 2.0; // Distância máxima de influência dos obstáculos
    const double ROBOT_RADIUS = 0.25; // Raio do robô 

    // 2. CONSTRUIR HISTOGRAMA POLAR
    std::vector<double> histogram(num_sectors, 0.0); 

    for (size_t i = 0; i < laser_ranges_.size(); ++i) {
      double angle = laser_angle_min_ + i * laser_angle_increment_; // Ângulo do laser
      double r = laser_ranges_[i]; // Distância do laser

      // Filtrar apenas o campo frontal
      if (angle < -M_PI/2.0 || angle > M_PI/2.0) 
        continue;
      
      // Ignorar leituras inválidas ou fora do alcance
      if (std::isinf(r) || std::isnan(r) || r > d_max) 
        continue;

      // Calcula o ângulo baseado no raio do robô
      double inflation = (r > ROBOT_RADIUS) ? std::asin(ROBOT_RADIUS / r) : M_PI/2.0; 
      
      // Mapear faixa de ângulos para os índices dos setores 
      int k_start = std::floor((angle - inflation + M_PI/2.0) / sector_size);
      int k_end = std::floor((angle + inflation + M_PI/2.0) / sector_size);
      k_start = std::max(0, k_start);
      k_end = std::min(num_sectors - 1, k_end);
      
      // Preencher todos os setores atingidos pelo obstáculo 
      for (int k = k_start; k <= k_end; ++k) {
          histogram[k] += 1.0 * (d_max - r); 
      }
    }
    // 2.1 SUAVIZAR O HISTOGRAMA 
    std::vector<double> smoothed_histogram(num_sectors, 0.0);
    int window_size = 3; 
    for (int k = 0; k < num_sectors; ++k) {
      double sum = 0.0;
      int total_weight = 0;
      for (int i = -window_size; i <= window_size; ++i) {
        int weight = window_size - std::abs(i) + 1;
        int index = k + i;
        if (index >= 0 && index < num_sectors) {
          sum += weight * histogram[index];
          total_weight += weight;
        }
      }
      smoothed_histogram[k] = sum / total_weight;
    }
    histogram = smoothed_histogram;

    // 3. IDENTIFICAR INTERVALOS CONTÍGUOS DE SETORES LIVRES
    struct Sector { 
      int start; 
      int end; 
    };
    std::vector<Sector> sectors;
    int current_start = -1;

    for (int k = 0; k < num_sectors; ++k) {
      if (histogram[k] < sector_threshold) {
        if (current_start == -1) 
          current_start = k;
      } else if (current_start != -1) {
        sectors.push_back({current_start, k - 1});
        current_start = -1;
      }
    }
    if (current_start != -1) 
      sectors.push_back({current_start, num_sectors - 1});

    // 4. SELECIONAR MELHOR SETOR COM BASE NA DIREÇÃO DO OBJETIVO
    int target_sector = std::floor((angle_to_goal() + M_PI/2.0) / sector_size);
    target_sector = std::max(0, std::min(num_sectors - 1, target_sector));

    int best_sector = -1;
    bool target_is_free = false;

    // Verificar se o objetivo está em um setor livre
    for (const auto& sector : sectors) {
      if (target_sector >= sector.start && target_sector <= sector.end) {
        best_sector = target_sector;
        target_is_free = true;
        break;
      }
    }

    // Caso o objetivo esteja bloqueado, selecionar o centro do setor livre mais próximo 
    if (!target_is_free && !sectors.empty()) {
      double min_dist_sector = 1000.0;
      for (const auto& sector : sectors) {
        int center = (sector.start + sector.end) / 2;
        double dist = std::abs(center - target_sector);
        if (dist < min_dist_sector) {
          min_dist_sector = dist;
          best_sector = center;
        }
      }
    }

    // 5. GERAR VELOCIDADE 
    if (best_sector != -1) {
      // Remapear índice do setor para ângulo
      double chosen_angle = -M_PI/2.0 + (best_sector + 0.5) * sector_size;

      const double Kp_w = 0.6;
      const double max_w = 0.5; 
  
      double w = Kp_w * chosen_angle;
      w = std::clamp(w, -max_w, max_w); 

      // Se precisar girar muito, reduz a velocidade linear
      double v = v_linear;
      if (std::abs(chosen_angle) > 0.5) {
          v *= 0.5; 
      }
      publish_velocity(v, w);
    } else {
      // Se não houver setores livres, gira no próprio eixo
      publish_velocity(0.0, 0.3); 
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VFHSimplified>());
  rclcpp::shutdown();
  return 0;
}
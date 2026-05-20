#include "phi_p3dx_mapping/mapping_node.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <algorithm>
#include <vector>

class Tp3MappingCpp : public MappingNode
{
public:
  Tp3MappingCpp() : MappingNode("tp3_mapping_cpp")
  {
    auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", sensor_qos, std::bind(&Tp3MappingCpp::laser_callback, this, std::placeholders::_1));
    int total = map_msg_.info.width * map_msg_.info.height;
    log_odds_.assign(total, 0.0);
    observed_.assign(total, false);
  }

protected:
  void on_odom() override
  {
    if (laser_ranges_.empty() || !new_laser_data_) 
      return;
    new_laser_data_ = false;

    int W = static_cast<int>(map_msg_.info.width);
    int H = static_cast<int>(map_msg_.info.height);

    // add o deslocamento do laser: 0.15m pra frente do centro
    double laser_offset_x = 0.15; 
    
    // calcula a posição do sensor laser
    double sensor_x = scan_x_ + laser_offset_x * std::cos(scan_theta_);
    double sensor_y = scan_y_ + laser_offset_x * std::sin(scan_theta_);
    auto [rx, ry] = meters_to_cells(sensor_x, sensor_y);

    int num_beams = static_cast<int>(laser_ranges_.size());
    for (int i = 0; i < num_beams; ++i) {
      float range = laser_ranges_[i];

      if (std::isnan(range) || range < 0.05f) continue;

      // define se o feixe bateu num obstáculo ou atingiu o alcance máximo livre
      bool hit_obstacle = (!std::isinf(range) && range < laser_range_max_ - 0.05);
      double capped_range = std::isinf(range) ? laser_range_max_ : std::min(static_cast<double>(range), laser_range_max_);

      // usa o ângulo sincronizado pro feixe
      double beam_angle = scan_theta_ + laser_angle_min_ + i * laser_angle_increment_;

      // fim do feixe
      double ex = sensor_x + capped_range * std::cos(beam_angle);
      double ey = sensor_y + capped_range * std::sin(beam_angle);
      auto [ex_cell, ey_cell] = meters_to_cells(ex, ey);
      raytrace(rx, ry, ex_cell, ey_cell, W, H, hit_obstacle);
    }

    // footprint do robô é livre
    auto [base_rx, base_ry] = meters_to_cells(scan_x_, scan_y_);
    mark_robot_footprint(base_rx, base_ry, W, H, 0.4);

    // converte log-odds pra probabilidade
    update_occupancy_grid(W, H);
    publish_map();
  }

private:
  static constexpr double p_occ_  = 0.7;
  static constexpr double p_free_ = 0.3;
  static constexpr double l_min_  = -6.0;
  static constexpr double l_max_  =  6.0;
  // log-odds
  const double l_occ_  = std::log(p_occ_  / (1.0 - p_occ_));
  const double l_free_ = std::log(p_free_ / (1.0 - p_free_));

  double laser_angle_min_       = 0.0; 
  double laser_angle_increment_ = 0.0;
  double laser_range_max_       = 8.0;

  std::vector<float> laser_ranges_;
  std::vector<double> log_odds_;    
  std::vector<bool> observed_;      
  bool new_laser_data_ = false;

  // variaveis para guardar o estado exato da odometria no instante do laser
  double scan_x_ = 0.0;
  double scan_y_ = 0.0;
  double scan_theta_ = 0.0;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    laser_ranges_ = msg->ranges;
    laser_angle_min_ = msg->angle_min;
    laser_angle_increment_ = msg->angle_increment;
    laser_range_max_ = static_cast<double>(msg->range_max);
    scan_x_ = x_;
    scan_y_ = y_;
    scan_theta_ = theta_;
    new_laser_data_ = true;
  }

  void raytrace(int x0, int y0, int x1, int y1, int W, int H, bool hit_obstacle)
  {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    int cx = x0;
    int cy = y0;

    while (true) {
      bool at_end = (cx == x1 && cy == y1);

      if (cx >= 0 && cx < W && cy >= 0 && cy < H) {
        int idx = cy * W + cx;

        // se for o ponto final e houver colisao, aumenta a chance de estar ocupado
        if (at_end && hit_obstacle) {
          log_odds_[idx] += l_occ_;
        } else { // caso contrário, aumenta a chance de estar livre
          log_odds_[idx] += l_free_;
        }
        log_odds_[idx] = std::clamp(log_odds_[idx], l_min_, l_max_);
        observed_[idx] = true;
      }

      if (at_end) 
        break;

      //algoritmo de bresenham para desenhar a reta do feixe laser no grid
      int e2 = 2 * err;
      if (e2 > -dy) { 
        err -= dy; 
        cx += sx; 
      }
      if (e2 < dx) {
        err += dx; 
        cy += sy; 
      }
    }
  }

  // força a área da base do robô a ser livre
  void mark_robot_footprint(int rx, int ry, int W, int H, double radius_m)
  {
    int r_cells = static_cast<int>(radius_m / map_msg_.info.resolution);
    for (int dy = -r_cells; dy <= r_cells; ++dy) {
      for (int dx = -r_cells; dx <= r_cells; ++dx) {
        int nx = rx + dx;
        int ny = ry + dy;
        if (nx >= 0 && nx < W && ny >= 0 && ny < H) {
          if (dx*dx + dy*dy <= r_cells*r_cells) {
            int idx = ny * W + nx;
            log_odds_[idx] += l_free_;
            log_odds_[idx] = std::clamp(log_odds_[idx], l_min_, l_max_);
            observed_[idx] = true;
          }
        }
      }
    }
  }

  // converte log-odds pra probabilidade
  void update_occupancy_grid(int W, int H)
  {
    int total = W * H;
    for (int i = 0; i < total; ++i) {
      if (!observed_[i]) {
        map_msg_.data[i] = -1; // -1 indica área desconhecida
      } else {
        //sigmoide
        double p = 1.0 / (1.0 + std::exp(-log_odds_[i]));
        map_msg_.data[i] = static_cast<int8_t>(std::round(p * 100.0));
      }
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tp3MappingCpp>());
  rclcpp::shutdown();
  return 0;
}
#include "navigation_node.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class WallFollowerPID : public NavigationNode {
public:
  WallFollowerPID() : NavigationNode("wall_follower_pid") {
    prev_error = 0.0;
    integral = 0.0;
    following_wall = false; 
  }

private:
  double prev_error;
  double integral;
  bool following_wall;  

  void control_loop() override {
    // 0. VERIFICAÇÕES INICIAIS
    if (!has_goal()) {
      stop();
      return;
    }
    if (!has_laser_data())
      return;

    // 1. PARÂMETROS DE CONTROLE
    const double kp = 0.2, ki = 0.0, kd = 0.2;
    const double desired_dist = 0.55;
    const double side = -1.0;  // -1.0 = direita, +1.0 = esquerda
    const double speed_v = 0.15;
    const double dt = 0.05;

    // 2. SEGURANÇA 
    double front_dist = get_front_distance(45.0);
    if (front_dist < 0.8) {
      publish_velocity(0.05, -side * 0.35);
      return;
    }

    // 3. MEDIÇÃO DA DISTÂNCIA LATERAL E DETECÇÃO DE PAREDE
    double current_dist = get_side_distance(side);
    if (current_dist < 1.5) {
      following_wall = true;
    } else if (current_dist > 2.5) {
      following_wall = false;
    }

    // 4. SE SEM PAREDE, NAVEGA EM DIREÇÃO AO OBJETIVO
    if (!following_wall) {
      double angle_err = angle_to_goal();
      double w = std::clamp(0.8 * angle_err, -0.8, 0.8);
      publish_velocity(speed_v, w);
      return;
    }

    // 5. CÁLCULO DO CTE = distância desejada - distância medida
    double error = desired_dist - current_dist;

    error = std::clamp(error, -0.5, 0.5); //limite do erro

    // 6. CONTROLADOR PID
    integral = std::clamp(integral + (error * dt), -0.5, 0.5);
    double derivative = (error - prev_error) / dt;
    double w = -side * (kp * error + ki * integral + kd * derivative);
    prev_error = error;
    double w_clamped = std::clamp(w, -0.8, 0.8);
    publish_velocity(speed_v, w_clamped);
  }

  double get_side_distance(double side) {
    double target_angle = (side < 0) ? -M_PI / 2.0 : M_PI / 2.0;
    double sum = 0.0;
    int count = 0;
    for (size_t i = 0; i < laser_ranges_.size(); ++i) {
      double angle = laser_angle_min_ + i * laser_angle_increment_;
      if (std::abs(angle - target_angle) < 0.30) {
        if (std::isfinite(laser_ranges_[i]) && laser_ranges_[i] > 0.05) {
          sum += laser_ranges_[i];
          count++;
        }
      }
    }
    return (count > 0) ? (sum / count) : 2.5; 
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WallFollowerPID>());
  rclcpp::shutdown();
  return 0;
}
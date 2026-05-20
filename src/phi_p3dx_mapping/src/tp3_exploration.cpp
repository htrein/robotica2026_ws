#include "phi_p3dx_mapping/exploration_node.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <cmath>

class Tp3ExplorationCpp : public ExplorationNode
{
public:
  Tp3ExplorationCpp() : ExplorationNode("tp3_exploration_cpp")
  {
    auto qos = rclcpp::QoS(10).transient_local();
    potential_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("potential_map", qos);
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&Tp3ExplorationCpp::odom_callback, this, std::placeholders::_1));
  }

protected:
  void on_map() override
  {
    if (!map_msg_) 
      return;

    int width = map_msg_->info.width;
    int height = map_msg_->info.height;

    //inicializa o campo potencial do tamanho do mapa com valor neutro
    if (potential_field.size() != (size_t)(width * height)) {
      potential_field.assign(width * height, 0.5); 
    }

    //células desconhecidas atraem o robô -> potencial baixo 
    //obstáculos repelem o robô -> potencial alto 
    for (int i = 0; i < width * height; ++i) {
      if (map_msg_->data[i] == -1) {
        potential_field[i] = 0.0;        
      } else if (map_msg_->data[i] > 60) {
        potential_field[i] = 1.0;       
      }
    }

    //fixa as bordas do mapa como desconhecidas 
    for (int x = 0; x < width; ++x) {
      potential_field[0 * width + x] = 0.0;                 
      potential_field[(height-1) * width + x] = 0.0;        
    }
    for (int y = 0; y < height; ++y) {
      potential_field[y * width + 0] = 0.0;                 
      potential_field[y * width + (width-1)] = 0.0;         
    }

    //Gauss-Seidel
    const int N = 150;
    for (int k = 0; k < N; ++k) {
      //percorre todas as células (excluindo as bordas)
      for (int y = 1; y < height - 1; ++y) {
        for (int x = 1; x < width - 1; ++x) {
          int idx = y * width + x;
          // Dirichlet
          if (map_msg_->data[idx] == -1 || map_msg_->data[idx] > 60)
            continue;
          //celula livre eh a média dos seus 4 vizinhos  
          potential_field[idx] = (potential_field[y * width + (x+1)] + potential_field[y * width + (x-1)] + potential_field[(y+1) * width + x] + potential_field[(y-1) * width + x]) / 4.0;
        }
      }
    }

    publish_potential_map();
  }

  void control_loop() override
  {
    if (!map_msg_ || potential_field.empty() || std::isnan(robot_x_))
      return;

    int width = map_msg_->info.width;
    int height = map_msg_->info.height;

    // converte a odometria para indices no Grid do mapa
    int rx = (int)((robot_x_ - map_msg_->info.origin.position.x) / map_msg_->info.resolution);
    int ry = (int)((robot_y_ - map_msg_->info.origin.position.y) / map_msg_->info.resolution);

    // impede o robô de ler memória fora do array
    if (rx < 0 || rx >= width || ry < 0 || ry >= height) {
      stop();
      return;
    }

    //calcula o gradiente 
    double grad_x; 
    double grad_y;

    if (rx == 0) {
      grad_x = potential_field[ry * width + (rx+1)] - potential_field[ry * width + rx];
    } else if (rx == width - 1) {
      grad_x = potential_field[ry * width + rx] - potential_field[ry * width + (rx-1)];
    } else {
      grad_x = (potential_field[ry * width + (rx+1)] - potential_field[ry * width + (rx-1)]) / 2.0;
    }

    if (ry == 0) {
      grad_y = potential_field[(ry+1) * width + rx] - potential_field[ry * width + rx];
    } else if (ry == height - 1) {
      grad_y = potential_field[ry * width + rx] - potential_field[(ry-1) * width + rx];
    } else {
      grad_y = (potential_field[(ry+1) * width + rx] - potential_field[(ry-1) * width + rx]) / 2.0;
    }

    double grad_mag = std::sqrt(grad_x * grad_x + grad_y * grad_y);

    // verifica se ainda existem fronteiras no mapa 
    bool unknown_exists = false;
    for (size_t i = 0; i < map_msg_->data.size(); ++i) {
      if (map_msg_->data[i] == -1) {
        unknown_exists = true;
        break;
      }
    }

    // se o campo estiver plano e não houver mais para onde explorar para o robô
    if (grad_mag < 1e-5 && !unknown_exists) {
      RCLCPP_INFO(this->get_logger(), "Exploração concluída");
      stop();
      return;
    }

    // calcula a direção de descida do gradiente
    double target_angle = std::atan2(-grad_y, -grad_x);
    publish_target_pose(robot_x_, robot_y_, target_angle, "map");

    // controle proporcional simples
    double angle_diff = target_angle - robot_theta_;
    // normaliza o angulo 
    while (angle_diff > M_PI) 
      angle_diff -= 2 * M_PI;
    while (angle_diff < -M_PI) 
      angle_diff += 2 * M_PI;

    // vel angular e limitação de segurança
    double w = std::clamp(2.0 * angle_diff, -0.8, 0.8);
    double v = 0.2; // v linear
    
    // se a curva for muito fechada, o robo gira no propio eixo 
    if (std::abs(angle_diff) > 0.5) {
      v = 0.0;  
    }
    publish_velocity(v, w);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr potential_pub_;

  std::vector<double> potential_field; // campo de potencial

  double robot_x_ = NAN;
  double robot_y_ = NAN;
  double robot_theta_ = NAN;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_theta_);
  }

  void publish_potential_map()
  {
    nav_msgs::msg::OccupancyGrid pot_msg;
    pot_msg.header = map_msg_->header;
    pot_msg.info = map_msg_->info;
    pot_msg.data.resize(potential_field.size());

    for (size_t i = 0; i < potential_field.size(); ++i) {
      if (map_msg_->data[i] == -1) {
        pot_msg.data[i] = -1; 
      } else {
        pot_msg.data[i] = static_cast<int8_t>(std::clamp(potential_field[i] * 100.0, 0.0, 100.0));
      }
    }
    potential_pub_->publish(pot_msg);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tp3ExplorationCpp>());
  rclcpp::shutdown();
  return 0;
}
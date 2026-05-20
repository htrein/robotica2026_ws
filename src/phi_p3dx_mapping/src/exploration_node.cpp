#include "phi_p3dx_mapping/exploration_node.hpp"
#include <algorithm>
#include <limits>
#include <cmath>

ExplorationNode::ExplorationNode(const std::string &node_name, double timer_period)
  : Node(node_name),
    laser_angle_min_(0.0), laser_angle_increment_(0.0),
    map_msg_(nullptr)
{
  // Publisher param enviar comandos de velocidade ao robô (ou simulador)
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  
  // Publisher para as orientações desejadas/alvos
  target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("target_pose", 10);

  // Subscriber de dados de varredura a laser (SensorDataQoS é melhor para sensores)
  auto sensor_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "laser_scan", sensor_qos,
    std::bind(&ExplorationNode::laser_callback, this, std::placeholders::_1));

  // Subscriber: mapa global
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "map", map_qos,
    std::bind(&ExplorationNode::map_callback, this, std::placeholders::_1));

  // Timer que controla o ciclo principal do nodo de forma constante
  timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&ExplorationNode::control_loop, this));

  RCLCPP_INFO(this->get_logger(), "[%s] inicializado.", node_name.c_str());
}

/**
 * @brief Callback chamado toda vez que um pacote de dados de laser chega.
 * 
 * Armazena as leituras em laser_ranges_ e aciona on_laser().
 * 
 * @param msg Mensagem LaserScan contendo a varredura atual.
 */
void ExplorationNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  laser_ranges_ = msg->ranges;
  laser_angle_min_ = msg->angle_min;
  laser_angle_increment_ = msg->angle_increment;
  on_laser(); // Chamada para a classe filha
}

/**
 * @brief Callback associado ao recebimento do Mapa Global (OccupancyGrid).
 * 
 * @param msg Mensagem contendo os dados do mapa recebido.
 */
void ExplorationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  map_msg_ = msg;
  on_map(); // Chamada para a classe filha
}

/**
 * @brief Retorna a distãncia medida pelo obstáculo mais próximo imediatamente à frente do robô.
 * 
 * Filtra as leituras dentro de um cone frontal configurável.
 * 
 * @param half_angle_deg Metade da abertura do cone em graus direcionado à frente do robô.
 * @return double Menor distância avaliada no cone, ou infinito caso não existam obstáculos ou dados indisponíveis.
 */
double ExplorationNode::get_front_distance(double half_angle_deg) const
{
  if (laser_ranges_.empty()) return std::numeric_limits<double>::infinity();

  double half_angle_rad = half_angle_deg * M_PI / 180.0;
  double min_dist = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < laser_ranges_.size(); ++i) {
    double angle = laser_angle_min_ + i * laser_angle_increment_;
    if (std::abs(angle) <= half_angle_rad) {
      double r = laser_ranges_[i];
      if (std::isnan(r) || std::isinf(r) || r < 0.05) {
        continue;
      }
      if (r < min_dist) {
        min_dist = r;
      }
    }
  }
  return min_dist;
}

/**
 * @brief Facilita o envio de comandos de velocidade (linear e angular) à base de rodas.
 * 
 * @param v Velocidade linear (frente) em m/s.
 * @param w Velocidade angular (rotação) em rad/s.
 */
void ExplorationNode::publish_velocity(double v, double w)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = v;
  twist.angular.z = w;
  cmd_vel_pub_->publish(twist);
}

/**
 * @brief Publica uma pose destino (ou vetor de movimento) para visualização.
 */
void ExplorationNode::publish_target_pose(double x, double y, double theta, const std::string &frame_id)
{
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = frame_id;
  
  pose_msg.pose.position.x = x;
  pose_msg.pose.position.y = y;
  pose_msg.pose.position.z = 0.0;
  
  pose_msg.pose.orientation.w = std::cos(theta / 2.0);
  pose_msg.pose.orientation.x = 0.0;
  pose_msg.pose.orientation.y = 0.0;
  pose_msg.pose.orientation.z = std::sin(theta / 2.0);
  
  target_pose_pub_->publish(pose_msg);
}

/**
 * @brief Envia um comando de parada imediata enviando o valor zero para as velocidades.
 */
void ExplorationNode::stop()
{
  publish_velocity(0.0, 0.0);
}

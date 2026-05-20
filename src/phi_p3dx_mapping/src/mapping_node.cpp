#include "phi_p3dx_mapping/mapping_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

MappingNode::MappingNode(const std::string &node_name, double timer_period)
  : Node(node_name),
    x_(0.0), y_(0.0), theta_(0.0),
    resolution_(0.05), origin_x_cells_(0), origin_y_cells_(0)
{
  (void)timer_period; // Ignorado no nó de mapeamento pois este é acionado por odometria
  
  // Configuração do Publisher para o mapa global com qualidade de serviço transient_local
  // Isso garante que nodos que conectarem atrasados ainda recebam o último mapa publicado.
  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.transient_local();
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);

  // Subscriber: odometria
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", 10,
    std::bind(&MappingNode::odom_callback, this, std::placeholders::_1));

  // Inicializa mapa 20x20m com resolução de 0.05m
  init_map(20.0, 20.0, 0.05);

  RCLCPP_INFO(this->get_logger(), "[%s] inicializado e aguardando odometria.", node_name.c_str());
}

/**
 * @brief Inicializa a grade do mapa com as dimensões específicas.
 * 
 * @param map_width_m Largura física do mapa em metros.
 * @param map_height_m Altura física do mapa em metros.
 * @param resolution Resolução (metros por célula).
 */
void MappingNode::init_map(double map_width_m, double map_height_m, double resolution)
{
  resolution_ = resolution;
  int width_cells = (int)(map_width_m / resolution_);
  int height_cells = (int)(map_height_m / resolution_);

  map_msg_.header.frame_id = "map";
  map_msg_.info.resolution = resolution_;
  map_msg_.info.width = width_cells;
  map_msg_.info.height = height_cells;
  
  // A origem do mapa fica no canto inferior esquerdo, então para que (0,0) seja o centro:
  map_msg_.info.origin.position.x = -map_width_m / 2.0;
  map_msg_.info.origin.position.y = -map_height_m / 2.0;
  map_msg_.info.origin.position.z = 0.0;
  map_msg_.info.origin.orientation.w = 1.0;

  origin_x_cells_ = width_cells / 2;
  origin_y_cells_ = height_cells / 2;

  // Preenche todo o mapa com células desconhecidas (valor -1)
  map_msg_.data.assign(width_cells * height_cells, -1);
}

/**
 * @brief Callback para receber atualizações do tópico de odometria (/odom).
 * 
 * Extrai a posição e converte a orientação (quaternio) para yaw (theta).
 * Após a atualização, chama o método virtual on_odom().
 * 
 * @param msg Mensagem recebida.
 */
void MappingNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;

  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, theta_);

  // Chamada para as classes filhas processarem os novos dados de odometria
  on_odom();
}

/**
 * @brief Converte coordenadas do mundo contínuo (metros) para índices da grade do mapa.
 * 
 * @param wx Coordenada X no mundo (metros).
 * @param wy Coordenada Y no mundo (metros).
 * @return std::tuple<int, int> Retorna a coluna (mx) e a linha (my) correspondente no grid.
 */
std::tuple<int, int> MappingNode::meters_to_cells(double wx, double wy) const
{
  int mx = (int)((wx - map_msg_.info.origin.position.x) / resolution_);
  int my = (int)((wy - map_msg_.info.origin.position.y) / resolution_);
  return {mx, my};
}

/**
 * @brief Publica a mensagem do mapa (OccupancyGrid) atualizada no tópico /map.
 */
void MappingNode::publish_map()
{
  map_msg_.header.stamp = this->now();
  map_pub_->publish(map_msg_);
}

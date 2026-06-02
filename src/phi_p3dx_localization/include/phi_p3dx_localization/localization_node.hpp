#ifndef PHI_P3DX_LOCALIZATION_NODE_HPP_
#define PHI_P3DX_LOCALIZATION_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/float32.hpp>
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <random>

/**
 * @brief Estrutura simples para representar uma partícula na localização.
 * 
 * Cada partícula armazena uma pose (x, y, theta) e um peso.
 * O peso será usado posteriormente para reamostragem no MCL completo.
 */
struct Particle
{
  double x;       ///< Posição X no frame "map"
  double y;       ///< Posição Y no frame "map"
  double theta;   ///< Orientação (radianos)
  double weight;  ///< Peso da partícula (inicialmente 1.0 / num_particles)
};

/**
 * @brief Modelo de odometria do Probabilistic Robotics.
 */
struct OdometryMotion {
  double rot1;
  double trans;
  double rot2;
};

/**
 * @class LocalizationNode
 * @brief Nó de localização simples baseado em filtro de partículas.
 * 
 * Este nó mantém um conjunto de partículas, publica para visualização em RViz,
 * e estima a pose como a média das partículas com covariância simples.
 * 
 * Este é um código didático base para que alunos implementem o MCL completo.
 * 
 * **Tópicos subscritos:**
 * - `/map` (nav_msgs/msg/OccupancyGrid): Mapa de ocupância para inicializar partículas
 * 
 * **Tópicos publicados:**
 * - `/particles` (geometry_msgs/msg/PoseArray): Visualização das partículas
 * - `/estimated_pose` (geometry_msgs/msg/PoseWithCovarianceStamped): Pose estimada com covariância
 * 
 * **Parâmetros:**
 * - `num_particles` (int): Número de partículas (default: 100)
 * - `publish_freq` (double): Frequência de publicação em Hz (default: 10.0)
 */
class LocalizationNode : public rclcpp::Node
{
public:
  /**
   * @brief Construtor do LocalizationNode.
   * 
   * @param node_name Nome do nó no ROS 2 (default: "localization_node")
   */
  explicit LocalizationNode(const std::string &node_name = "localization_node");
  virtual ~LocalizationNode() = default;

protected:
  //  Publishers 
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particles_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr estimated_pose_pub_;

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Variáveis de entrada da localização
  sensor_msgs::msg::LaserScan::SharedPtr z_t;
  OdometryMotion u_t; ///< Componentes de odometria rot1, trans, rot2 (Probabilistic Robotics)

  // Estado Interno de Odometria
  nav_msgs::msg::Odometry::SharedPtr current_odom_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_used_;
  bool first_odom_received_ = false;

  // Timer para publicação periódica
  rclcpp::TimerBase::SharedPtr publish_timer_;

  // Estado Interno
  std::vector<Particle> particles_;
  int num_particles_;
  std::string frame_id_;
  bool full_space_generation_;  /// Se true, gera partículas uniformemente em todo o espaço livre do mapa

  // Mapa atual
  nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
  bool map_received_ = false;

  // Geradores de números aleatórios
  std::mt19937 rng_;
  std::normal_distribution<double> normal_dist_;

  // Métodos principais

  /**
   * @brief Inicializa o conjunto de partículas.
   * 
   * Se um mapa foi recebido e `full_space_generation` é true, 
   * inicializa as partículas uniformemente
   * em todas as células livres do mapa.
   * 
   * Caso contrário, inicializa em duas seções do mapa com orientação zero.
   */
  void initialize_particles();

  /**
   * @brief Callback para receber atualização do mapa.
   * 
   * Processa o mapa recebido e marca que o mapa está disponível para
   * inicialização de partículas por tentativa de geração aleatória.
   * 
   * @param msg Mensagem OccupancyGrid recebida
   */
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  /**
   * @brief Calcula a pose estimada como a média das partículas.
   * 
   * Para a orientação, usa média circular:
   * - Calcula mean_sin = média(sin(theta_i))
   * - Calcula mean_cos = média(cos(theta_i))
   * - Resultado: atan2(mean_sin, mean_cos)
   * 
   * @param mean_x Referência para receber a média de X
   * @param mean_y Referência para receber a média de Y
   * @param mean_theta Referência para receber a média de theta (circular)
   */
  void calculate_mean_pose(double &mean_x, double &mean_y, double &mean_theta) const;

  /**
   * @brief Calcula a covariância simples das partículas.
   * 
   * Calcula as variâncias de x, y e theta e retorna uma matriz
   * 6x6 com os valores diagonais preenchidos (outras posições = 0).
   * 
   * @param mean_x Média de X (calculada previamente)
   * @param mean_y Média de Y (calculada previamente)
   * @param mean_theta Média de theta (calculada previamente)
   * @return Matriz 6x6 com covariância (36 elementos)
   */
  std::array<double, 36> calculate_covariance(
    double mean_x, double mean_y, double mean_theta) const;

  /**
   * @brief Publica o conjunto de partículas como PoseArray.
   * 
   * Usado para visualização em RViz.
   */
  void publish_particles();

  /**
   * @brief Publica a pose estimada com covariância.
   * 
   * Calcula média, covariância e publica em `/estimated_pose`.
   */
  void publish_estimated_pose();

  /**
   * @brief Callbacks para sensor laser e odometria
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

  /**
   * @brief Função auxiliar para extrair yaw de quaternion
   */
  double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) const;

  /**
   * @brief Loop principal executado pelo timer.
   * 
   * Chamado a cada 1/publish_freq segundos.
   * Publica partículas e pose estimada.
   */
  void on_timer();

  /**
   * @brief Método virtual para que subclasses possam implementar atualização.
   * 
   * Será sobrescrito em classes filhas (ex: exemplo com rotação).
   * De momento, não faz nada.
   */
  virtual void update_particles() {}

private:
  // Parâmetros
  double publish_freq_;
};

#endif  // PHI_P3DX_LOCALIZATION_NODE_HPP_

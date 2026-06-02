#include "phi_p3dx_localization/localization_node.hpp"
#include <cmath>
#include <numeric>

LocalizationNode::LocalizationNode(const std::string &node_name)
  : Node(node_name),
    num_particles_(1000),
    frame_id_("map"),
    rng_(std::random_device{}()),
    normal_dist_(0.0, 1.0)
{
  // Declarar e obter parâmetros
  this->declare_parameter<int>("num_particles", 1000);
  this->declare_parameter<double>("publish_freq", 10.0);
  this->declare_parameter<std::string>("frame_id", "map");
  this->declare_parameter<bool>("full_space_generation", true);

  num_particles_ = this->get_parameter("num_particles").as_int();
  publish_freq_ = this->get_parameter("publish_freq").as_double();
  frame_id_ = this->get_parameter("frame_id").as_string();
  full_space_generation_ = this->get_parameter("full_space_generation").as_bool();

  // Cria publishers
  particles_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "particles", rclcpp::QoS(10).transient_local());
  
  estimated_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "estimated_pose", rclcpp::QoS(10));

  // Cria subscriber para o mapa
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(10).transient_local(),
    std::bind(&LocalizationNode::map_callback, this, std::placeholders::_1));

  // Cria subscribers para scan e odometria
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "laser_scan", rclcpp::SensorDataQoS(),
    std::bind(&LocalizationNode::scan_callback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::QoS(10),
    std::bind(&LocalizationNode::odom_callback, this, std::placeholders::_1));

  // Inicializa u_t
  u_t.rot1 = 0.0;
  u_t.trans = 0.0;
  u_t.rot2 = 0.0;

  // Timer para publicação (ex: 10 Hz = 0.1 s)
  double timer_period = 1.0 / publish_freq_;
  publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(timer_period),
    std::bind(&LocalizationNode::on_timer, this));

  // Inicializa partículas
  initialize_particles();

  if(full_space_generation_){
    RCLCPP_INFO(this->get_logger(), 
      "[%s] inicializado com %d partículas (geração em todo espaço livre). Freq: %.1f Hz. Frame: %s",
      node_name.c_str(), num_particles_, publish_freq_, frame_id_.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), 
      "[%s] inicializado com %d partículas (geração em duas seções). Freq: %.1f Hz. Frame: %s",
      node_name.c_str(), num_particles_, publish_freq_, frame_id_.c_str());
  }
}

void LocalizationNode::initialize_particles()
{
  particles_.clear();
  particles_.resize(num_particles_);

  double weight = 1.0 / num_particles_;

  if (map_received_ && current_map_) {
    // Referências locais para facilitar acesso
    const double resolution = current_map_->info.resolution;
    const double origin_x = current_map_->info.origin.position.x;
    const double origin_y = current_map_->info.origin.position.y;
    const int width = static_cast<int>(current_map_->info.width);
    const int height = static_cast<int>(current_map_->info.height);

    // Limites do mapa em coordenadas do mundo
    const double min_x = origin_x;
    const double min_y = origin_y;
    const double max_x = origin_x + width * resolution;
    const double max_y = origin_y + height * resolution;

    // Geradores aleatórios para posição e orientação
    std::uniform_real_distribution<double> x_dist(min_x, max_x);
    std::uniform_real_distribution<double> y_dist(min_y, max_y);

    std::uniform_real_distribution<double> x_dist1(min_x, 0);
    std::uniform_real_distribution<double> y_dist1(min_y, 0);
    std::uniform_real_distribution<double> x_dist2(0, max_x);
    std::uniform_real_distribution<double> y_dist2(0, max_y);
    
    std::uniform_real_distribution<double> theta_dist(-M_PI, M_PI);

    const int FREE_THRESHOLD = 50;

    double world_x, world_y, theta;
    int i = 0;
    while(i < num_particles_) {
      bool valid = false;

      // Gera pose aleatória
      if(full_space_generation_){
        world_x = x_dist(rng_);
        world_y = y_dist(rng_);
        theta = theta_dist(rng_);
      } else {
        if(i%2 == 0){
          world_x = x_dist1(rng_);
          world_y = y_dist1(rng_);
        } else {
          world_x = x_dist2(rng_);
          world_y = y_dist2(rng_);
        }
        theta = 0;
      }

      // Converte para coordenadas de célula
      int col = static_cast<int>(std::floor((world_x - origin_x) / resolution));
      int row = static_cast<int>(std::floor((world_y - origin_y) / resolution));

      // Verifica no mapa se a célula é livre
      if (col >=0  && col < width && row >= 0 && row < height) {
        int idx = row * width + col;
        if (idx >= 0 && idx < static_cast<int>(current_map_->data.size())) {
          int8_t cell_value = current_map_->data[idx];
          if (cell_value >= 0 && cell_value < FREE_THRESHOLD) {
            valid = true;
          }
        }
      }

      if(valid){
        // Adiciona partícula no conjunto
        particles_[i].x = world_x;
        particles_[i].y = world_y;
        particles_[i].theta = theta;
        particles_[i].weight = weight;
        i++;
      }
    }

    RCLCPP_INFO(this->get_logger(),
      "Partículas inicializadas no mapa (%dx%d)", width, height);
  }
}

void LocalizationNode::calculate_mean_pose(double &mean_x, double &mean_y, double &mean_theta) const
{
  if (particles_.empty()) {
    mean_x = 0.0;
    mean_y = 0.0;
    mean_theta = 0.0;
    return;
  }

  // Média simples de X e Y
  mean_x = 0.0;
  mean_y = 0.0;
  double sum_sin = 0.0;
  double sum_cos = 0.0;

  for (const auto &p : particles_) {
    mean_x += p.weight*p.x;
    mean_y += p.weight*p.y;
    sum_sin += p.weight * std::sin(p.theta);
    sum_cos += p.weight * std::cos(p.theta);
  }
  // Média circular para theta
  mean_theta = std::atan2(sum_sin, sum_cos);
}

std::array<double, 36> LocalizationNode::calculate_covariance(
  double mean_x, double mean_y, double mean_theta) const
{
  std::array<double, 36> cov;
  cov.fill(0.0);

  if (particles_.empty()) {
    return cov;
  }

  // Variâncias
  double var_xx = 0.0;
  double var_xy = 0.0;
  double var_xtheta = 0.0;
  double var_yy = 0.0;
  double var_yx = 0.0;
  double var_ytheta = 0.0;
  double var_thetax = 0.0;
  double var_thetay = 0.0;
  double var_thetatheta = 0.0;

  for (const auto &p : particles_) {
    double dx = p.x - mean_x;
    double dy = p.y - mean_y;
    
    // Para theta, usar diferença angular
    double dtheta = p.theta - mean_theta;
    // Normalizar para [-π, π]
    while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
    while (dtheta < -M_PI) dtheta += 2.0 * M_PI;

    // Computar variâncias e covariâncias para a partícula
    var_xx += p.weight * dx * dx;
    var_xy += p.weight * dx * dy;
    var_xtheta += p.weight * dx * dtheta;
    var_yy += p.weight * dy * dy;
    var_yx += p.weight * dy * dx;
    var_ytheta += p.weight * dy * dtheta;
    var_thetax += p.weight * dtheta * dx;
    var_thetay += p.weight * dtheta * dy;
    var_thetatheta += p.weight * dtheta * dtheta;
  }

  // Preencher matriz 6x6
  cov[0] = var_xx;           // (0,0) - variância de X
  cov[1] = var_xy;           // (0,1) - covariância XY
  cov[5] = var_xtheta;       // (0,5) - covariância X-theta
  cov[6] = var_yx;           // (1,0) - covariância YX (igual a XY)
  cov[7] = var_yy;           // (1,1) - variância de Y
  cov[11] = var_ytheta;      // (1,5) - covariância Y-theta
  cov[30] = var_thetax;      // (5,0) - covariância theta-X (igual a X-theta)
  cov[31] = var_thetay;      // (5,1) - covariância theta-Y (igual a Y-theta)
  cov[35] = var_thetatheta;  // (5,5) - variância de theta

  return cov;
}

void LocalizationNode::publish_particles()
{
  if (particles_.empty()) {
    return;
  }

  auto msg = std::make_shared<geometry_msgs::msg::PoseArray>();
  msg->header.frame_id = frame_id_;
  msg->header.stamp = this->now();
  msg->poses.resize(particles_.size());

  for (size_t i = 0; i < particles_.size(); ++i) {
    const auto &p = particles_[i];
    msg->poses[i].position.x = p.x;
    msg->poses[i].position.y = p.y;
    msg->poses[i].position.z = 0.0;

    // Converter theta para quaternion (rotação (yaw) apenas em Z)
    double half_theta = p.theta / 2.0;
    msg->poses[i].orientation.x = 0.0;
    msg->poses[i].orientation.y = 0.0;
    msg->poses[i].orientation.z = std::sin(half_theta);
    msg->poses[i].orientation.w = std::cos(half_theta);
  }

  particles_pub_->publish(*msg);
}

void LocalizationNode::publish_estimated_pose()
{
  double mean_x, mean_y, mean_theta;
  calculate_mean_pose(mean_x, mean_y, mean_theta);

  auto cov = calculate_covariance(mean_x, mean_y, mean_theta);

  auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  msg->header.frame_id = frame_id_;
  msg->header.stamp = this->now();

  // Posição
  msg->pose.pose.position.x = mean_x;
  msg->pose.pose.position.y = mean_y;
  msg->pose.pose.position.z = 0.0;

  // Orientação (quaternion da média theta)
  double half_theta = mean_theta / 2.0;
  msg->pose.pose.orientation.x = 0.0;
  msg->pose.pose.orientation.y = 0.0;
  msg->pose.pose.orientation.z = std::sin(half_theta);
  msg->pose.pose.orientation.w = std::cos(half_theta);

  // Covariância
  msg->pose.covariance = cov;

  estimated_pose_pub_->publish(*msg);
}

void LocalizationNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  z_t = msg;
}

void LocalizationNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_odom_ = msg;
  if (!first_odom_received_) {
    last_odom_used_ = msg;
    first_odom_received_ = true;
  }
}

double LocalizationNode::get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q) const
{
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

void LocalizationNode::on_timer()
{
  // Calcular deslocamento u_t baseado na odometria
  if (first_odom_received_ && current_odom_ && last_odom_used_) {
    double last_x = last_odom_used_->pose.pose.position.x;
    double last_y = last_odom_used_->pose.pose.position.y;
    double last_yaw = get_yaw_from_quaternion(last_odom_used_->pose.pose.orientation);

    double curr_x = current_odom_->pose.pose.position.x;
    double curr_y = current_odom_->pose.pose.position.y;
    double curr_yaw = get_yaw_from_quaternion(current_odom_->pose.pose.orientation);

    double dx = curr_x - last_x;
    double dy = curr_y - last_y;

    double delta_yaw = curr_yaw - last_yaw;
    while (delta_yaw > M_PI) delta_yaw -= 2.0 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

    double trans = std::hypot(dx, dy);
    double rot1 = 0.0;
    
    if (trans > 1e-6) {
      rot1 = std::atan2(dy, dx) - last_yaw;
      while (rot1 > M_PI) rot1 -= 2.0 * M_PI;
      while (rot1 < -M_PI) rot1 += 2.0 * M_PI;
    }
    
    double rot2 = delta_yaw - rot1;
    while (rot2 > M_PI) rot2 -= 2.0 * M_PI;
    while (rot2 < -M_PI) rot2 += 2.0 * M_PI;

    u_t.rot1 = rot1;
    u_t.trans = trans;
    u_t.rot2 = rot2;

    last_odom_used_ = current_odom_;
  } else {
    u_t.rot1 = 0.0;
    u_t.trans = 0.0;
    u_t.rot2 = 0.0;
  }

  // Permite que subclasses implementem atualização de partículas
  update_particles();

  // Publica partículas e pose estimada
  publish_particles();
  publish_estimated_pose();
}

void LocalizationNode::map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  map_received_ = true;
  
  // Reinicializa partículas agora que temos o mapa
  initialize_particles();
  
  RCLCPP_INFO(this->get_logger(), 
    "Mapa recebido: %u x %u, resolução: %.3f m", 
    msg->info.width, msg->info.height, msg->info.resolution);
}

#!/usr/bin/env python3
"""
@brief Classe base para localização usando filtro de partículas (MCL).

Esta classe implementa a estrutura básica de um filtro de partículas para localização
de robôs móveis. Mantém um conjunto de partículas, calcula média e covariância,
e publica para visualização no RViz.

**Tópicos publicados:**
- `/particles` (geometry_msgs/msg/PoseArray): Visualização das partículas
- `/estimated_pose` (geometry_msgs/msg/PoseWithCovarianceStamped): Pose estimada com covariância

**Tópicos subscritos:**
- `/map` (nav_msgs/msg/OccupancyGrid): Mapa para inicializar partículas

**Parâmetros ROS:**
- `num_particles` (int, default: 1000)
- `publish_freq` (float, default: 10.0)
- `frame_id` (str, default: 'map')
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseArray, PoseWithCovarianceStamped, Pose
from nav_msgs.msg import OccupancyGrid
import math
import random
import numpy as np


class Particle:
    """Estrutura simples para representar uma partícula."""
    def __init__(self, x=0.0, y=0.0, theta=0.0, weight=1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class LocalizationNode(Node):
    """Classe base para localização usando filtro de partículas."""

    def __init__(self, node_name='localization_node'):
        super().__init__(node_name)

        # Parâmetros
        self.declare_parameter('num_particles', 1000)
        self.declare_parameter('publish_freq', 10.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('full_space_generation', True)

        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value
        self.publish_freq = self.get_parameter('publish_freq').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.full_space_generation = self.get_parameter('full_space_generation').get_parameter_value().bool_value

        # Estado interno
        self.particles = []
        self.current_map = None
        self.map_received = False

        # Publishers
        self.particles_pub = self.create_publisher(PoseArray, 'particles', 10)
        self.estimated_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'estimated_pose', 10)

        # Subscribers
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        # Timer para publicação
        timer_period = 1.0 / self.publish_freq
        self.publish_timer = self.create_timer(timer_period, self.on_timer)

        # Inicializa partículas
        self.initialize_particles()

        if self.full_space_generation:
            self.get_logger().info(
                f'[{node_name}] inicializado com {self.num_particles} partículas (geração em todo espaço livre). '
                f'Freq: {self.publish_freq:.1f} Hz. Frame: {self.frame_id}'
            )
        else:
            self.get_logger().info(
                f'[{node_name}] inicializado com {self.num_particles} partículas (geração em duas seções). '
                f'Freq: {self.publish_freq:.1f} Hz. Frame: {self.frame_id}'
            )

    def initialize_particles(self):

        self.particles = []

        if self.map_received is False and self.current_map is None:
            self.get_logger().info("Map not received yet.")
            return
        else:
            self.get_logger().info("Generating particles.")

        """Inicializa partículas no mapa conforme full_space_generation."""
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        width = self.current_map.info.width
        height = self.current_map.info.height

        min_x = origin_x
        min_y = origin_y
        max_x = origin_x + width * resolution
        max_y = origin_y + height * resolution

        free_threshold = 50

        i = 0
        while i < self.num_particles:
            valid = False

            # Gera pose aleatória baseado em full_space_generation
            if self.full_space_generation:
                # Todo espaço livre
                world_x = random.uniform(min_x, max_x)
                world_y = random.uniform(min_y, max_y)
                theta = random.uniform(-math.pi, math.pi)
            else:
                # Duas seções: quadrante esquerdo e direito
                if i % 2 == 0:
                    # Quadrante esquerdo (min_x a 0, min_y a 0)
                    world_x = random.uniform(min_x, 0.0)
                    world_y = random.uniform(min_y, 0.0)
                else:
                    # Quadrante direito (0 a max_x, 0 a max_y)
                    world_x = random.uniform(0.0, max_x)
                    world_y = random.uniform(0.0, max_y)
                theta = 0.0  # Sempre zero nas duas seções

            # Converte para coordenadas de célula
            col = int(math.floor((world_x - origin_x) / resolution))
            row = int(math.floor((world_y - origin_y) / resolution))

            # Verifica se a célula é livre
            if col >= 0 and col < width and row >= 0 and row < height:
                idx = row * width + col
                if idx >= 0 and idx < len(self.current_map.data):
                    cell_value = self.current_map.data[idx]
                    if cell_value >= 0 and cell_value < free_threshold:
                        valid = True

            if valid:
                particle = Particle(world_x, world_y, theta, 1.0 / self.num_particles)
                self.particles.append(particle)
                i += 1

        self.get_logger().info(
            f'{len(self.particles)} Partículas inicializadas no mapa ({width}x{height})'
        )

    def calculate_mean_pose(self):
        """Calcula a pose estimada como a média das partículas."""
        if not self.particles:
            return 0.0, 0.0, 0.0

        # Média simples de X e Y
        mean_x = sum(p.weight * p.x for p in self.particles)
        mean_y = sum(p.weight * p.y for p in self.particles)

        # Média circular para theta
        sum_sin = sum(p.weight * math.sin(p.theta) for p in self.particles)
        sum_cos = sum(p.weight * math.cos(p.theta) for p in self.particles)
        mean_theta = math.atan2(sum_sin, sum_cos)

        return mean_x, mean_y, mean_theta

    def calculate_covariance(self, mean_x, mean_y, mean_theta):
        """Calcula a covariância simples das partículas."""
        if not self.particles:
            return [0.0] * 36

        cov = [0.0] * 36

        # Variâncias e covariancias
        var_xx = 0.0
        var_xy = 0.0
        var_xtheta = 0.0
        var_yy = 0.0
        var_yx = 0.0
        var_ytheta = 0.0
        var_thetax = 0.0
        var_thetay = 0.0
        var_thetatheta = 0.0

        for p in self.particles:
            dx = p.x - mean_x
            dy = p.y - mean_y
            dtheta = p.theta - mean_theta
            # Normalizar dtheta para [-π, π]
            while dtheta > math.pi:
                dtheta -= 2.0 * math.pi
            while dtheta < -math.pi:
                dtheta += 2.0 * math.pi

            # Computar variâncias e covariâncias para a partícula
            var_xx += p.weight * dx * dx
            var_xy += p.weight * dx * dy
            var_xtheta += p.weight * dx * dtheta
            var_yy += p.weight * dy * dy
            var_yx += p.weight * dy * dx
            var_ytheta += p.weight * dy * dtheta
            var_thetax += p.weight * dtheta * dx
            var_thetay += p.weight * dtheta * dy
            var_thetatheta += p.weight * dtheta * dtheta

            # Preencher matriz 6x6
            cov[0] = var_xx           # (0,0) - variância de X
            cov[1] = var_xy           # (0,1) - covariância XY
            cov[5] = var_xtheta       # (0,5) - covariância X-theta
            cov[6] = var_yx;          # (1,0) - covariância YX (igual a XY)
            cov[7] = var_yy;          # (1,1) - variância de Y
            cov[11] = var_ytheta      # (1,5) - covariância Y-theta
            cov[30] = var_thetax      # (5,0) - covariância theta-X (igual a X-theta)
            cov[31] = var_thetay      # (5,1) - covariância theta-Y (igual a Y-theta)
            cov[35] = var_thetatheta  # (5,5) - variância de theta

        return cov

    def publish_particles(self):
        """Publica o conjunto de partículas como PoseArray."""
        if not self.particles:
            self.get_logger().info("Empty particles.")
            return

        msg = PoseArray()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poses = []

        for p in self.particles:
            pose = Pose()
            pose.position.x = p.x
            pose.position.y = p.y
            pose.position.z = 0.0

            # Converter theta para quaternion
            half_theta = p.theta / 2.0
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = math.sin(half_theta)
            pose.orientation.w = math.cos(half_theta)

            msg.poses.append(pose)

        self.particles_pub.publish(msg)

    def publish_estimated_pose(self):
        """Publica a pose estimada com covariância."""
        mean_x, mean_y, mean_theta = self.calculate_mean_pose()
        cov = self.calculate_covariance(mean_x, mean_y, mean_theta)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        # Posição
        msg.pose.pose.position.x = mean_x
        msg.pose.pose.position.y = mean_y
        msg.pose.pose.position.z = 0.0

        # Orientação
        half_theta = mean_theta / 2.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half_theta)
        msg.pose.pose.orientation.w = math.cos(half_theta)

        # Covariância
        msg.pose.covariance = cov

        self.estimated_pose_pub.publish(msg)

    def map_callback(self, msg):
        """Callback para receber atualização do mapa."""
        self.current_map = msg
        self.map_received = True

        # Reinicializa partículas agora que temos o mapa
        self.initialize_particles()

        self.get_logger().info(
            f'Mapa recebido: {msg.info.width}x{msg.info.height}, '
            f'resolução: {msg.info.resolution:.3f} m'
        )

    def on_timer(self):
        """Loop principal executado pelo timer."""
        # Permite que subclasses implementem atualização de partículas
        self.update_particles()

        # Publica partículas e pose estimada
        self.publish_particles()
        self.publish_estimated_pose()

    def update_particles(self):
        """Método virtual para que subclasses possam implementar atualização."""
        pass

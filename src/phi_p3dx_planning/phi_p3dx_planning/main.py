#!/usr/bin/env python3
"""
Módulo principal para planejamento autônomo em ROS2.

Este módulo define a classe PlanningNode, uma classe base para implementar
algoritmos de planejamento em robôs móveis. Ela gerencia a leitura de
sensores (odometria e laser), mapa global, recebimento de objetivos (goals) do RViz,
e publicação de comandos de velocidade.

Exemplo de uso:
    class MyPlanner(PlanningNode):
        def on_goal(self):
            self.get_logger().info('Novo objetivo recebido!')

        def _control_loop(self):
            if self.has_goal():
                # Lógica de planejamento aqui
                pass
"""

import math
import numpy as np

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """
    Converte um quaternion para ângulo yaw (rotação em Z).

    Args:
        x, y, z, w: Componentes do quaternion.

    Returns:
        Ângulo yaw em radianos.
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class PlanningNode(Node):
    """
    Classe base para nós de planejamento em ROS2.

    Gerencia odometria, dados do laser, objetivos (goals), mapa global e comandos de velocidade.
    Deve ser herdada para implementar algoritmos específicos de planejamento.

    Atributos:
        x, y, theta: Posição e orientação atual do robô.
        laser_ranges: Array com distâncias do laser (inf para inválidos).
        goal: Tupla (x, y) do objetivo atual, ou None.
        map_msg: O grid de ocupação mais recente (OccupancyGrid), ou None.
    """

    def __init__(self, node_name: str = 'planning_node',
                 timer_period: float = 0.05):
        """
        Inicializa o nó de planejamento.

        Args:
            node_name: Nome do nó ROS2.
            timer_period: Período do loop de controle em segundos (padrão 20 Hz).
        """
        super().__init__(node_name)

        # Estado do robô (odometria)
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        # Dados do laser
        self.laser_ranges: np.ndarray = np.array([])
        self.laser_angle_min: float = 0.0
        self.laser_angle_max: float = 0.0
        self.laser_angle_increment: float = 0.0

        # Dados do sonar
        self.sonar_ranges: np.ndarray = np.array([])
        self.sonar_angles: np.ndarray = np.array([])

        # Objetivo de planejamento (definido pelo RViz)
        self.goal: tuple | None = None
        self.goal_theta: float = 0.0

        # Mapa global / grid de planejamento
        self.map_msg: OccupancyGrid | None = None

        # Publisher para comandos de velocidade
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher para visualizar as arvores de exploracao/planejamento
        self.marker_pub = self.create_publisher(MarkerArray, 'tree_markers', 10)

        # Configuracao padrao dos markers de nos (esferas)
        self.nodes_marker = Marker()
        self.nodes_marker.header.frame_id = 'map'
        self.nodes_marker.ns = 'tree_nodes'
        self.nodes_marker.id = 0
        self.nodes_marker.type = Marker.SPHERE_LIST
        self.nodes_marker.action = Marker.ADD
        self.nodes_marker.pose.orientation.w = 1.0
        self.nodes_marker.scale.x = 0.1
        self.nodes_marker.scale.y = 0.1
        self.nodes_marker.scale.z = 0.1
        self.nodes_marker.color.r = 1.0
        self.nodes_marker.color.g = 0.0
        self.nodes_marker.color.b = 1.0
        self.nodes_marker.color.a = 1.0

        # Configuracao padrao dos markers de arestas (linhas)
        self.edges_marker = Marker()
        self.edges_marker.header.frame_id = 'map'
        self.edges_marker.ns = 'tree_edges'
        self.edges_marker.id = 1
        self.edges_marker.type = Marker.LINE_LIST
        self.edges_marker.action = Marker.ADD
        self.edges_marker.pose.orientation.w = 1.0
        self.edges_marker.scale.x = 0.02  # Espessura da linha
        self.edges_marker.color.r = 0.5
        self.edges_marker.color.g = 0.0
        self.edges_marker.color.b = 0.5
        self.edges_marker.color.a = 1.0

        # Subscribers para sensores e objetivos
        self.create_subscription(
            Odometry, '/odom', self._cb_odom, 10)

        self.create_subscription(
            LaserScan, '/laser_scan', self._cb_laser,
            qos_profile=qos_profile_sensor_data)

        self.create_subscription(
            PointCloud2, '/sonar_cloud', self._cb_sonar,
            qos_profile=qos_profile_sensor_data)

        # /goal_pose é publicado pelo botão "2D Nav Goal" do RViz2
        self.create_subscription(
            PoseStamped, '/goal_pose', self._cb_goal, 10)

        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._cb_map, map_qos)

        # Timer para executar o loop de controle periodicamente
        self.create_timer(timer_period, self._control_loop)

        self.get_logger().info(
            f'[{node_name}] iniciado — '
            'use "2D Nav Goal" no RViz para enviar um goal.'
        )

    def _cb_odom(self, msg: Odometry) -> None:
        """
        Callback para mensagens de odometria (/odom).

        Atualiza a posição e orientação do robô e chama o hook on_odom().

        Args:
            msg: Mensagem Odometry com pose atual.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.on_odom()

    def _cb_laser(self, msg: LaserScan) -> None:
        """
        Callback para mensagens do laser (/laser_scan).

        Processa os ranges, filtrando valores inválidos, e chama o hook on_laser().

        Args:
            msg: Mensagem LaserScan com dados do sensor.
        """
        self.laser_ranges = np.array([
            r if msg.range_min <= r <= msg.range_max else float('inf')
            for r in msg.ranges
        ], dtype=float)
        self.laser_angle_min = msg.angle_min
        self.laser_angle_max = msg.angle_max
        self.laser_angle_increment = msg.angle_increment
        self.on_laser()

    def print_sonar_pairs(self):
        if self.sonar_ranges is None or len(self.sonar_ranges) == 0:
            return

        s = ""
        for r, a in zip(self.sonar_ranges, self.sonar_angles):
            s += f"({a:.0f}°,{r:.2f}) "

        self.get_logger().info(s)

    def _cb_sonar(self, msg: PointCloud2) -> None:
        """
        Callback para mensagens de sonar (PointCloud2).

        Extrai pontos (x,y,z), calcula distância e ângulo.
        """

        ranges = []
        angles = []

        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p

            dist = math.sqrt(x*x + y*y)
            angle = math.degrees(math.atan2(y, x))

            ranges.append(dist)
            angles.append(angle)

        self.sonar_ranges = np.array(ranges, dtype=float)
        self.sonar_angles = np.array(angles, dtype=float)

        # self.print_sonar_pairs()

        self.on_sonar()

    def _cb_goal(self, msg: PoseStamped) -> None:
        """
        Callback para objetivos de planejamento (/goal_pose).

        Define o novo objetivo e chama o hook on_goal().

        Args:
            msg: Mensagem PoseStamped com o objetivo.
        """
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        q = msg.pose.orientation
        self.goal_theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f'Novo goal → x={self.goal[0]:.2f}  '
            f'y={self.goal[1]:.2f}  θ={math.degrees(self.goal_theta):.1f}°'
        )
        self.on_goal()

    def _cb_map(self, msg: OccupancyGrid) -> None:
        """
        Callback para o mapa (/map).

        Atualiza o mapa salvo e chama o hook on_map().
        """
        self.map_msg = msg
        self.on_map()

    def on_odom(self) -> None:
        """
        Hook chamado após receber odometria.

        Sobrescreva em subclasses para reagir a mudanças de pose.
        """
        pass

    def on_laser(self) -> None:
        """
        Hook chamado após receber dados do laser.

        Sobrescreva em subclasses para processar leituras do sensor.
        """
        pass

    def on_sonar(self) -> None:
        """
        Hook chamado após receber dados do sonar.

        Sobrescreva em subclasses para processar leituras do sonar.
        """
        pass

    def on_goal(self) -> None:
        """
        Hook chamado após receber um novo objetivo.

        Sobrescreva em subclasses para iniciar planejamento.
        """
        pass

    def on_map(self) -> None:
        """
        Hook chamado após receber o mapa pela primeira vez ou quando for atualizado.
        """
        pass

    def get_pose(self) -> tuple[float, float, float]:
        """
        Retorna a pose atual do robô.

        Returns:
            Tupla (x, y, theta) com posição e orientação.
        """
        return self.x, self.y, self.theta

    def has_laser_data(self) -> bool:
        """
        Verifica se há dados válidos do laser.

        Returns:
            True se o array de ranges não estiver vazio.
        """
        return len(self.laser_ranges) > 0

    def has_sonar_data(self) -> bool:
        """
        Verifica se há dados válidos do sonar.

        Returns:
            True se o array de ranges não estiver sonar.
        """
        return len(self.sonar_ranges) > 0

    def get_region_distance(self, idx_start: int, idx_end: int) -> float:
        """
        Calcula a distância mínima em uma região específica do scan laser.

        Args:
            idx_start: Índice inicial da região.
            idx_end: Índice final da região.

        Returns:
            Distância mínima na região, ou inf se inválida.
        """
        if not self.has_laser_data():
            return float('inf')
        region = self.laser_ranges[idx_start: idx_end + 1]
        return float(np.min(region)) if len(region) > 0 else float('inf')

    def get_front_distance(self, half_angle_deg: float = 15.0) -> float:
        """
        Calcula a distância mínima na frente do robô.

        Args:
            half_angle_deg: Metade do ângulo frontal em graus (padrão 15°).

        Returns:
            Distância mínima na frente, ou inf se sem dados.
        """
        if not self.has_laser_data():
            return float('inf')
        n      = len(self.laser_ranges)
        center = n // 2
        half   = int(math.radians(half_angle_deg)
                     / max(self.laser_angle_increment, 1e-9))
        return self.get_region_distance(
            max(0, center - half), min(n - 1, center + half)
        )

    def has_goal(self) -> bool:
        """
        Verifica se há um objetivo ativo.

        Returns:
            True se goal não for None.
        """
        return self.goal is not None

    def clear_goal(self) -> None:
        """
        Remove o objetivo atual (e.g., quando atingido).
        """
        self.goal = None
        self.get_logger().info('Objetivo atingido.')

    def distance_to_goal(self) -> float:
        """
        Calcula a distância euclidiana até o objetivo.

        Returns:
            Distância até o goal, ou inf se sem goal.
        """
        if self.goal is None:
            return float('inf')
        return math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)

    def angle_to_goal(self) -> float:
        """
        Calcula o ângulo relativo até o objetivo (erro de orientação).

        Returns:
            Ângulo em radianos (-pi a pi), ou 0 se sem goal.
        """
        if self.goal is None:
            return 0.0
        desired = math.atan2(self.goal[1] - self.y, self.goal[0] - self.x)
        err = desired - self.theta
        return math.atan2(math.sin(err), math.cos(err))

    def publish_velocity(self, v: float, w: float) -> None:
        """
        Publica comando de velocidade no tópico /cmd_vel.

        Args:
            v: Velocidade linear (m/s).
            w: Velocidade angular (rad/s).
        """
        twist = Twist()
        twist.linear.x  = float(v)
        twist.angular.z = float(w)
        self.pub_cmd.publish(twist)

    def stop(self) -> None:
        """
        Para o robô imediatamente.
        """
        self.publish_velocity(0.0, 0.0)

    def _control_loop(self) -> None:
        """
        Loop de controle executado periodicamente (padrão 20 Hz).

        Sobrescreva em subclasses para implementar a lógica de planejamento.
        """
        pass
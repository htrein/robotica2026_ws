import math
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid


class ExplorationNode(Node):
    """
    @brief Classe base para nós de exploração do ambiente.
    
    Assina as mensagens de laser e do mapa, além de publicar para cmd_vel.
    """
    def __init__(self, node_name: str = 'exploration_node', timer_period: float = 0.1):
        """
        @brief Construtor da classe ExplorationNode.
        
        @param node_name Nome com qual o nó será registrado no ROS 2.
        @param timer_period Período do loop principal em segundos.
        """
        super().__init__(node_name)

        # Dados do Laser
        self.laser_ranges = []
        self.laser_angle_min = 0.0
        self.laser_angle_increment = 0.0

        # Mapa
        self.map_msg = None

        # Publisher param enviar comandos de velocidade ao robô (ou simulador)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher para as orientações desejadas/alvos
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        # Subscriber de dados de varredura a laser (SensorDataQoS é melhor para sensores)
        self.create_subscription(
            LaserScan, '/laser_scan', self._cb_laser,
            qos_profile=qos_profile_sensor_data)

        # Assinante de leitura do mapa
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.create_subscription(OccupancyGrid, '/map', self._cb_map, map_qos)

        # Timer que controla o ciclo principal do nodo de forma constante
        self.create_timer(timer_period, self._control_loop)
        
        self.get_logger().info(f'[{node_name}] inicializado.')

    def _cb_laser(self, msg: LaserScan) -> None:
        """
        @brief Callback chamado toda vez que um pacote de dados de laser chega.
        
        Armazena as leituras em laser_ranges e aciona on_laser().
        
        @param msg Mensagem LaserScan contendo a varredura atual.
        """
        self.laser_ranges = list(msg.ranges)
        self.laser_angle_min = msg.angle_min
        self.laser_angle_increment = msg.angle_increment
        self.on_laser()

    def _cb_map(self, msg: OccupancyGrid) -> None:
        """
        @brief Callback associado ao recebimento do Mapa Global (OccupancyGrid).
        
        @param msg Mensagem contendo os dados do mapa recebido.
        """
        self.map_msg = msg
        self.on_map()

    def get_front_distance(self, half_angle_deg: float = 15.0) -> float:
        """
        @brief Retorna a distãncia medida pelo obstáculo mais próximo imediatamente à frente do robô.
        
        Filtra as leituras dentro de um cone frontal configurável.
        
        @param half_angle_deg Metade da abertura do cone em graus direcionado à frente do robô.
        @return float Menor distância avaliada no cone, ou infinito caso não existam obstáculos ou dados indisponíveis.
        """
        if not self.laser_ranges:
            return float('inf')
        
        n = len(self.laser_ranges)
        center = n // 2
        
        # fallback para divisões por zero ou incrementos negativos
        inc = 1e-9 if self.laser_angle_increment == 0 else self.laser_angle_increment
        half = int(abs(math.radians(half_angle_deg)) / abs(inc))
        
        start = max(0, center - half)
        end = min(n - 1, center + half)
        
        region = self.laser_ranges[start : end + 1]
        return min(region) if region else float('inf')

    def publish_velocity(self, v: float, w: float) -> None:
        """
        @brief Facilita o envio de comandos de velocidade (linear e angular) à base de rodas.
        
        @param v Velocidade linear (frente) em m/s.
        @param w Velocidade angular (rotação) em rad/s.
        """
        twist = Twist()
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.pub_cmd.publish(twist)

    def publish_target_pose(self, x: float, y: float, theta: float, frame_id: str = 'base_link') -> None:
        """
        @brief Publica uma pose destino (ou vetor de movimento) para visualização.
        
        @param x Posição X da pose alvo.
        @param y Posição Y da pose alvo.
        @param theta Ângulo (yaw) em radianos.
        @param frame_id Frame de referência para a pose.
        """
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = 0.0
        
        # Converter ângulo para quaternion (w e z apenas para rotação em torno de Z)
        pose_msg.pose.orientation.w = math.cos(theta / 2.0)
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = math.sin(theta / 2.0)
        
        self.target_pose_pub.publish(pose_msg)

    def stop(self) -> None:
        """
        @brief Envia um comando de parada imediata enviando o valor zero para as velocidades.
        """
        self.publish_velocity(0.0, 0.0)

    def on_laser(self) -> None:
        """
        @brief Chamado automaticamente ao receber novas leituras do LIDAR.
        """
        pass

    def on_map(self) -> None:
        """
        @brief Chamado automaticamente quando um novo mapa é recebido.
        """
        pass

    def _control_loop(self) -> None:
        """
        @brief Loop contínuo executado através do timer (ex. para reações aleatórias ou predeterminadas).
        """
        pass

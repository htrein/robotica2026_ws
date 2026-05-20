import math
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from nav_msgs.msg import Odometry, OccupancyGrid

def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

class MappingNode(Node):
    """
    @brief Classe base para nós de mapeamento em ROS2.
    
    Gerencia a subscrição de odometria e publicação do mapa (OccupancyGrid).
    """
    def __init__(self, node_name: str = 'mapping_node', timer_period: float = 0.1):
        """
        @brief Construtor da classe MappingNode.
        
        @param node_name Nome do nó no ROS 2.
        @param timer_period Período de controle opcional (ignorado caso o ciclo dependa dos sensores).
        """
        super().__init__(node_name)

        # Estado do robô
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0

        # Mapa
        self.map_msg = OccupancyGrid()
        self.resolution = 0.05
        self.origin_x_cells = 0
        self.origin_y_cells = 0

        # Configuração do Publisher para o mapa global com qualidade de serviço transient_local
        # Isso garante que nodos que conectarem atrasados ainda recebam o último mapa publicado.
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self._cb_odom, 10)

        # Inicializa a grade do mapa com as dimensões específicas.
        self.init_map(20.0, 20.0, 0.05)

        self.get_logger().info(f'[{node_name}] inicializado e aguardando odometria.')

    def init_map(self, map_width_m: float, map_height_m: float, resolution: float) -> None:
        """
        @brief Função auxiliar para inicializar as informações básicas (dimensões e escala) do OccupancyGrid.
        
        @param map_width_m Largura física do mapa em metros.
        @param map_height_m Altura física do mapa em metros.
        @param resolution Resolução (metros por célula).
        """
        self.resolution = resolution
        width_cells = int(map_width_m / self.resolution)
        height_cells = int(map_height_m / self.resolution)

        self.map_msg.header.frame_id = 'map'
        self.map_msg.info.resolution = self.resolution
        self.map_msg.info.width = width_cells
        self.map_msg.info.height = height_cells
        
        # A origem do mapa fica no canto inferior esquerdo, então para que (0,0) seja o centro:
        self.map_msg.info.origin.position.x = -map_width_m / 2.0
        self.map_msg.info.origin.position.y = -map_height_m / 2.0
        self.map_msg.info.origin.position.z = 0.0
        self.map_msg.info.origin.orientation.w = 1.0

        self.origin_x_cells = width_cells // 2
        self.origin_y_cells = height_cells // 2

        # Preenche todo o mapa com células desconhecidas (valor -1)
        self.map_msg.data = [-1] * (width_cells * height_cells)

    def _cb_odom(self, msg: Odometry) -> None:
        """
        @brief Callback para receber atualizações do tópico de odometria (/odom).
        
        Extrai a posição e converte a orientação (quaternio) para yaw (theta).
        Após a atualização, chama o método virtual on_odom().
        
        @param msg Mensagem recebida.
        """
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        
        # Chamada para as classes filhas processarem os novos dados de odometria
        self.on_odom()

    def meters_to_cells(self, wx: float, wy: float) -> tuple[int, int]:
        """
        @brief Transforma coordenadas contínuas do ambiente para índices da grade (grid).
        
        @param wx Coordenada X no mundo (metros).
        @param wy Coordenada Y no mundo (metros).
        @return tuple[int, int] Retorna a coluna (mx) e a linha (my) correspondente no grid.
        """
        mx = int((wx - self.map_msg.info.origin.position.x) / self.resolution)
        my = int((wy - self.map_msg.info.origin.position.y) / self.resolution)
        return mx, my

    def publish_map(self) -> None:
        """
        @brief Publica a mensagem do mapa (OccupancyGrid) atualizada no tópico /map.
        """
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_msg)

    def on_odom(self) -> None:
        """
        @brief Método chamado a cada nova leitura de odometria. Subclasses devem implementar este comportamento.
        """
        pass

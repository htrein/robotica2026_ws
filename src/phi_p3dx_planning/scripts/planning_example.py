#!/usr/bin/env python3
"""
Exemplo simples de navegação usando mapa para planejamento de trajetória.

Este script implementa uma navegação simples: antes de iniciar, verifica no mapa global
se o objetivo é válido (livre de obstáculos). Se for livre, gira para alinhar 
com o objetivo e depois anda em linha reta, parando caso o sensor detecte um obstáculo à frente.
"""

import math
import rclpy
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
from phi_p3dx_planning.main import PlanningNode


class PlanningExample(PlanningNode):
    """
    Planejador de exemplo: valida o goal no mapa global, gira para o goal, anda em frente, e evita colisão.
    """

    def __init__(self):
        super().__init__(node_name='planning_example_navigator', timer_period=0.1)

        # Parâmetros do controle
        self.front_threshold = 0.5  # Distância para parar por obstáculo (m)
        self.angle_threshold = math.radians(15.0)  # Ângulo para considerar alinhado (graus)
        self.linear_speed = 0.2    # Velocidade linear (m/s)
        self.angular_speed = 0.1   # Velocidade angular (rad/s)

    def publish_tree(self) -> None:
        """
        Exibe uma árvore de exemplo (RRT, etc) no RViz.

        Utiliza as configurações já estabelecidas na classe base (PlanningNode)
        para marcações de nós (SPHERE_LIST) e arestas (LINE_LIST).
        Neste exemplo, a função limpa os pontos anteriores e preenche 
        novos pontos usando as coordenadas globais e absolutas baseadas 
        na posição atual do robô, simulando um nó de origem e 3 nós filhos.
        """
        marker_array = MarkerArray()

        # Limpa os pontos anteriores
        self.nodes_marker.points.clear()
        self.edges_marker.points.clear()

        # Definindo os pontos (em coordenadas globais do mapa)
        
        # Nó 0 na posição atual do robô
        p0 = Point(x=self.x, y=self.y, z=0.0)
        
        # Nó 1 à frente (1 metro na direção theta)
        p1 = Point(x=self.x + 1.0 * math.cos(self.theta), 
                   y=self.y + 1.0 * math.sin(self.theta), 
                   z=0.0)
        
        # Nó 2 a 45 graus para a esquerda
        p2 = Point(x=self.x + 1.0 * math.cos(self.theta + math.pi / 4.0),
                   y=self.y + 1.0 * math.sin(self.theta + math.pi / 4.0),
                   z=0.0)

        # Nó 3 a 45 graus para a direita
        p3 = Point(x=self.x + 1.0 * math.cos(self.theta - math.pi / 4.0),
                   y=self.y + 1.0 * math.sin(self.theta - math.pi / 4.0),
                   z=0.0)

        # Adicionando os nós na lista de esferas
        self.nodes_marker.points.extend([p0, p1, p2, p3])

        # Adicionando as arestas (cada par de pontos representa uma linha: p0->p1, p0->p2, p0->p3)
        self.edges_marker.points.extend([p0, p1, p0, p2, p0, p3])

        marker_array.markers.append(self.nodes_marker)
        marker_array.markers.append(self.edges_marker)

        self.marker_pub.publish(marker_array)

    def on_goal(self) -> None:
        """
        Callback chamado automaticamente ao receber um novo objetivo (goal) do RViz.

        Verifica se as coordenadas do objetivo estão dentro dos limites do mapa e
        correspondem a uma célula livre de obstáculos.
        """
        # Se não há um objetivo válido armazenado, ignora
        if self.goal is None:
            return
            
        # Se o mapa global ainda não foi recebido, não é possível validar o destino
        if self.map_msg is None:
            self.get_logger().warn('Novo goal recebido (nenhum mapa recebido ainda), ignorando.')
            self.goal = None
            self.stop()
            return

        # Obtém as coordenadas (em metros) do objetivo
        x, y = self.goal
        
        # Obtém os parâmetros do mapa (resolução, posição de origem, dimensões da grade)
        res = self.map_msg.info.resolution
        origin_x = self.map_msg.info.origin.position.x
        origin_y = self.map_msg.info.origin.position.y
        width = self.map_msg.info.width
        height = self.map_msg.info.height

        # Converte as coordenadas físicas do mundo (x, y) para índices espaciais (coluna, linha) na matriz do mapa
        col = int((x - origin_x) / res)
        row = int((y - origin_y) / res)

        # Confirma se o nó/célula alvo está dentro das margens conhecidas do mapa
        if 0 <= col < width and 0 <= row < height:
            # Como data é um vetor 1D, calcula o índice linear
            idx = row * width + col
            val = self.map_msg.data[idx]
            
            # Verifica o custo de ocupação da célula:
            # - 0 indica totalmente livre
            # - 100 indica totalmente ocupado
            # - -1 indica área desconhecida
            # Consideramos > 50 como obstáculo
            if val > 50 or val == -1:
                self.get_logger().warn(f'Goal ocupado ou desconhecido (custo={val}). Ignorando...')
                self.goal = None  # Reseta o objetivo se for inválido
                self.stop()       # Garante que o robô não tente ir para um ponto perigoso
            else:
                self.get_logger().info(f'Novo goal recebido e é livre (custo={val})! Iniciando...')
        else:
            self.get_logger().warn('Goal fora dos limites do mapa. Ignorando...')
            self.goal = None
            self.stop()

    def _control_loop(self) -> None:
        """
        Loop de controle executado periodicamente (definido por timer_period).

        Implementa a lógica de planejamento local básica: verifica se alcançou
        o objetivo, calcula erros de navegação, decide entre virar, andar ou parar.
        """
        # Se não há objetivo atual, garante que o robô está parado
        if not self.has_goal():
            self.stop()
            return

        # Impede navegação às cegas (exige dados do laser)
        if not self.has_laser_data():
            self.stop()
            return
        
        # Checa se esta suficientemente proximo do objetivo
        if self.distance_to_goal() < self.front_threshold:
            self.clear_goal()
            self.stop()
            return

        # Calcula o erro angular para o objetivo (rotação necessária)
        angle_err = self.angle_to_goal()
        self.get_logger().info(f'Angulo para objetivo={math.degrees(angle_err):.1f}°')


        # Se não estiver apontando adequadamente para o alvo
        if abs(angle_err) > self.angle_threshold:
            # Girar no próprio eixo para alinhar
            w = self.angular_speed 
            if angle_err > 0:
                w = self.angular_speed 
            else:
                w = -self.angular_speed
            self.publish_velocity(0.0, w)
            self.get_logger().info(f'Girando: vel angular={w:.1f}')
        else:
            # Já está alinhado, agora verifica obstáculo à frente antes de andar
            front_dist = self.get_front_distance()
            if front_dist < self.front_threshold:
                self.stop()
                self.get_logger().info('Obstáculo à frente: parando.')
                return

            # Caminho está livre, progredir na direção atual
            self.publish_velocity(self.linear_speed, 0.0)
            self.get_logger().info('Andando em frente.')

        # Atualiza a visualização da árvore na interface do RViz
        self.publish_tree()


def main(args=None):
    rclpy.init(args=args)
    planner = PlanningExample()
    rclpy.spin(planner)
    planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
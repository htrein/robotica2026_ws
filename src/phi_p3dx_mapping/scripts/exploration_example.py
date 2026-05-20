#!/usr/bin/env python3
"""
@brief Exemplo simples de nodo de exploração.

Realiza uma caminhada aleatória: anda para frente até encontrar 
um obstáculo usando os dados do laser de varredura. Ao detectar,
gira para evitar colisão e escolhe uma direção aleatória por tempo.
O mapa é recebido via callback apenas para demonstrar o consumo,
mas não é ativamente usado para decidir o caminho na lógica básica.
"""
import rclpy
import random
from phi_p3dx_mapping.exploration_node import ExplorationNode

class ExplorationExample(ExplorationNode):
    def __init__(self):
        super().__init__(node_name='exploration_example_py')

    def _control_loop(self) -> None:
        if not self.laser_ranges:
            return # Aguarda dados do laser

        front_dist = self.get_front_distance(20.0) # 20 graus para cada lado

        # Distância de segurança (parar e girar se menor que isso)
        MIN_DIST = 1.0

        if front_dist < MIN_DIST:
            # Obstáculo detectado, girar
            self.get_logger().info(f'Obstáculo detectado a frente ({front_dist:.2f}m). Virando...')
            self.publish_target_pose(0.0, 0.0, 1.57, 'base_link')  # Intenção: virar à esquerda
            self.publish_velocity(0.0, 0.4)
        else:
            # Caminho livre, avançar
            self.publish_target_pose(0.0, 0.0, 0.0, 'base_link')  # Intenção: ir em frente
            self.publish_velocity(0.25, 0.0)

    def on_map(self) -> None:
        # Apenas para ilustrar a leitura do mapa gerado pelo outro nodo
        if self.map_msg:
            self.get_logger().info(
                f'Mapa recebido! Dimensões: {self.map_msg.info.width}x{self.map_msg.info.height}, '
                f'Resolução: {self.map_msg.info.resolution:.2f}',
                once=True
            )

            # Imprimir o valor e indice da célula do robo
            robot_x = self.map_msg.info.width // 2
            robot_y = self.map_msg.info.height // 2
            robot_index = robot_x + robot_y * self.map_msg.info.width
            self.get_logger().info(f'Valor da célula onde está o robô: {self.map_msg.data[robot_index]}')

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

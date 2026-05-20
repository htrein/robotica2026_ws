#!/usr/bin/env python3
"""
Exemplo simples de mapeamento iterativo.

Registra as posições já visitadas como área livre.
"""
import rclpy
from phi_p3dx_mapping.mapping_node import MappingNode

class MappingExample(MappingNode):
    def __init__(self):
        super().__init__(node_name='mapping_example_py')

    def on_odom(self) -> None:
        # Toda vez que a odometria chega, marca um círculo ao redor da posição como livre
        self.mark_free_space(self.x, self.y, 0.5) # Raio de 0.5 m
        self.publish_map()

    def mark_free_space(self, wx: float, wy: float, radius_m: float) -> None:
        r_cells = int(radius_m / self.resolution)
        mx, my = self.meters_to_cells(wx, wy)

        width_cells = self.map_msg.info.width
        height_cells = self.map_msg.info.height

        # Percorre uma janela ao redor da posição do robô
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                nx = mx + dx
                ny = my + dy

                # Verifica se a célula está dentro dos limites da grade
                if 0 <= nx < width_cells and 0 <= ny < height_cells:
                    # Verifica se está dentro do raio do círculo
                    if dx*dx + dy*dy <= r_cells * r_cells:
                        idx = ny * width_cells + nx
                        self.map_msg.data[idx] = 0 # 0 significa espaço livre

def main(args=None):
    rclpy.init(args=args)
    node = MappingExample()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

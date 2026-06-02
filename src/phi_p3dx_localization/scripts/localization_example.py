#!/usr/bin/env python3
"""
@brief Exemplo de uso de LocalizationNode em Python.

Demonstra como estender a classe base para implementar comportamento customizado.
Neste exemplo, as partículas rotacionam continuamente, permitindo observar:
- Como a pose estimada acompanha a rotação
- Como a covariância se comporta com movimento
- Exemplo de como sobrescrever update_particles() para adicionar lógica

**Parâmetros adicionais:**
- `rotation_speed` (float, default: 0.5 rad/s)

**Como usar:**
```bash
ros2 run phi_p3dx_localization localization_example_py
```
"""

import rclpy
import math
from phi_p3dx_localization.main import LocalizationNode


class LocalizationExample(LocalizationNode):
    """Exemplo de localização com rotação contínua das partículas."""

    def __init__(self, node_name='localization_example_py'):
        super().__init__(node_name)

        # Parâmetro adicional
        self.declare_parameter('rotation_speed', 0.5)
        self.rotation_speed = self.get_parameter('rotation_speed').get_parameter_value().double_value

        # Estado para controle de tempo
        self.last_update_time = self.get_clock().now()

        self.get_logger().info(
            f'[{node_name}] Exemplo de localização com rotação inicializado.'
        )
        self.get_logger().info(f'Velocidade de rotação: {self.rotation_speed:.2f} rad/s')

    def update_particles(self):
        """Sobrescreve o método de atualização para rotacionar as partículas."""
        # Calcula delta de tempo desde a última atualização
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9  # Converte para segundos
        self.last_update_time = now

        # Incremento de ângulo
        delta_theta = self.rotation_speed * dt

        # Rotaciona todas as partículas
        for p in self.particles:
            p.theta += delta_theta

            # Normaliza theta para [-π, π]
            while p.theta > math.pi:
                p.theta -= 2.0 * math.pi
            while p.theta < -math.pi:
                p.theta += 2.0 * math.pi


def main(args=None):
    """Função principal."""
    rclpy.init(args=args)
    node = LocalizationExample()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
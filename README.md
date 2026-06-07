### Exemplos de execução:
```bash
ros2 launch phi_p3dx_navigation bringup_mobilesim.launch.py map_name:=indoor.map
ros2 launch phi_p3dx_navigation bringup_mobilesim.launch.py map_name:=obstacles.map
```

### Implementação TP1
```bash
ros2 run phi_p3dx_navigation wall_follower_pid
ros2 run phi_p3dx_navigation vfh_simplified
```

### Implementação TP2
```bash
ros2 run phi_p3dx_planning rrt_cpp
ros2 run phi_p3dx_planning restricted_rrt_cpp
```

### Implementação TP3
```bash
ros2 run phi_p3dx_mapping tp3_mapping_cpp
ros2 run phi_p3dx_mapping tp3_exploration_cpp
```

### Implementação TP4
```bash
ros2 run phi_p3dx_localization mcl_node_cpp --ros-args -p num_particles:=10000
```

#### Teleop
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Importante:
- **Wall Following com PID**: Recomenda-se utilizar o mapa `indoor.map`, pois este contém corredores adequados para o comportamento esperado.
- **VFH (Vector Field Histogram)**: Recomenda-se utilizar ambientes com obstáculos (`obstacles` ou `obstacles.map`).

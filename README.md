# Robotica Workspace

O framework permite executar o robô em diferentes ambientes simulados, tanto no Gazebo quanto no MobileSim.

## Gazebo
Os mundos disponíveis atualmente no Gazebo são:
- **obstacles**: Ambiente com obstáculos.
- **empty_world**: Ambiente vazio.

### Exemplos de execução:
```bash
ros2 launch phi_p3dx_navigation bringup_gazebo.launch.py world_name:=obstacles
ros2 launch phi_p3dx_navigation bringup_gazebo.launch.py world_name:=empty_world
```

---

## MobileSim
No MobileSim, estão disponíveis os seguintes mapas:
- **obstacles.map**: Equivalente ao ambiente com obstáculos do Gazebo.
- **indoor.map**: Ambiente com corredores.

### Exemplos de execução:
```bash
ros2 launch phi_p3dx_navigation bringup_mobilesim.launch.py map_name:=indoor.map
ros2 launch phi_p3dx_navigation bringup_mobilesim.launch.py map_name:=obstacles.map
```

---

### Implementação TP1
```bash
ros2 run phi_p3dx_navigation wall_follower_pid
ros2 run phi_p3dx_navigation vfh_simplified
```

## Importante:
- **Wall Following com PID**: Recomenda-se utilizar o mapa `indoor.map`, pois este contém corredores adequados para o comportamento esperado.
- **VFH (Vector Field Histogram)**: Recomenda-se utilizar ambientes com obstáculos (`obstacles` ou `obstacles.map`).
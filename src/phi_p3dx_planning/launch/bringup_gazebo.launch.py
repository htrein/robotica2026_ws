"""
Launch principal para simulação completa no Gazebo 3D.

Este launch configura e inicia:
- O ambiente Gazebo com um mundo SDF (ex.: empty_world ou obstacles).
- O robô Pioneer 3-DX spawnado no mundo, com publishers de estado (robot_state_publisher, joint_state_publisher).
- A ponte ROS-Gazebo para comunicação entre ROS e Gazebo.
- RViz opcionalmente para visualização (padrão: ligado).
- (COMENTADO) O nó de navegação (obstacle_avoidance) para controle do robô.
- O servidor de mapas configurado e ativado com nav2_map_server (se slam=false).
- O mapeamento SLAM com slam_toolbox (se slam=true).

Argumentos:
- world_name: Nome do mundo SDF a carregar (padrão: 'obstacles').
- map_name: Nome do mapa a ser carregado pelo map_server sem a extensão yaml (padrão: 'obstacles').
- slam: Se verdadeiro (true), inicia o slam_toolbox para criar o mapa em vez de carregar um existente (padrão: 'false').
- robot_namespace: Namespace para o robô (padrão: vazio).
- x, y, yaw: Posição inicial do robô no mundo.
- use_rviz: Se deve abrir RViz (padrão: true).

Exemplo de uso:
  ros2 launch phi_p3dx_navigation bringup_gazebo.launch.py world_name:=empty_world use_rviz:=false
"""

from os.path import join
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

## o arquivo main para subir o robô no gazebo 3D
def generate_launch_description():
    pkg_navigation = get_package_share_directory("phi_p3dx_navigation")
    pkg_description = get_package_share_directory("phi_p3dx_description")
    pkg_planning = get_package_share_directory("phi_p3dx_planning")

    world_name = LaunchConfiguration("world_name")
    map_name = LaunchConfiguration("map_name")
    slam = LaunchConfiguration("slam")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    yaw = LaunchConfiguration("yaw")
    namespace = LaunchConfiguration("robot_namespace")
    use_rviz = LaunchConfiguration("use_rviz")

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_env.launch.py")),
        launch_arguments={'world_name': world_name}.items()
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_spawn_gz.launch.py")),
        launch_arguments={
            'x': x,
            'y': y,
            'yaw': yaw,
            'robot_namespace': namespace,
            'use_sim_time': 'true'
        }.items()
    )

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_state_publishers.launch.py")),
        launch_arguments={
            'robot_namespace': namespace,
            'use_sim_time': 'true'
        }.items()
    )

    rviz_config_file = PythonExpression([
        "'", join(pkg_description, 'rviz', 'rviz_map.rviz'), "' if '", slam, "' == 'false' else '", join(pkg_description, 'rviz', 'rviz.rviz'), "'"
    ])

    rviz_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_rviz.launch.py")),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'robot_namespace': namespace,
                    'use_sim_time': 'true',
                    'rviz_config': rviz_config_file,
                }.items()
            )
        ]
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_navigation.launch.py")),
        launch_arguments={
            'robot_namespace': namespace,
            'use_sim_time': 'true'
        }.items()
    )

    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_planning, "launch", "includes", "bringup_map_server.launch.py")),
        condition=UnlessCondition(slam),
        launch_arguments={
            'map_name': map_name,
            'use_sim_time': 'true'
        }.items()
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_planning, "launch", "includes", "bringup_slam.launch.py")),
        condition=IfCondition(slam),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="obstacles"),
        DeclareLaunchArgument("map_name", default_value="obstacles"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("x", default_value="-0.1686"),
        DeclareLaunchArgument("y", default_value="0.0154"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        
        world_launch,
        state_publishers,
        robot_launch,
        map_server_launch,
        slam_launch,
        rviz_launch,
        # navigation_launch
    ])
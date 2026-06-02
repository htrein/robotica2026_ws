"""
Launch auxiliar para iniciar o Map Server.

Este launch:
- Carrega o mapa especificado (via yaml) no nó do nav2_map_server.
- Ativa e gerencia o ciclo de vida do map_server usando o nav2_lifecycle_manager.

Argumentos:
- map_name: Nome do mapa a ser carregado sem extensão (padrão: 'obstacles').
- use_sim_time: Se deve usar o tempo de simulação (padrão: 'false').

Incluído por: bringup_gazebo.launch.py, bringup_mobilesim.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

## inicializa o nav2_map_server e lifecycle_manager com o mapa parametrizado
def generate_launch_description():
    pkg_description = get_package_share_directory('phi_p3dx_description')

    map_name = LaunchConfiguration('map_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_map_name_cmd = DeclareLaunchArgument(
        'map_name',
        default_value='obstacles',
        description='Name of the map to load'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    yaml_filename = PathJoinSubstitution([pkg_description, 'map', [map_name, '.yaml']])

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': yaml_filename},
                    {'use_sim_time': use_sim_time}]
    )

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    start_static_transform_cmd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map_odom',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    ld = LaunchDescription()
    ld.add_action(declare_map_name_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_static_transform_cmd)

    return ld

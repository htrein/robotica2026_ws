"""
Launch auxiliar para mapeamento com slam_toolbox.

Este launch:
- Inicia o nó async_slam_toolbox do pacote slam_toolbox atraves de seu include padrão.

Argumentos:
- use_sim_time: Se deve usar o tempo de simulação (padrão: 'false').

Incluído por: bringup_gazebo.launch.py, bringup_mobilesim.launch.py
"""

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

## inicializa o slam_toolbox no modo online assíncrono para mapeamento
def generate_launch_description():
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_planning = get_package_share_directory('phi_p3dx_planning')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = join(pkg_planning, 'config', 'mapper_params_online_async.yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params_file
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(slam_launch)

    return ld

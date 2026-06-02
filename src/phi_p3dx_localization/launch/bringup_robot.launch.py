"""
Launch principal para controle do robô real Pioneer 3-DX.
Inicia phi_p3dx_aria, map_server (opcional), TF e RViz.
"""
from os.path import join
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_localization = get_package_share_directory('phi_p3dx_localization')
    pkg_navigation = get_package_share_directory('phi_p3dx_navigation')

    port_arg = DeclareLaunchArgument('port', default_value='192.168.1.11:10002', description='')
    namespace_arg = DeclareLaunchArgument('robot_namespace', default_value='', description='pioneer3dx')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true', description='Abrir RViz2 automaticamente')
    map_name_arg = DeclareLaunchArgument('map_name', default_value='obstacles', description='Mapa para carregar (vazio para desabilitar)')
    load_map_arg = DeclareLaunchArgument('load_map', default_value='false', description='Se deve carregar um mapa')

    port      = LaunchConfiguration('port')
    namespace = LaunchConfiguration('robot_namespace')
    use_rviz  = LaunchConfiguration('use_rviz')
    map_name  = LaunchConfiguration('map_name')
    load_map  = LaunchConfiguration('load_map')

    state_publishers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_navigation, "launch", "includes", "bringup_state_publishers.launch.py")),
        launch_arguments={
            'robot_namespace': namespace,
            'use_sim_time': 'false'
        }.items()
    )

    phi_aria_node = Node(
        package='phi_p3dx_aria',
        executable='phi_p3dx',
        name='phi_p3dx_aria',
        namespace=namespace,
        output='screen',
        parameters=[{
            'port':       port,
            'odom_frame':       'odom',
            'base_link_frame':  'base_link',
            'sonar_frame':      'base_link',
            'laser_frame':      'lidar_link',
            'publish_aria_lasers': True,
        }]
    )

    # Map Server (opcional)
    map_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_localization, "launch", "includes", "bringup_map_server.launch.py")),
        condition=IfCondition(load_map),
        launch_arguments={
            'map_name': map_name,
            'use_sim_time': 'false'
        }.items()
    )

    rviz_config_file = join(pkg_localization, 'config', 'localization.rviz')
    
    rviz_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    join(pkg_navigation, 'launch', 'includes', 'bringup_rviz.launch.py')
                ),
                condition=IfCondition(use_rviz),
                launch_arguments={
                    'robot_namespace': namespace,
                    'use_sim_time': 'false',
                    'rviz_config': rviz_config_file
                }.items(),
            ),
        ],
    )

    return LaunchDescription([
        port_arg,
        namespace_arg,
        use_rviz_arg,
        map_name_arg,
        load_map_arg,

        state_publishers,
        phi_aria_node,
        map_server_launch,
        rviz_launch,
    ])

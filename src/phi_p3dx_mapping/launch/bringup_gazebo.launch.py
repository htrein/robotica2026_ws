"""
Launch principal para simulação 3D com Gazebo do pacote phi_p3dx_mapping.
"""
from os.path import join
from launch.conditions import IfCondition
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_navigation = get_package_share_directory("phi_p3dx_navigation")
    pkg_description = get_package_share_directory("phi_p3dx_description")
    pkg_mapping = get_package_share_directory("phi_p3dx_mapping")

    world_name = LaunchConfiguration("world_name")
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

    tf_map_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_mapping, "launch", "includes", "bringup_tf_map_odom.launch.py"))
    )

    rviz_config_file = join(pkg_mapping, 'rviz', 'exploration.rviz')

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

    return LaunchDescription([
        DeclareLaunchArgument("world_name", default_value="obstacles"),
        DeclareLaunchArgument("robot_namespace", default_value=""),
        DeclareLaunchArgument("x", default_value="-0.1686"),
        DeclareLaunchArgument("y", default_value="0.0154"),
        DeclareLaunchArgument("yaw", default_value="0.0"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        
        world_launch,
        state_publishers,
        robot_launch,
        tf_map_odom,
        rviz_launch,
    ])

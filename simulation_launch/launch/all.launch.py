import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_launch')

    robot_model_file_path = os.path.join(pkg_share, 'urdf', 'lunabot_tracked.urdf')
    world_file_path = os.path.join(pkg_share, 'world', 'apollo_landing_site.world')
    rviz_config_file_path = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    world_file = LaunchConfiguration('world_file')
    robot_model_file = LaunchConfiguration('robot_model_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration("z_pose")
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    declare_world_file = DeclareLaunchArgument(
        name='world_file',
        default_value=world_file_path,
        description='Path to world file'
    )
    declare_robot_model_file = DeclareLaunchArgument(
        name='robot_model_file',
        default_value=robot_model_file_path,
        description='Path to robot model file'
    )
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_x_pose = DeclareLaunchArgument(
        name='x_pose',
        default_value='-2.5',
        description='X-coordinate of robot spawn point'
    )
    declare_y_pose = DeclareLaunchArgument(
        name='y_pose',
        default_value='0.0',
        description='Y-coordinate of robot spawn point'
    )
    declare_z_pose = DeclareLaunchArgument(
        name='z_pose',
        default_value='2.0',
        description='z-coordinate of robot spawn point'
    )
    declare_log_level = DeclareLaunchArgument(
        name='log_level',
        default_value='info',
        description='Log level for nodes'
    )
    delcare_rviz_config_file = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=rviz_config_file_path,
        description='Path to RViz config file'
    )

    # declare nodes to launch
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    robot_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_world.launch.py')
        ),
        launch_arguments={
            'world_file': world_file,
            'robot_model_file': robot_model_file,
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
            'z_pose': z_pose
        }.items()
    )

    return LaunchDescription([
        declare_world_file,
        declare_robot_model_file,
        declare_use_sim_time,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_log_level,
        delcare_rviz_config_file,

        rviz_node,

        robot_world_launch
    ])
import os
from os import environ
from os import pathsep

from scripts import GazeboRosPaths

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('simulation_launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    robot_model_file_path = os.path.join(pkg_share, 'urdf', 'lunabot_tracked.urdf')
    world_file_path = os.path.join(pkg_share, 'world', 'lunar_arena.world')
    extra_gazebo_args_str = '--gui-client-plugin libKeyboardGUIPlugin.so'

    world_file = LaunchConfiguration('world_file')
    robot_model_file = LaunchConfiguration('robot_model_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration("z_pose")

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
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_x_pose = DeclareLaunchArgument(
        name='x_pose',
        default_value='-2.1',
        description='X-coordinate of robot spawn point'
    )
    declare_y_pose = DeclareLaunchArgument(
        name='y_pose',
        default_value='0.0',
        description='Y-coordinate of robot spawn point'
    )
    declare_z_pose = DeclareLaunchArgument(
        name='z_pose',
        default_value='1.0',
        description='z-coordinate of robot spawn point'
    )
    declare_log_level = DeclareLaunchArgument(
        name='log_level',
        default_value='info',
        description='Log level for nodes'
    )
    declare_extra_gazebo_args = DeclareLaunchArgument(
        name='extra_gazebo_args',
        default_value=extra_gazebo_args_str,
        description='Extra arguments to be passed to Gazebo'
    )

    gzserver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )
    # gzclient_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     ),
    #     launch_arguments={
    #         'verbose': 'true'
    #     }.items()
    # )
    
    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    env = {
        'GAZEBO_MODEL_PATH': model,
        'GAZEBO_PLUGIN_PATH': plugin,
        'GAZEBO_RESOURCE_PATH': media,
    }

    cmd = ['gzclient', '--gui-client-plugin', 'libKeyboardGUIPlugin.so']
    gzclient_launch = ExecuteProcess(
        cmd=cmd,
        output='screen',
        additional_env=env,
        shell=False,
        on_exit=Shutdown()
    )

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={
            'robot_model_file': robot_model_file,
            'use_sim_time': use_sim_time
        }.items()
    )
    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'robot_model_file': robot_model_file,
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
        declare_extra_gazebo_args,

        gzserver_launch,
        gzclient_launch,
        robot_state_publisher_launch,
        spawn_robot_launch
    ])
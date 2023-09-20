import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

package_name = 'crab_bot'
pkg_share = get_package_share_directory(package_name)
default_rviz_config_path = PathJoinSubstitution(
    [pkg_share, 'config/main.rviz'])
ARGUMENTS = [
    DeclareLaunchArgument(name='log_level', default_value='info',
                          description='Logging level'),
    DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                          description='Absolute path to rviz config file'),
    DeclareLaunchArgument(name='use_ros2_control', default_value='true',
                          description='Flag to enable use_ros2_control'),
    DeclareLaunchArgument(name='use_sim_time', default_value='true',
                          description='Flag to enable use_sim_time'),
    DeclareLaunchArgument('world_path', default_value='',
                          description='The world path, by default is empty.world'),
]


def generate_launch_description():

    log_level = LaunchConfiguration("log_level")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    world_path = LaunchConfiguration("world_path")

    # Create a robot_state_publisher node
    xacro_file = PathJoinSubstitution([
        pkg_share, 'crabbot_description', 'robot.2wd.urdf.xacro'])
    robot_description_config = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        ' ',
        xacro_file,
        ' ',
        'use_ros2_control:=',
        use_ros2_control])
    params = {'robot_description': robot_description_config,
              'use_sim_time': use_sim_time}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # gazebo_params_file = os.path.join(
    #     pkg_share, 'config', 'gazebo_params.yaml')
    # # Include the Gazebo launch file, provided by the gazebo_ros package
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #         get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    #     launch_arguments={
    #         'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    # )
    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             world_path],
        output='screen',
    )
    # Gazebo client
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        # condition=IfCondition(LaunchConfiguration('gui')),
    )

    gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_share, 'launch', 'joystick.launch.py'
        ])),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    twist_mux_params = PathJoinSubstitution([
        pkg_share, 'config', 'twist_mux_topics.yaml'])
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel_out', '/diffbot_base_controller/cmd_vel_unstamped')]
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'crab_bot'],
        output='screen'
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Launch them all!
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(gamepad)
    ld.add_action(twist_mux)
    ld.add_action(gzserver)
    ld.add_action(gzclient)
    ld.add_action(spawn_entity)
    ld.add_action(diff_drive_spawner)
    ld.add_action(joint_broad_spawner)

    return ld

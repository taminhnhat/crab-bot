import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit

from launch_ros.actions import Node


def generate_launch_description():

    use_ros2_control = LaunchConfiguration("use_ros2_control")
    pkg_share = get_package_share_directory("crab_bot")
    default_rviz_config_path = os.path.join(pkg_share, 'config/main.rviz')

    #
    # robot state publisher
    #
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_share, "description", "robot.urdf.xacro"]),
        " use_ros2_control:=true",
        " use_sim:=false",
    ])

    robot_description = {'robot_description': robot_description_content}
    controller_params_file = PathJoinSubstitution(
        [pkg_share, 'config', 'robot_controllers.yaml'])

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file],
        output="both",
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
        # remappings=[
        #     ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
    )

    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller",
                   "--controller-manager", "/controller_manager"],
    )

    delayed_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node,
            on_exit=[robot_controller_spawner],
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node,
            on_exit=[rviz_node],
        )
    )

    joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'joystick.launch.py'
        )]), launch_arguments={'use_sim_time': 'false'}.items()
    )

    twist_mux_params = os.path.join(
        pkg_share, 'config', 'twist_mux_topics.yaml')
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # Launch them all!
    # ld = LaunchDescription()
    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_ros2_control', default_value='true',
                             description='Flag to enable use_ros2_control'),
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner_node,
        delayed_robot_controller_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        joy_node,
        twist_mux_node,
    ])

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name = 'crab_bot'  # <--- CHANGE ME
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    pkg_share = get_package_share_directory(package_name)
    default_rviz_config_path = os.path.join(pkg_share, 'config/main.rviz')

    #
    # robot state publisher
    #
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([pkg_share, "description", "robot.urdf.xacro"]),
        " use_ros2_control:=",
        use_ros2_control,
        ' use_sim_time:=',
        use_sim_time
    ])

    robot_description = {'robot_description': robot_description_content}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    #
    # rviz
    #
    gamepad = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'joystick.launch.py'
        )]), launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    #
    # twist mux
    #
    twist_mux_params = os.path.join(
        pkg_share, 'config', 'twist_mux_topics.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    robot_description = Command(
        ['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params_file = os.path.join(
        pkg_share, 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file]
    )

    delayed_controller_manager = TimerAction(
        period=3.0, actions=[controller_manager])

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_spawner],
        )
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

    # Code for delaying a node (I haven't tested how effective it is)
    #
    # First add the below lines to imports
    # from launch.actions import RegisterEventHandler
    # from launch.event_handlers import OnProcessExit
    #
    # Then add the following below the current diff_drive_spawner
    # delayed_diff_drive_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=spawn_entity,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    #
    # Replace the diff_drive_spawner in the final return with delayed_diff_drive_spawner

    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                              description='Absolute path to rviz config file'),
        DeclareLaunchArgument(name='use_sim_time', default_value='false',
                             description='Flag to enable use_sim_time'),
        DeclareLaunchArgument(name='use_ros2_control', default_value='true',
                             description='Flag to enable use_ros2_control'),
        robot_state_publisher,
        gamepad,
        twist_mux,
        delayed_controller_manager,
        delayed_diff_drive_spawner,
        delayed_joint_broad_spawner,
        rviz
    ])

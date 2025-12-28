import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable, LogInfo, Shutdown
from launch.actions.timer_action import TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

"""Launch gz server, ros-gz-bridge, collision publisher, jackal, and gz gui optionally."""

# TODO: use_sim_time?
# TODO: gz_sim.launch.py not safely exiting 

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('gui', default_value='true',
                          choices=['true', 'false'], description='Start gazebo gui.'),
    DeclareLaunchArgument('world_idx', default_value='1',
                          description='BARN World Index: [0-299].'),
    DeclareLaunchArgument('setup_path',
                          default_value=PathJoinSubstitution([get_package_share_directory('jackal_helper'), 'config']),
                          description='Clearpath setup path. The folder which contains robot.yaml.'),
    DeclareLaunchArgument('out_file',
                          default_value="out.txt",
                          description='File name which trial results are stored.'),
    DeclareLaunchArgument('timeout',
                          default_value='100',
                          description='Trial timeout time in seconds.'),
]

def parse_world_idx(world_idx:str)->str:
    world_idx = int(world_idx)
    if world_idx < 300:  # static environment from 0-299
        world_name = f"BARN/world_{world_idx}.world" 
    elif world_idx < 360:  # Dynamic environment from 300-359
        world_name = f"DynaBARN/world_{world_idx - 300}.world"
    else:
        raise ValueError(f"World index {world_idx} does not exist")

    return  world_name

def launch_ros_gazebo(context, *args, **kwargs):
    """
    Launches Gazebo server + gui, sets up resources path, and launches ros-gz-bridge.
    """
    # Packages
    pkg_jackal_helper = get_package_share_directory('jackal_helper')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Paths
    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gui_config = PathJoinSubstitution([pkg_jackal_helper, 'config', 'gui.config'])

    gui_cmd = "" if LaunchConfiguration('gui').perform(context)=='true' else " -s"
    world_name = parse_world_idx(LaunchConfiguration("world_idx").perform(context))
    world_path = os.path.join(pkg_jackal_helper, "worlds", world_name)
    
    # Gazebo Simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [world_path,
                        gui_cmd,
                         ' -r',
                         ' --gui-config ',
                         gui_config])
        ]
    )

    # bridge_config_file = PathJoinSubstitution([pkg_jackal_helper, 'config', 'bridge.yaml'])
    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        # parameters=[{"config_file":bridge_config_file}]
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/default/control@ros_gz_interfaces/srv/ControlWorld',
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
            '/robot/touched@std_msgs/msg/Bool@gz.msgs.Boolean',
            '/model/robot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ]
    )

    return [gz_sim, clock_bridge]

def spawn_jackal(context, *args, **kwargs):
    # SPAWN ROBOT
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')

    spawner_launch_path = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])
    
    world_name = parse_world_idx(LaunchConfiguration('world_idx').perform(context))

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([spawner_launch_path]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', world_name),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', '2.0'),
            ('y', '2.0'),
            ('z', '0.3')]
    )

    return [robot_spawn]

def generate_launch_description():
    # Start the BARN_runner node after 5 seconds. this replaces the run.py in ROS1 version of The_BARN_Challenge.
    BARN_runner_node = TimerAction(
        period = 10.0,
        actions=[
            LogInfo(msg="BARN Runner node started. Starting Trial..."),
            Node(
                package="jackal_helper",
                executable="barn_runner.py",
                name="barn_runner",
                output='screen',
                on_exit=[LogInfo(msg="Trial ended. Shutting down in 5 seconds..."),TimerAction(period=5.0, actions=[Shutdown()])],
                parameters=[
                    {'world_idx': LaunchConfiguration('world_idx')},
                    {'out_file': LaunchConfiguration('out_file')},
                    {'timeout': LaunchConfiguration('timeout')}
                    ]
            )
        ]
    ) 

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_ros_gazebo))
    ld.add_action(OpaqueFunction(function=spawn_jackal))
    ld.add_action(BARN_runner_node)
    return ld


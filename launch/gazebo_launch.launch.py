import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

"""Launch gz server, ros-gz-bridge, collision publisher, jackal, and gz gui optionally."""

ARGUMENTS = [
    DeclareLaunchArgument('rviz', default_value='false',
                          choices=['true', 'false'], description='Start rviz.'),
    DeclareLaunchArgument('gui', default_value='true',
                          choices=['true', 'false'], description='Start gazebo gui.'),
    DeclareLaunchArgument('world_idx', default_value='1',
                          description='Gazebo World'),
    DeclareLaunchArgument('setup_path',
                          default_value=[get_package_share_directory('jackal_helper'), '/config'],
                          description='Clearpath setup path'),
]

def parse_world_idx(world_idx:str)->str:
    base_path = get_package_share_directory("jackal_helper")
    world_idx = int(world_idx)
    if world_idx < 300:  # static environment from 0-299
        world_name = f"BARN/world_{world_idx}.world" 
        INIT_POSITION = [-2.25, 3, 1.57]  # in world frame
        GOAL_POSITION = [0, 10]  # relative to the initial position
    elif world_idx < 360:  # Dynamic environment from 300-359
        world_name = f"DynaBARN/world_{world_idx - 300}.world"
        INIT_POSITION = [11, 0, 3.14]  # in world frame
        GOAL_POSITION = [-20, 0]  # relative to the initial position
    else:
        raise ValueError(f"World index {world_idx} does not exist")

    return os.path.join(base_path, "worlds", world_name)

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

    # # Determine all ros packages that are sourced
    # packages_paths = [os.path.join(p, 'share') for p in os.getenv('AMENT_PREFIX_PATH').split(':')]

    # # Set ignition resource path to include all sourced ros packages
    # gz_sim_resource_path = SetEnvironmentVariable(
    #     name='GZ_SIM_RESOURCE_PATH',
    #     value=[
    #         os.path.join(pkg_jackal_helper, 'worlds', 'BARN') + ':',
    #         ':' + ':'.join(packages_paths)])

    gui_cmd = "" if LaunchConfiguration('gui').perform(context)=='true' else " -s"
    world_path = parse_world_idx(LaunchConfiguration("world_idx").perform(context))
    
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

    bridge_config_file = PathJoinSubstitution([pkg_jackal_helper, 'config', 'bridge.yaml'])
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
    
    world_name = f"world_{LaunchConfiguration('world_idx').perform(context)}.world"

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
    
    # # Collision Publisher Node
    # collision_pub_node = Node(
    #     package="jackal_helper",
    #     executable="collision_publisher"
    # )

    gz_sim_via_clearpath_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([get_package_share_directory('clearpath_gz'), 'launch', 'gz_sim.launch.py'])]),
        launch_arguments=[
            ('world', LaunchConfiguration('world'))
        ]
    )

    

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_ros_gazebo))
    # ld.add_action(gz_sim_via_clearpath_gz)
    ld.add_action(OpaqueFunction(function=spawn_jackal))
    # ld.add_action(collision_pub_node)
    return ld


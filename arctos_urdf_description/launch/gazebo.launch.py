import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('arctos_urdf_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Default paths
    default_model_path = os.path.join(pkg_share, 'urdf', 'arctos_urdf.xacro')
    
    # Launch arguments
    paused_arg = DeclareLaunchArgument(
        name='paused',
        default_value='false',
        description='Start Gazebo paused'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Start Gazebo GUI'
    )
    
    headless_arg = DeclareLaunchArgument(
        name='headless',
        default_value='false',
        description='Start Gazebo headless'
    )
    
    debug_arg = DeclareLaunchArgument(
        name='debug',
        default_value='false',
        description='Start Gazebo in debug mode'
    )
    
    # Robot description
    robot_description_content = Command([
        'xacro ', default_model_path
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'paused': LaunchConfiguration('paused'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'gui': LaunchConfiguration('gui'),
            'headless': LaunchConfiguration('headless'),
            'debug': LaunchConfiguration('debug'),
        }.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Spawn robot in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'arctos_urdf',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.0'
        ],
        output='screen'
    )
    
    return LaunchDescription([
        paused_arg,
        use_sim_time_arg,
        gui_arg,
        headless_arg,
        debug_arg,
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node
    ])

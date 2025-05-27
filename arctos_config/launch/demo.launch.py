import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    # Get package directories
    arctos_config_dir = get_package_share_directory('arctos_config')
    arctos_urdf_dir = get_package_share_directory('arctos_urdf_description')
    
    # Launch arguments
    use_gui_arg = DeclareLaunchArgument('use_gui', default_value='false')
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='true')
    
    # Robot description
    robot_description_content = Command([
        'xacro ', 
        os.path.join(arctos_urdf_dir, 'urdf', 'arctos_urdf.xacro')
    ])
    
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    # SRDF
    robot_description_semantic_content = Command([
        'cat ',
        os.path.join(arctos_config_dir, 'config', 'arctos_urdf.srdf')
    ])
    robot_description_semantic = {'robot_description_semantic': ParameterValue(robot_description_semantic_content, value_type=str)}
    
    # Load kinematics config
    kinematics_yaml_file = os.path.join(arctos_config_dir, 'config', 'kinematics.yaml')
    with open(kinematics_yaml_file, 'r') as file:
        kinematics_config = yaml.safe_load(file)
    
    # Load joint limits config
    joint_limits_yaml_file = os.path.join(arctos_config_dir, 'config', 'joint_limits.yaml')
    with open(joint_limits_yaml_file, 'r') as file:
        joint_limits_config = yaml.safe_load(file)
    
    # Load OMPL planning config
    ompl_planning_yaml_file = os.path.join(arctos_config_dir, 'config', 'ompl_planning.yaml')
    with open(ompl_planning_yaml_file, 'r') as file:
        ompl_planning_config = yaml.safe_load(file)
    
    # Load fake controllers config
    fake_controllers_yaml_file = os.path.join(arctos_config_dir, 'config', 'fake_controllers.yaml')  
    with open(fake_controllers_yaml_file, 'r') as file:
        fake_controllers_config = yaml.safe_load(file)
    
    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'source_list': ['move_group/fake_controller_joint_states']
        }],
        condition=UnlessCondition(LaunchConfiguration('use_gui'))
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{
            'source_list': ['move_group/fake_controller_joint_states']
        }],
        condition=IfCondition(LaunchConfiguration('use_gui'))
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen'
    )
    
    # MoveGroup node parameters
    move_group_parameters = [
        robot_description,
        robot_description_semantic,
        kinematics_config,
        ompl_planning_config,
        fake_controllers_config,
        joint_limits_config,
        {
            'use_sim_time': False,
            'publish_robot_description_semantic': True,
            'allow_trajectory_execution': True,
            'fake_execution': True,
            'fake_execution_type': 'interpolate',
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'moveit_manage_controllers': True,
            'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
            'start_state_max_bounds_error': 0.1,
        }
    ]
    
    # MoveGroup node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=move_group_parameters
    )
    
    # RViz with updated config
    rviz_config_file = os.path.join(arctos_config_dir, 'config', 'moveit2.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic
        ],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    return LaunchDescription([
        use_gui_arg,
        use_rviz_arg,
        joint_state_publisher,
        joint_state_publisher_gui,
        robot_state_publisher,
        move_group_node,
        rviz_node,
    ])

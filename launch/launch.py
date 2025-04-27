import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro
from launch.conditions import IfCondition


def generate_launch_description():

    pkg_name = 'manipulator'  # the package name

    # Add launch argument for controller selection
    controller_type = LaunchConfiguration('controller_type', default='forward_position_controller')
    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='forward_position_controller',
        description='Controller type to use: forward_position_controller or joint_trajectory_controller'
    )

    # Add launch arguments for optional nodes
    rviz_enabled = LaunchConfiguration('rviz', default='true')
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    joint_state_enabled = LaunchConfiguration('joint_state_publisher', default='true')
    declare_joint_state = DeclareLaunchArgument(
        'joint_state_publisher',
        default_value='true',
        description='Whether to launch joint state publisher'
    )

    gazebo_enabled = LaunchConfiguration('gazebo', default='true')
    declare_gazebo = DeclareLaunchArgument(
        'gazebo',
        default_value='true',
        description='Whether to launch Gazebo'
    )

    # World file configuration
    world_file = LaunchConfiguration('world', default='world.xml')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='world.xml',
        description='World file to load in Gazebo'
    )

    pkg_share = get_package_share_directory(pkg_name)

    urdf_path = 'resources/robot_description/manipulator.urdf.xacro'
    rviz_relative_path = 'resources/rviz/config.rviz'
    rviz_absolute_path = os.path.join(pkg_share, rviz_relative_path)

    # extracting the robot deffinition from the xacro file
    xacro_file = os.path.join(pkg_share, urdf_path)
    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_name = "Franka_Panda"

    # add the path to the model file to gazebo
    models_path = os.path.join(get_package_share_directory(pkg_name), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        model_path = models_path
        
    # Handle world file path using PathJoinSubstitution
    world_path = PathJoinSubstitution([
        pkg_share,
        'resources/worlds',
        world_file
    ])

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # Rviz2 node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_absolute_path],
        condition=IfCondition(rviz_enabled)
    )

    # Gazebo launch file
    # Note: Gazebo Classic (current version) reaches end-of-life in January 2025
    # Migration guide: https://gazebosim.org/docs/latest/gazebo_classic_migration
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={
            'world': world_path,
            'output': 'log',  # Redirect Gazebo output to log file
            'verbose': 'true'  # Enable verbose output for debugging
        }.items(),
        condition=IfCondition(gazebo_enabled)
    )

    # entity spawn node (to spawn the robot from the /robot_description topic)
    spawn_entity_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                             arguments=['-topic', 'robot_description',
                                        '-entity', robot_name],
                             output='screen',
                             condition=IfCondition(gazebo_enabled))

    # spawning the joint broadcaster
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Load controller configuration
    controller_config = os.path.join(
        get_package_share_directory('manipulator'),
        'resources',
        'controllers',
        'controllers.yaml'
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_type], 
        output="screen",
        parameters=[controller_config]
    )

    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(joint_state_enabled)
    )
    
    # Run the nodes
    return LaunchDescription([
        declare_controller_type,  # Add the argument declaration
        declare_rviz,  # Add RViz argument
        declare_joint_state,  # Add joint state publisher argument
        declare_gazebo,  # Add Gazebo argument
        declare_world,  # Add world file argument
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        node_robot_state_publisher,
        node_joint_state_publisher,
        launch_gazebo,
        spawn_entity_robot,
        node_rviz,
        spawn_broadcaster,
        spawn_controller
    ])

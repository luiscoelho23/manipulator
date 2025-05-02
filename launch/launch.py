import os
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro
from launch.conditions import IfCondition
import yaml


def generate_launch_description():
    # Process command line arguments to identify the selected controller
    selected_controller = 'forward_position_controller'  # Default controller
    for arg in sys.argv:
        if arg.startswith('controller_type:='):
            selected_controller = arg.split(':=')[1]
            break

    pkg_name = 'manipulator'  # the package name

    # Add launch argument for controller selection
    controller_type = LaunchConfiguration('controller_type', default='forward_position_controller')
    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='forward_position_controller',
        description='Controller type to use (example: forward_position_controller)'
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
            'verbose': 'true',  # Enable verbose output for debugging
            'robot_description': robot_description_content,  # Pass robot description to Gazebo
            'use_sim_time': 'true',  # Use simulation time
            'pause': 'false'  # Start simulation unpaused
        }.items(),
        condition=IfCondition(gazebo_enabled)
    )

    # entity spawn node (to spawn the robot from the /robot_description topic)
    spawn_entity_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                             arguments=['-topic', 'robot_description',
                                        '-entity', robot_name],
                             output='screen',
                             condition=IfCondition(gazebo_enabled))

    # Load controller configuration from file
    controller_config = os.path.join(
        get_package_share_directory('manipulator'),
        'config',
        'controllers.yaml'
    )
    print("Using controller config:", controller_config)
    
    # Load the controller configuration file to extract controller names
    with open(controller_config, 'r') as file:
        controller_yaml = yaml.safe_load(file)
        # Extract controller names from the controller_manager section
        controllers = []
        if 'controller_manager' in controller_yaml and 'ros__parameters' in controller_yaml['controller_manager']:
            # Skip the update_rate parameter and any other parameters that aren't controllers
            for name, params in controller_yaml['controller_manager']['ros__parameters'].items():
                if isinstance(params, dict) and 'type' in params:
                    controllers.append(name)
        print(f"Found controllers: {controllers}")
    
    # Validate selected controller exists in the available controllers
    if selected_controller not in controllers and selected_controller != 'joint_state_broadcaster':
        print(f"WARNING: Selected controller '{selected_controller}' not found in controllers.yaml")
        print(f"Using default controller 'forward_position_controller' instead")
        if 'forward_position_controller' in controllers:
            selected_controller = 'forward_position_controller'
        elif controllers:
            selected_controller = controllers[0]
        else:
            print("ERROR: No controllers defined in controllers.yaml")
    
    # Create the resources/controllers directory and symlink if needed
    os.makedirs(os.path.join(get_package_share_directory('manipulator'), 'config'), exist_ok=True)
    controller_yaml_dest = os.path.join(get_package_share_directory('manipulator'), 'config', 'controllers.yaml')
    if not os.path.exists(controller_yaml_dest):
        os.symlink(controller_config, controller_yaml_dest)
    
    # Controller manager node with controllers loaded from yaml file
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="screen"
    )

    # Spawn the joint state broadcaster first
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    # Joint state publisher node
    node_joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(joint_state_enabled)
    )
    
    # Build the launch description with required components
    ld = LaunchDescription([
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
        controller_manager_node,  # Add the controller manager node
        node_rviz,
        spawn_broadcaster,
    ])
    
    # Function to create controller spawning node with active or inactive status
    def create_controller_node(controller_name, active=True):
        args = [controller_name, "--controller-manager", "/controller_manager"]
        if not active:
            args.append("--inactive")
        
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=args,
            output="screen"
        )
    
    # For each controller, create one node
    print(f"Selected controller: {selected_controller}")
    print(f"Use controller_type:=<controller_name> to select a different controller")
    
    for controller in controllers:
        # Skip joint_state_broadcaster as it's handled separately
        if controller == 'joint_state_broadcaster':
            continue
            
        # Add controller nodes to the launch description
        # Default controller is active, others are inactive by default
        is_active = (controller == selected_controller)
        
        if is_active:
            ld.add_action(create_controller_node(controller, active=True))
        else:
            ld.add_action(create_controller_node(controller, active=False))
    
    # Add example of how to enable a different controller from command line
    print("\nTo launch with a specific controller, use:")
    print(f"ros2 launch {pkg_name} launch.py controller_type:=<controller_name>")
    print(f"Available controllers: {', '.join(controllers)}")
    
    return ld

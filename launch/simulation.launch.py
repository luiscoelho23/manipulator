import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
import xacro


def generate_launch_description():

    pkg_name = 'manipulator'  # the package name

    pkg_share = get_package_share_directory(pkg_name)

    urdf_path = 'resources/robot_description/manipulator.urdf.xacro'

    rviz_relative_path = 'resources/rviz/config.rviz'

    rviz_absolute_path = os.path.join(pkg_share, rviz_relative_path)

    # extracting the robot deffinition from the xacro file
    xacro_file = os.path.join(pkg_share, urdf_path)
    robot_description_content = xacro.process_file(xacro_file).toxml()
    robot_name = "Franka_Panda"

    # add the path to the model file to  gazebo
    models_path = os.path.join(get_package_share_directory(pkg_name), 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        model_path = os.environ['GAZEBO_MODEL_PATH'] + ':' + models_path
    else:
        model_path = models_path
        
    world_relative_path =  "resources/worlds/world.xml"
    world_absolute_path = os.path.join(pkg_share, world_relative_path)

    # robot state publisher nodey
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
        arguments=['-d', rviz_absolute_path]
    )
    # Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
        launch_arguments={'world': world_absolute_path}.items()
    )
    # entity spawn node (to spawn the robot from the /robot_description topic)
    spawn_entity_robot = Node(package='gazebo_ros', executable='spawn_entity.py',
                             arguments=['-topic', 'robot_description',
                                        '-entity', robot_name],
                             output='screen')

    # spawning the joint broadcaster
    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        #arguments=["forward_position_controller"], 
        arguments=["joint_trajectory_controller"],
        output="screen",
    )
    
    # Run the nodes
    return LaunchDescription([
    
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=model_path),
        node_robot_state_publisher,
        launch_gazebo,
        spawn_entity_robot,
        # node_rviz,
        spawn_broadcaster,
        spawn_controller
    ])

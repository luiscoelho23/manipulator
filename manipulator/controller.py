#!/usr/bin/python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint, JointTrajectory
from std_msgs.msg import Float64MultiArray, Header
from builtin_interfaces.msg import Duration
from rcl_interfaces.srv import GetParameters
import numpy as np
from urdf_parser_py.urdf import URDF
import xml.etree.ElementTree as ET
import logging
from threading import Thread, Event
import time
from control_msgs.msg import JointTolerance
from controller_manager_msgs.msg import ControllerState
import subprocess

class ControllerClient(Node):
    def __init__(self, ts=0.1, controller_type='auto', control_mode='position'):
        super().__init__(node_name='joint_controller')
        
        # Configure logger to output to terminal
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        # Also configure Python's root logger
        logging.basicConfig(level=logging.INFO)
        
        self.get_logger().info(f'Initializing controller with type: {controller_type} and control mode: {control_mode}')

        self.real_angles = None
        self.ts = ts
        self.controller_type = controller_type
        self.control_mode = control_mode
        self._stop_event = Event()
        self._executor = None
        
        # Debugging flag
        self.debug_mode = True
        
        # Define joint names first - this is fallback if we can't get from controller
        self.joint_names = ['panda_joint1',
                          'panda_joint2',
                          'panda_joint3',
                          'panda_joint4',
                          'panda_joint5',
                          'panda_joint6',
                          'panda_joint7']
        self.get_logger().info('Defined joint names for Panda robot')
        
        # Initialize joint limits dictionary
        self.joint_limits = {}
        
        # Get robot description from parameter server
        self._get_robot_description()
        
        # Parse joint limits from URDF
        self._parse_joint_limits()

        # Create joint state subscriber with QoS profile
        qos_profile = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos_profile
        )
        self.get_logger().info('Created joint state subscriber with QoS profile')

        # Create a timer to check if we're receiving joint states
        self.create_timer(1.0, self._check_joint_states)

        # Detect controller type if auto
        if controller_type == 'auto':
            self.get_logger().info('Auto-detecting controller type...')
            self.controller_type = self._detect_controller_type()
            self.get_logger().info(f'Detected controller type: {self.controller_type}')

        # Initialize the appropriate controller
        self._init_controller()

        # Get joint names from controller parameters
        self._get_joint_names_from_controller()

        # Start the background thread for spinning
        self._spin_thread = Thread(target=self._spin_node, daemon=True)
        self._spin_thread.start()
        self.get_logger().info('Started background thread for spinning')

    def _init_controller(self):
        """Initialize the appropriate controller based on type and mode"""
        if self.controller_type == 'forward_position_controller':
            # Create publisher for forward_position_controller
            self._publisher = self.create_publisher(
                Float64MultiArray,
                '/forward_position_controller/commands',
                10
            )
            self.get_logger().info('Created publisher for forward position controller')
        else:
            # For trajectory controllers, determine the correct action server name
            if self.controller_type == 'joint_trajectory_controller':
                if self.control_mode == 'position':
                    action_name = '/joint_trajectory_controller_position/follow_joint_trajectory'
                else:  # effort mode
                    action_name = '/joint_trajectory_controller_effort/follow_joint_trajectory'
            else:
                # Use the controller type directly as the action server prefix
                action_name = f'/{self.controller_type}/follow_joint_trajectory'
                
            self.get_logger().info(f'Using action server: {action_name}')
            self._action_client = ActionClient(
                node=self, 
                action_type=FollowJointTrajectory,
                action_name=action_name
            )
            
            # Check if action server is available
            if not self._action_client.wait_for_server(timeout_sec=1.0):
                self.get_logger().warning(f"Action server {action_name} not available immediately. Will check again when sending goals.")
            else:
                self.get_logger().info(f"Successfully connected to action server {action_name}")

    def _spin_node(self):
        """Background thread function to spin the node"""
        self._executor = rclpy.executors.SingleThreadedExecutor()
        self._executor.add_node(self)
        
        try:
            while not self._stop_event.is_set():
                self._executor.spin_once(timeout_sec=0.1)
        finally:
            self._executor.remove_node(self)
            self._executor.shutdown()

    def _get_robot_description(self):
        """Get robot description from parameter server"""
        self.get_logger().info('Requesting robot description from parameter server...')
        # Create client to get parameters
        client = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        
        # Wait for service to be available
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for parameter service...')
        
        # Create request
        request = GetParameters.Request()
        request.names = ['robot_description']
        
        # Call service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.values:
                self.robot_description = response.values[0].string_value
                self.get_logger().info('Successfully retrieved robot description')
            else:
                self.get_logger().error('Failed to get robot description: No values returned')
        else:
            self.get_logger().error('Service call failed: No result received')

    def _parse_joint_limits(self):
        """Parse joint limits from URDF and estimate acceleration limits"""
        try:
            self.get_logger().info('Parsing joint limits from URDF...')
            # Parse URDF
            robot = URDF.from_xml_string(self.robot_description)
            
            # Extract joint limits
            for joint in robot.joints:
                if joint.type != 'fixed':  # Only consider movable joints
                    joint_name = joint.name
                    if joint_name in self.joint_names:
                        # Get velocity and effort limits from URDF
                        vel_limit = joint.limit.velocity if hasattr(joint.limit, 'velocity') else 20.0
                        effort_limit = joint.limit.effort if hasattr(joint.limit, 'effort') else 87.0
                        
                        # Estimate acceleration limit based on velocity and effort
                        accel_limit = min(vel_limit / 0.1, effort_limit / 2.0)
                        
                        self.joint_limits[joint_name] = {
                            'accel': accel_limit,
                            'vel': vel_limit,
                            'effort': effort_limit
                        }
                        self.get_logger().info(f'Set limits for {joint_name}: accel={accel_limit:.2f}, vel={vel_limit:.2f}, effort={effort_limit:.2f}')
            
            # Verify all required joints have limits
            for joint_name in self.joint_names:
                if joint_name not in self.joint_limits:
                    self.get_logger().warning(f'No limits found for {joint_name}, using defaults')
                    self.joint_limits[joint_name] = {
                        'accel': 10.0,
                        'vel': 20.0,
                        'effort': 87.0
                    }
                    
        except Exception as e:
            self.get_logger().error(f'Error parsing joint limits: {str(e)}')
            # Set default limits if parsing fails
            for joint_name in self.joint_names:
                self.joint_limits[joint_name] = {
                    'accel': 10.0,
                    'vel': 20.0,
                    'effort': 87.0
                }
            self.get_logger().warning('Using default joint limits due to parsing error')

    def _detect_controller_type(self):
        """Auto-detect available controllers"""
        self.get_logger().info('Attempting to detect available controllers...')
        
        # Try trajectory controllers first (preferred)
        controller_options = [
            ('joint_trajectory_controller_position', '/joint_trajectory_controller_position/follow_joint_trajectory'),
            ('joint_trajectory_controller_effort', '/joint_trajectory_controller_effort/follow_joint_trajectory'),
        ]
        
        # Check each controller type
        for controller_type, action_name in controller_options:
            try:
                self.get_logger().info(f'Checking for controller: {controller_type} at {action_name}')
                temp_client = ActionClient(node=self, action_type=FollowJointTrajectory, action_name=action_name)
                if temp_client.wait_for_server(timeout_sec=0.5):
                    self.get_logger().info(f'Detected {controller_type} controller')
                    
                    # Set control mode based on detected controller type
                    if 'effort' in controller_type:
                        self.control_mode = 'effort'
                    else:
                        self.control_mode = 'position'
                        
                    return controller_type
            except Exception as e:
                self.get_logger().debug(f'Failed to detect {controller_type}: {str(e)}')
        
        # Fall back to forward position controller
        try:
            # Check if publisher topic exists
            self.get_logger().info('Checking for forward_position_controller...')
            # Just return the forward controller type since we can't easily check if it exists
            self.control_mode = 'position'
            return 'forward_position_controller'
        except Exception as e:
            self.get_logger().warning(f'Failed to detect forward_position_controller: {str(e)}')
        
        # If nothing is detected, default to forward position controller
        self.get_logger().warning('No controllers detected, defaulting to forward_position_controller')
        self.control_mode = 'position'
        return 'forward_position_controller'

    def _joint_state_callback(self, msg):
        """Callback for joint state messages"""
        try:
            # Create a mapping of joint names to their positions
            joint_positions = dict(zip(msg.name, msg.position))
            
            # Update real_angles with the positions in the correct order
            self.real_angles = []
            for joint_name in self.joint_names:
                if joint_name in joint_positions:
                    self.real_angles.append(joint_positions[joint_name])
                else:
                    if self.debug_mode:
                        self.get_logger().warning(f'Joint {joint_name} not found in joint states. Available joints: {msg.name}')
                    return
            
        except Exception as e:
            self.get_logger().error(f'Error in joint state callback: {str(e)}')

    def _check_joint_states(self):
        """Check if we're receiving joint states and log the status"""
        if self.real_angles is None:
            self.get_logger().warning('No joint states received yet')
            if self.debug_mode:
                self.get_logger().info('Make sure the robot state publisher is running and joint states are being published')            

    def send_goal(self, angle1, angle2, angle3, angle4, angle5, angle6, angle7, duration=None):
        """
        Send a goal to the controller.
        
        Args:
            angle1-angle7: Joint angles in radians for position mode, joint efforts in Nm for effort mode
            duration: Optional duration for the trajectory (only used for trajectory controllers)
        """
        try:
            angles = [float(angle1), float(angle2), float(angle3), float(angle4), float(angle5), float(angle6), float(angle7)]

            # Initial log message BEFORE sending goal (to fix the print order issue)
            if self.debug_mode:
                self.get_logger().info("Preparing to send goal to controller")
                self.get_logger().info(f"Using controller type: {self.controller_type} in {self.control_mode} mode")
            
            # For trajectory controllers
            if 'trajectory' in self.controller_type:
                temp_node = rclpy.create_node('trajectory_test_node')
                # Log values for debugging
                if self.debug_mode:
                    if self.control_mode == 'position':
                        self.get_logger().info(f"Target joint positions: {[f'{a:.3f}' for a in angles]}")
                    else:
                        self.get_logger().info(f"Target joint efforts: {[f'{a:.3f}' for a in angles]}")
                
                # Create goal message
                goal_msg = FollowJointTrajectory.Goal()
                
                # Set up the trajectory
                traj = JointTrajectory()
                traj.joint_names = self.joint_names
                
                # Use current position for start point if available, otherwise use zeros
                start_positions = self.real_angles if self.real_angles is not None else [0.0] * 7
                
                # Create two points - start and target
                start_point = JointTrajectoryPoint()
                start_point.positions = [float(pos) for pos in start_positions]
                start_point.velocities = [0.0] * len(self.joint_names)
                
                # Set accelerations for position mode
                if self.control_mode == 'position':
                    start_point.accelerations = [0.0] * len(self.joint_names)
                # Set efforts for effort mode
                else:
                    start_point.effort = [0.0] * len(self.joint_names)
                
                # Important: Use small non-zero time to avoid "zero time_from_start" error
                start_point.time_from_start = Duration(sec=0, nanosec=1000000) #1ms 
                
                # Create target point
                end_point = JointTrajectoryPoint()
                
                if self.control_mode == 'position':
                    # In position mode, the angles are target positions
                    end_point.positions = [float(pos) for pos in angles]
                    end_point.velocities = [0.0] * len(self.joint_names)
                    end_point.accelerations = [0.0] * len(self.joint_names)
                else:
                    # In effort mode, we still need positions (use current)
                    end_point.positions = [float(pos) for pos in start_positions]
                    end_point.velocities = [0.0] * len(self.joint_names)
                    # Set the efforts to the target values
                    end_point.effort = [float(eff) for eff in angles]
                
                # Use provided duration or default to 2.0 seconds (successful in tests)
                traj_duration = 2.0 if duration is None else float(duration)
                traj_sec = int(traj_duration)
                traj_nanosec = int((traj_duration - traj_sec) * 1e9)
                end_point.time_from_start = Duration(sec=traj_sec, nanosec=traj_nanosec)
                
                # Add points to trajectory
                traj.points = [start_point, end_point]
                
                # Set goal
                goal_msg.trajectory = traj
                
                # Set goal tolerance - use standard tolerance
                goal_msg.goal_time_tolerance = Duration(sec=1, nanosec=0)
                
                # Fast non-blocking check for action server
                if not self._action_client.wait_for_server(timeout_sec=0.1):
                    self.get_logger().error('Action server not available: Check that the controller is properly loaded and active')
                    return

                # Important: Log "Sending goal" message just before the actual send
                self.get_logger().info(f"Sending goal to {self.control_mode} controller...")
                
                # Dump full goal details for debugging
                if self.debug_mode:
                    self.dump_goal_details(goal_msg)
                
                # Send the goal
                self._send_goal_future = self._action_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(temp_node, self._send_goal_future, timeout_sec=1.0)
                goal_handle = self._send_goal_future.result()
                
                if self.debug_mode:
                    self.get_logger().info(f"Goal sent to {self.controller_type}")
            else:
                # For forward controllers (position or effort)
                if self.debug_mode:
                    self.get_logger().info(f"Sending command to forward_position_controller: {[f'{a:.3f}' for a in angles]}")
                
                # Send command using publisher for forward_position_controller
                msg = Float64MultiArray()
                msg.data = angles
                self._publisher.publish(msg)
                
                # Log after sending
                if self.debug_mode:
                    self.get_logger().info("Command sent to forward_position_controller")
                
        except Exception as e:
            self.get_logger().error(f'Error in send_goal: {str(e)}')
            if self.debug_mode:
                import traceback
                self.get_logger().error(traceback.format_exc())
        
        finally:
        # Clean up
            if 'temp_node' in locals():
                temp_node.destroy_node()

    def set_debug_mode(self, enable=True):
        """Enable or disable debug mode for verbose logging"""
        self.debug_mode = enable
        self.get_logger().info(f"Debug mode {'enabled' if enable else 'disabled'}")

    def get_controller_info(self):
        """Return information about the current controller configuration"""
        info = {
            'controller_type': self.controller_type,
            'control_mode': self.control_mode,
            'joint_names': self.joint_names,
        }
        
        if 'trajectory' in self.controller_type:
            info['action_server'] = self._action_client.action_name
        else:
            info['publisher_topic'] = '/forward_position_controller/commands'
            
        return info

    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info('Shutting down controller node')
        self._stop_event.set()
        if hasattr(self, '_spin_thread'):
            self._spin_thread.join()
        if self._executor is not None:
            self._executor.shutdown()
        super().destroy_node()
        
    # Alias for backward compatibility
    destroy = destroy_node

    def check_controller_status(self):
        """Check the status of controllers to help diagnose why goals might be rejected"""
        try:
            self.get_logger().info("Checking controller status...")
            # Create a subscription to controller_manager/status topic
            # This is a temporary subscription just to check status once
            status_received = False
            
            def status_callback(msg):
                nonlocal status_received
                if status_received:
                    return
                    
                status_received = True
                self.get_logger().info("Controller statuses:")
                for controller in msg.controller:
                    self.get_logger().info(f"  - {controller.name}: {controller.state}, type: {controller.type}")
                    if controller.name == self.controller_type or controller.name in self.controller_type:
                        self.get_logger().info(f"  Claimed resources: {controller.claimed_interfaces}")
            
            status_sub = self.create_subscription(
                msg_type=ControllerState,
                topic='/controller_manager/status',
                callback=status_callback,
                qos_profile=10
            )
            
            # Spin a few times to get the status message
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
                if status_received:
                    break
                    
            # Cleanup the temporary subscription
            self.destroy_subscription(status_sub)
            
            # Check if the controller is active
            if not status_received:
                self.get_logger().error("Could not get controller status - check if controller_manager is running")
                
        except Exception as e:
            self.get_logger().error(f"Error checking controller status: {e}")
    
    def dump_goal_details(self, goal_msg):
        """Debug helper to dump complete details about a goal message"""
        self.get_logger().info("=== DUMPING GOAL DETAILS ===")
        try:
            # Log trajectory header
            self.get_logger().info(f"Trajectory header stamp: {goal_msg.trajectory.header.stamp.sec}.{goal_msg.trajectory.header.stamp.nanosec}")
            self.get_logger().info(f"Trajectory header frame: {goal_msg.trajectory.header.frame_id}")
            
            # Log joint names
            self.get_logger().info(f"Joint names: {goal_msg.trajectory.joint_names}")
            
            # Log number of points
            self.get_logger().info(f"Number of points: {len(goal_msg.trajectory.points)}")
            
            # Log first point
            if len(goal_msg.trajectory.points) > 0:
                p = goal_msg.trajectory.points[0]
                self.get_logger().info(f"First point time: {p.time_from_start.sec}.{p.time_from_start.nanosec:09d}s")
                self.get_logger().info(f"First point positions: {[f'{pos:.4f}' for pos in p.positions]}")
                self.get_logger().info(f"First point velocities: {[f'{vel:.4f}' for vel in p.velocities]}")
                self.get_logger().info(f"First point accelerations: {[f'{acc:.4f}' for acc in p.accelerations]}")
                
            # Log last point
            if len(goal_msg.trajectory.points) > 1:
                p = goal_msg.trajectory.points[-1]
                self.get_logger().info(f"Last point time: {p.time_from_start.sec}.{p.time_from_start.nanosec:09d}s")
                self.get_logger().info(f"Last point positions: {[f'{pos:.4f}' for pos in p.positions]}")
                self.get_logger().info(f"Last point velocities: {[f'{vel:.4f}' for vel in p.velocities]}")
                self.get_logger().info(f"Last point accelerations: {[f'{acc:.4f}' for acc in p.accelerations]}")
                
            # Log goal time tolerance
            secs = goal_msg.goal_time_tolerance.sec + goal_msg.goal_time_tolerance.nanosec * 1e-9
            self.get_logger().info(f"Goal time tolerance: {secs:.4f}s")
            
            # Log tolerances
            if hasattr(goal_msg, 'path_tolerance') and goal_msg.path_tolerance:
                self.get_logger().info(f"Path tolerance: {len(goal_msg.path_tolerance)} joints")
                for tol in goal_msg.path_tolerance:
                    self.get_logger().info(f"  {tol.name}: pos={tol.position}, vel={tol.velocity}, acc={tol.acceleration}")
                    
            if hasattr(goal_msg, 'goal_tolerance') and goal_msg.goal_tolerance:
                self.get_logger().info(f"Goal tolerance: {len(goal_msg.goal_tolerance)} joints")
                for tol in goal_msg.goal_tolerance:
                    self.get_logger().info(f"  {tol.name}: pos={tol.position}, vel={tol.velocity}, acc={tol.acceleration}")
            
        except Exception as e:
            self.get_logger().error(f"Error dumping goal details: {e}")
        self.get_logger().info("==============================")

    def _get_joint_names_from_controller(self):
        """Get joint names from controller parameter server"""
        self.get_logger().info('Attempting to get joint names from controller parameter...')
        
        # Determine parameter name based on controller type
        if 'trajectory' in self.controller_type:
            # For trajectory controllers, try to get the joints parameter
            controller_name = self.controller_type
            
            try:
                # Create client to get parameters
                client = self.create_client(GetParameters, f'/{controller_name}/get_parameters')
                
                # Wait for service to be available (with a short timeout)
                if client.wait_for_service(timeout_sec=0.5):
                    # Create request
                    request = GetParameters.Request()
                    request.names = ['joints']
                    
                    # Call service
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                    
                    if future.result() is not None:
                        response = future.result()
                        if response.values and hasattr(response.values[0], 'string_array_value') and response.values[0].string_array_value:
                            self.joint_names = response.values[0].string_array_value
                            self.get_logger().info(f'Retrieved joint names from controller: {self.joint_names}')
                            return
                        else:
                            self.get_logger().warning('Failed to get joint names from parameter: No valid values returned')
                    else:
                        self.get_logger().warning('Service call failed or timed out')
                else:
                    self.get_logger().warning(f'Parameter service for {controller_name} not available')
                
            except Exception as e:
                self.get_logger().warning(f'Error getting joint names from parameter: {e}')
        
        # Alternative approach - try using ROS CLI to get params
        try:
            import subprocess
            controller_name = self.controller_type
            cmd = ['ros2', 'param', 'get', f'/{controller_name}', 'joints']
            result = subprocess.run(cmd, capture_output=True, text=True)
            
            if result.returncode == 0 and result.stdout:
                # Parse the output - it will look like "value: \n- joint1\n- joint2\n..."
                joint_list = []
                for line in result.stdout.splitlines():
                    if line.strip().startswith('-'):
                        joint_name = line.strip()[2:].strip()  # Remove '- ' prefix
                        joint_list.append(joint_name)
            
                if joint_list:
                    self.joint_names = joint_list
                    self.get_logger().info(f'Retrieved joint names using CLI: {self.joint_names}')
                    return
            else:
                self.get_logger().warning(f'Failed to get joints param using CLI: {result.stderr}')
            
        except Exception as e:
            self.get_logger().warning(f'Error using CLI to get joint names: {e}')
        
        self.get_logger().warning(f'Could not get joint names from controller, using default: {self.joint_names}')
        
        # Try to at least check if the default names are valid
        try:
            joint_state_msg = self._get_latest_joint_state()
            if joint_state_msg:
                for joint_name in self.joint_names:
                    if joint_name not in joint_state_msg.name:
                        self.get_logger().error(f'Joint name {joint_name} not found in /joint_states')
                        self.get_logger().error(f'Available joints: {joint_state_msg.name}')
        except Exception as e:
            self.get_logger().warning(f'Error validating joint names: {e}')

    def _get_latest_joint_state(self):
        """Get the latest joint state message"""
        joint_state_msg = None
        
        # Create a simple callback to capture the message
        def joint_state_callback(msg):
            nonlocal joint_state_msg
            joint_state_msg = msg
        
        # Create a temporary subscription
        temp_sub = self.create_subscription(
            JointState,
            '/joint_states',
            joint_state_callback,
            1
        )
        
        # Spin a few times to get a message
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
            if joint_state_msg is not None:
                break
            
        # Clean up
        self.destroy_subscription(temp_sub)
        
        return joint_state_msg

    if __name__ == '__main__':
        import sys
        print("Running trajectory controller test...")
        success = test_trajectory_controller()
        print(f"Test {'SUCCEEDED' if success else 'FAILED'}")
        sys.exit(0 if success else 1)
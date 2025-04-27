#!/usr/bin/python3

from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from rclpy.duration import Duration

class ControllerClient(Node):
    def __init__(self, ts=0.001, controller_type='forward_position_controller'):
        super().__init__(node_name='joint_controller')

        self.real_angles = JointState().position
        self.ts = ts
        self.controller_type = controller_type
        self.joint_names = ['panda_joint1',
                            'panda_joint2',
                            'panda_joint3',
                            'panda_joint4',
                            'panda_joint5',
                            'panda_joint6',
                            'panda_joint7']

        if controller_type == 'joint_trajectory_controller':
            # Create action client for joint_trajectory_controller
            self._action_client = ActionClient(node=self, action_type=FollowJointTrajectory,
                                           action_name='/joint_trajectory_controller/follow_joint_trajectory')
        else:
            # Create publisher for forward_position_controller
            self._publisher = self.create_publisher(
                Float64MultiArray,
                '/forward_position_controller/commands',
                10
            )

    def send_goal(self, angle1, angle2, angle3, angle4, angle5, angle6, angle7):
        angles = [float(angle1), float(angle2), float(angle3), float(angle4), float(angle5), float(angle6), float(angle7)]

        if self.controller_type == 'joint_trajectory_controller':
            # Send goal using action client
            goal_msg = FollowJointTrajectory.Goal()
            points = []

            point = JointTrajectoryPoint()
            point.time_from_start = Duration(seconds=self.ts).to_msg()
            point.positions = angles

            points.append(point)

            goal_msg.goal_time_tolerance = Duration(seconds=(self.ts/10)).to_msg()
            goal_msg.trajectory.joint_names = self.joint_names
            goal_msg.trajectory.points = points

            self._action_client.wait_for_server()
            self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        else:
            # Send command using publisher
            msg = Float64MultiArray()
            msg.data = angles
            self._publisher.publish(msg)
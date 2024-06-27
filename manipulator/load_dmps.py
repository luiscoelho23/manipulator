#!/usr/bin/python3

import sys
import numpy as np
import controller
import time
import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
from threading import Thread

sys.path.append('/home/luisc/workspaces/ws_manipulator/build/mplibrary')
import pymplibrary as motion


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.listener_callback,
            10)

        self.real_angle = [0, 0, 0]

    def listener_callback(self, msg):
        self.real_angle[0] = msg.position[1]
        self.real_angle[1] = msg.position[3]
        self.real_angle[2] = msg.position[5]


def dmp_load():
    while phase.value < 0.99:
        phase.update(ts)
        policy.value[1] = subscriber.real_angle[0]
        policy.value[2] = subscriber.real_angle[1]
        policy.value[3] = subscriber.real_angle[2]
        policy.update(ts, phase.value)
        client.send_goal(0, (policy.value[1]) * np.pi / 180, 0, (-180 + policy.value[2]) * np.pi / 180,
                         0, (-180 + policy.value[3]) * np.pi / 180, 0)
        time.sleep(ts)


# load entire motion library and use first DMP stored
# @note alternatively, only required DMP could be loaded (from index or label)
library = motion.mpx.load_from('/home/luisc/workspaces/ws_manipulator/src/manipulator/resource/dmp/dmp_output/dmp.mpx')
policy = library.policies[0]
# policy.reset(policy.goals())
for dim in policy:
    dim.reset(dim.goal)

# initialize phase system
phase = motion.LinearPacer(1.0)
phase.value = phase.limits.lower
# apply temporal scaling
# @note optional, in case we want to speed up / slow down duration
tscaling = 0.5
phase.pace *= tscaling
policy.tscale(tscaling)

# integration timestep
# @note arbitrary, does not change shape of policy (if below aliasing threshold), only the number of updates in a cycle/period
# @note alternatively, regression config or origin reference data could be used to estimate/load sampling frequency
ts = 0.00669

rclpy.init()

client = controller.JointControlClient(ts)
subscriber = MinimalSubscriber()

thread = Thread(target=dmp_load)
thread.start()

rclpy.spin(subscriber)

subscriber.destroy_node()
client.destroy_node()
thread.join()
rclpy.shutdown()

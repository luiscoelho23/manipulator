#!/usr/bin/python3

import sys
import numpy as np
import math
import controller
import load_rl
import time
import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
from threading import Thread
import kdl_parser
import PyKDL as kdl
from urdf_parser_py.urdf import URDF

sys.path.append('/home/luisc/ws_manipulator/build/mplibrary')
import pymplibrary as motion

def get_ee_position(ang1,ang2,ang3):      
        
        joint_angles = kdl.JntArray(7)
        joint_angles[0] = (-180) * np.pi /180  # Joint 1 angle in radians
        joint_angles[1] = (ang1) * np.pi /180  # Joint 2 angle in radians
        joint_angles[2] = (0) * np.pi /180  # Joint 3 angle in radians
        joint_angles[3] = (-180 + ang2) * np.pi /180  # Joint 4 angle in radians
        joint_angles[4] = (0) * np.pi /180  # Joint 5 angle in radians
        joint_angles[5] = (135 + ang3) * np.pi /180  # Joint 6 angle in radians
        joint_angles[6] = 0 * np.pi /180  # Joint 7 angle in radians
        
        fk_solver = kdl.ChainFkSolverPos_recursive(kdl_chain)
        eeframe = kdl.Frame()
        fk_solver.JntToCart(joint_angles, eeframe)
        
        return eeframe.p.x() , eeframe.p.z()

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
    
    main_traj = open("/home/luisc/main.csv", "w")
    real_traj = open("/home/luisc/rl.csv", "w")
    
    main_trajectory_ang = np.array([])

    x_ob, z_ob = -0.2, -0.3

    phase.value = 0.0
    #policy.reset([126.86,138.226,27.3175])
    policy.reset([95.39636567969238,95.39636567969238,9.219886560012206])


    while phase.value < 0.999: 
        phase.update(ts)
        policy.update(ts, phase.value)

        #if (phase.value > 0.555 and phase.value < 0.560):
            #policy.value = [policy.value[0] + 1,policy.value[1] +1,policy.value[2]+1]
        main_trajectory_ang = np.append(main_trajectory_ang,[policy.value[0], policy.value[1], policy.value[2]])
        x,z = get_ee_position(policy.value[0],policy.value[1],policy.value[2])
        out = str(x) + ";" + str(z) + "\n"
        main_traj.write(out)

    main_traj.close()

    print("Done")

    phase.value = 0
    #policy.reset([126.86,138.226,27.3175])
    policy.reset([95.39636567969238,95.39636567969238,9.219886560012206])
    traj_index = 0

    

    #policy.goal(0).addState(10, 0.5, 0.495)
    #policy.goal(1).addState(10, 0.5, 0.495)
    #policy.goal(2).addState(10, 0.5, 0.495)

    #policy.setGoals

    while phase.value < 0.999:

        x,z = get_ee_position(policy.value[0],policy.value[1],policy.value[2])
        x_t,z_t = get_ee_position(main_trajectory_ang[0 + traj_index * 3],main_trajectory_ang[1 + traj_index * 3],main_trajectory_ang[2 + traj_index * 3])
        state = [x,z,x_t,z_t,phase.value,x_ob,z_ob]
        action = load_rl.get_action(state)
        phase.update(ts)
        policy.update(ts, phase.value)
        #if phase.value > 0.49 and phase.value < 0.5:
        #    print("+++")
        #    policy.value = [policy.value[0] + 1 ,policy.value[1] + 1 ,policy.value[2]+ 1]
        if math.dist([x,z],[x_ob,z_ob]) < 0.10:
            policy.value = [policy.value[0] + action[0] ,policy.value[1] + action[1] ,policy.value[2]+ action[2]]
        #client.send_goal((-180) * np.pi / 180,( policy.value[0]  ) * np.pi / 180, 0, (-180 + policy.value[1]) * np.pi / 180,
        #                 0, (135 + policy.value[2]) * np.pi / 180, 0)
        out = str(x) + ";" + str(z) + "\n"
        real_traj.write(out)
       # traj_index += 1
        time.sleep(0.001)

    
    print("DONE")
    real_traj.close()
    

robot = URDF.from_xml_file("/home/luisc/ws_manipulator/src/manipulator/resources/robot_description/manipulator.urdf")
(_,kdl_tree) = kdl_parser.treeFromUrdfModel(robot)
kdl_chain = kdl_tree.getChain("panda_link0", "panda_finger")

# load entire motion library and use first DMP stored
# @note alternatively, only required DMP could be loaded (from index or label)
#library = motion.mpx.load_from('/home/luisc/ws_manipulator/src/manipulator/resources/dmp/dmp_output/dmp.mpx')
library = motion.mpx.load_from('/home/luisc/ws_dmps/src/mplearn/python/dmp.mpx')
policy = library.policies[0]
phase = motion.LinearPacer(1.0)
tscaling = 1
phase.pace *= tscaling
policy.tscale(tscaling)

# integration timestep
# @note arbitrary, does not change shape of policy (if below aliasing threshold), only the number of updates in a cycle/period
# @note alternatively, regression config or origin reference data could be used to estimate/load sampling frequency
ts = 0.001

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

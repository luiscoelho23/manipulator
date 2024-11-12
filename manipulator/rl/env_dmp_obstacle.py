import sys
import numpy as np

import time

import gym
from gym import error, spaces
from gym.error import DependencyNotInstalled

sys.path.append('/home/luisc/ws_manipulator/build/mplibrary')
import pymplibrary as motion

import PyKDL as kdl
from urdf_parser_py.urdf import URDF
import kdl_parser as kdl_parser


class DmpObstacleEnv:
    
    def __init__(self):     
        self.main_trajectory_ang = np.array([])
        
        self.agent_position = np.empty(2)
        self.agent_last_position = np.empty(2)
        self.agent_velocity = np.empty(2)

        self.target_position = np.empty(2)
        self.target_last_position = np.empty(2)
        self.target_velocity = np.empty(2)
        
        self.obstacles = np.array([(-0.4, -0.15)])
        self.closest_obtacle = np.array([-0.4, -0.15])
        self.traj_index = 0
        self.done = False   
    
        self.robot = URDF.from_xml_file("/home/luisc/ws_manipulator/src/manipulator/resources/robot_description/manipulator.urdf")
        (_,self.kdl_tree) = kdl_parser.treeFromUrdfModel(self.robot)
        self.kdl_chain = self.kdl_tree.getChain("panda_link0", "panda_finger")
        self.library = motion.mpx.load_from('/home/luisc/ws_manipulator/src/manipulator/resources/dmp/dmp_output/mapped_dmp_traj5.mpx')
        self.policy = self.library.policies[0]
        self.phase = motion.LinearPacer(1.0)
        self.phase.value = 0.1
        self.ts = 0.001 
        self.load_dmp()
        #self.policy.reset([126.86,138.226,27.3175])
        self.policy.reset([95.39636567969238,95.39636567969238,9.219886560012206])
        
        self.render_mode = True
        self.fps = 25000000 # MAX
        self.window_size = 512
        self.scale = 1 
        self.surf = None
        self.window = None
        self.clock = None
             
        
    def reset(self, seed=None, options=None):
        
        self.agent_position = np.empty(2)
        self.agent_last_position = np.empty(2)
        self.agent_velocity = np.empty(2)

        self.target_position = np.empty(2)
        self.target_last_position = np.empty(2)
        self.target_velocity = np.empty(2)
        
        self.traj_index = 0
        
        self.policy = self.library.policies[0]
        self.phase = motion.LinearPacer(1.0)
        self.phase.value = 0.1
        self.ts = 0.001 
        self.load_dmp()
        #self.policy.reset([126.86,138.226,27.3175])
        self.policy.reset([95.39636567969238,95.39636567969238,9.219886560012206])
        
        self.done = False
        
        return self._get_obs(), {}

    def _get_obs(self):
        return self.agent_position[0],self.agent_position[1],self.target_position[0],self.target_position[1], self.phase.value, self.closest_obtacle[0],self.closest_obtacle[1]

    def step(self, action):
        
        reward = 0
        self.phase.update(self.ts)
        self.policy.update(self.ts, self.phase.value)
        

        for obstacle in self.obstacles:
            delta = self.agent_position - obstacle
            dist = np.sqrt(delta[0]**2 + delta[1]**2)


            if dist < 0.10:  
                self.policy.value = [self.policy.value[0] + action[0] ,self.policy.value[1] + action[1] , self.policy.value[2]+ action[2] ]
            else:
                reward -= abs(action[0] + action[1] + action[2]) * 100

            if dist < 0.08:
                reward -= 10/dist
            if dist < 0.06:
                reward -= 1000/dist
            if dist < 0.04:
                reward -= 100000/dist

        

        self.agent_position = self.get_ee_position(self.policy.value[0] ,self.policy.value[1],self.policy.value[2])
        self.agent_velocity = np.array(self.agent_position) - np.array(self.agent_last_position)
        self.agent_last_position = self.agent_position

        self.target_position = self.get_ee_position(self.main_trajectory_ang[0 + self.traj_index * 3],self.main_trajectory_ang[1 + self.traj_index * 3],self.main_trajectory_ang[2 + self.traj_index * 3])
        self.target_velocity = np.array(self.target_position) - np.array(self.target_last_position)
        self.target_last_position = self.target_position

        delta = np.array(self.agent_position) - np.array(self.target_position)
        distance_main_trajectory = np.sqrt(delta[0]**2 + delta[1]**2) 
        
        reward -= (distance_main_trajectory * 1000)
        
        reward -= np.sqrt((self.agent_velocity[0] - self.target_velocity[0])**2 + (self.agent_velocity[1] - self.target_velocity[1])**2) * 1000
        
        if self.phase.value >= 0.992:
            if distance_main_trajectory < 0.05:
                reward += 1/(distance_main_trajectory + 0.01)
            reward -= distance_main_trajectory * 1000
            self.done = True
       
        self.traj_index += 1
        
        self.render()
        
        return self._get_obs(), reward, self.done, {},{}

    def render(self):
        
        try:
            import pygame
            from pygame import gfxdraw
        except ImportError:
            raise DependencyNotInstalled(
                "pygame is not installed, run `pip install gym[box2d]`"
            )
        if self.window is None and self.render_mode:
            pygame.init()
            pygame.display.init()
            self.window = pygame.display.set_mode((self.window_size, self.window_size))
        if self.clock is None:
            self.clock = pygame.time.Clock()

        self.surf = pygame.Surface((self.window_size, self.window_size))

        pygame.transform.scale(self.surf, (self.scale, self.scale))
        pygame.draw.rect(self.surf, (255, 255, 255), self.surf.get_rect())    
        
        pygame.draw.circle(self.surf, (250,0,0), ((self.obstacles[0][0]+0.9) * self.window_size,(self.obstacles[0][1]+0.8) * self.window_size), 17)

        pygame.draw.circle(self.surf, (0,0,255), ((self.agent_position[0]+0.9) * self.window_size,(self.agent_position[1]+0.8) * self.window_size), 5)
        
        pygame.draw.circle(self.surf, (0,255,0), ((self.target_position[0]+0.9) * self.window_size,(self.target_position[1]+0.8) * self.window_size), 5)
        
           
        self.surf = pygame.transform.flip(self.surf, False, True)

        if self.render_mode:
            assert self.window is not None
            self.window.blit(self.surf, (0, 0))
            pygame.event.pump()
            self.clock.tick(self.fps)
            pygame.display.flip()

    def close(self):
        pass

    def load_dmp(self):
        policy = self.library.policies[0]
        
        #policy.reset([126.86,138.226,27.3175])
        policy.reset([95.39636567969238,95.39636567969238,9.219886560012206])
        
        phase = motion.LinearPacer(1.0)
        phase.value = 0.1
        
        while phase.value < 0.992:
            phase.update(self.ts)
            policy.update(self.ts, phase.value)
            self.main_trajectory_ang = np.append(self.main_trajectory_ang,[policy.value[0], policy.value[1], policy.value[2]])
        
        self.agent_position = self.get_ee_position(self.main_trajectory_ang[0],self.main_trajectory_ang[1],self.main_trajectory_ang[2])
        self.target_position = self.get_ee_position(self.main_trajectory_ang[0],self.main_trajectory_ang[1],self.main_trajectory_ang[2])

    def get_ee_position(self,ang1,ang2,ang3):      
        
        joint_angles = kdl.JntArray(7)
        joint_angles[0] = (-180) * np.pi /180  # Joint 1 angle in radians
        joint_angles[1] = (ang1) * np.pi /180  # Joint 2 angle in radians
        joint_angles[2] = (0) * np.pi /180  # Joint 3 angle in radians
        joint_angles[3] = (-180 + ang2) * np.pi /180  # Joint 4 angle in radians
        joint_angles[4] = (0) * np.pi /180  # Joint 5 angle in radians
        joint_angles[5] = (135 + ang3) * np.pi /180  # Joint 6 angle in radians
        joint_angles[6] = 0 * np.pi /180  # Joint 7 angle in radians
        
        fk_solver = kdl.ChainFkSolverPos_recursive(self.kdl_chain)
        eeframe = kdl.Frame()
        fk_solver.JntToCart(joint_angles, eeframe)
        
        return eeframe.p.x() , eeframe.p.z()
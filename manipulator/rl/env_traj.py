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

import csv


class traj_generator:
    
    def __init__(self):
        
        self.index = 0

        self.robot = URDF.from_xml_file("/home/luisc/ws_manipulator/src/manipulator/resources/robot_description/manipulator.urdf")
        (_,self.kdl_tree) = kdl_parser.treeFromUrdfModel(self.robot)
        self.kdl_chain = self.kdl_tree.getChain("panda_link0", "panda_finger")

        self.human_ang = []

        with open("/home/luisc/ws_manipulator/src/manipulator/resources/dmp/3dmp.csv","r") as csvfile:
            reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:
                self.human_ang.append(row)

        self.robot_ang = self.human_ang[0]

        self.human_ee_postion = []
        with open("/home/luisc/ws_manipulator/src/manipulator/resources/dmp/ee_trajectory.csv","r") as csvfile:
            next(csvfile)
            reader = csv.reader(csvfile, quoting=csv.QUOTE_NONNUMERIC)
            for row in reader:
                self.human_ee_postion.append(row)

        self.robot_ee_position = self.get_ee_position(self.human_ang[0][0],self.human_ang[0][1],self.human_ang[0][2])
        self.robot_ee_prev_position = self.robot_ee_position

        self.render_mode = True
        self.fps = 2500000000 # MAX
        self.window_size = 512
        self.scale = 1 
        self.surf = None
        self.window = None
        self.clock = None
             
        
    def reset(self, seed=None, options=None):
        
        self.index = 0
        self.robot_ang = self.human_ang[0]
        self.robot_ee_position = self.get_ee_position(self.human_ang[0][0],self.human_ang[0][1],self.human_ang[0][2])       
        
        self.done = False
        
        return self._get_obs(), {}

    def _get_obs(self):
        return self.human_ang[self.index][0],self.human_ang[self.index][1],self.human_ang[self.index][2],self.human_ee_postion[self.index][0], self.human_ee_postion[self.index][2],self.index

    def step(self, action):
        
        reward = 0

        self.robot_ang = [self.human_ang[self.index][0] + action[0] ,self.human_ang[self.index][1] + action[1] ,self.human_ang[self.index][2] + action[2]]
        self.robot_ee_position = self.get_ee_position(self.robot_ang[0], self.robot_ang[1], self.robot_ang[2])

        if self.index > 0:
            reward -= abs((self.robot_ee_position[0] - self.robot_ee_prev_position[0]) - (self.human_ee_postion[self.index][0]-self.human_ee_postion[self.index - 1][0])) * 1000000
            reward -= abs((self.robot_ee_position[1] - self.robot_ee_prev_position[1]) - (self.human_ee_postion[self.index][2]-self.human_ee_postion[self.index - 1][2])) * 1000000

        reward -= abs(self.robot_ee_position[0] - self.human_ee_postion[self.index][0]) * 10000
        reward -= abs(self.robot_ee_position[1] - self.human_ee_postion[self.index][2]) * 10000

        #reward -= abs(self.robot_ang[0] - self.human_ang[self.index][0]) 
        #reward -= abs(self.robot_ang[1] - self.human_ang[self.index][1]) 
        #reward -= abs(self.robot_ang[2] - self.human_ang[self.index][2])
       
        self.robot_ee_prev_position = self.robot_ee_position

        if self.index == 0:
            reward += 1/(abs(self.robot_ee_position[0] - self.human_ee_postion[0][0]) + 1e-7)
            reward += 1/(abs(self.robot_ee_position[1] - self.human_ee_postion[0][2]) + 1e-7)  

        self.index += 1
    
        if self.index == 149:
            reward += 1/(abs(self.robot_ee_position[0] - self.human_ee_postion[148][0]) + 1e-7)
            reward += 1/(abs(self.robot_ee_position[1] - self.human_ee_postion[148][2]) + 1e-7)
            self.index = 0    
            self.done = True

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

        pygame.draw.circle(self.surf, (0,0,255), ((self.robot_ee_position[0] + 0.75) * self.window_size,(self.robot_ee_position[1]+0.75) * self.window_size), 20)
        
        pygame.draw.circle(self.surf, (0,255,0), ((self.human_ee_postion[self.index][0] + 0.75 ) * self.window_size,(self.human_ee_postion[self.index][2] + 0.75) * self.window_size), 5)
        
           
        self.surf = pygame.transform.flip(self.surf, False, True)

        if self.render_mode:
            assert self.window is not None
            self.window.blit(self.surf, (0, 0))
            pygame.event.pump()
            self.clock.tick(self.fps)
            pygame.display.flip()

    def close(self):
        pass

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
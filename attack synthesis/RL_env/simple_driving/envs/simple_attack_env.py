from cgitb import reset
from socket import socket
import gym
import numpy as np
import math
import pybullet as p
import os
from os import popen
import time
import numpy
# from pyrobolearn import pyrobolearn
from simple_driving.resources.drone import Drone
#from simple_driving.resources.plane import Plane
from simple_driving.resources.goal import Goal
from  simple_driving.resources.criu_controller import terminate_criu, check_pid, get_pid
import matplotlib.pyplot as plt
from pyrobolearn.simulators import Bullet
from pyrobolearn.worlds import BasicWorld
from sklearn.preprocessing import MinMaxScaler
import json
from json.decoder import JSONDecodeError

class SimpleAttackEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, render = False, physical_frames = 100, goal_move = 10, logging=False):
       
        self.action_space = gym.spaces.Discrete(2)
        #The observation space is 23 dimensional and continuous, 
        # X, Y, Z position in WORLD_FRAME (in meters, 3 values)
        # Quaternion orientation in WORLD_FRAME (4 values)
        # Roll, pitch and yaw angles in WORLD_FRAME (in radians, 3 values)
        # The velocity vector in WORLD_FRAME (in m/s, 3 values)
        # Angular velocity in WORLD_FRAME (3 values)
        # Motors' speeds (in RPMs, 4 values)
        # X, Y, Z position of the Goal

        # MIN            
        #   x           y          z         vx         vy         vz         wx  \
        # -34.363537 -355.264848  0.0 -13.412876 -11.786844 -17.955759 -66.292523   
        #  wy         wz          m1   m2    m3     m4  
        # -79.035092 -43.143995  0.0  0.0 -950.0 -950.0  

        #         MAX:            
        # x            y          z         vx           vy        vz         wx  \
        # 36.296095 -267.921035  48.026962  12.328264  8.574444  2.122254  83.197424   
        #  wy         wz        m1     m2   m3   m4  
        # 70.18792  79.238195  950.0  950.0  0.0  0.0  

         #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ       P0            P1            P2            P3     GoalX   GoalY   GoalZ
        # obs_lower_bound = np.array([-np.inf, -np.inf, 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf, -np.inf,   0.,           0.,           0.,           0., -np.inf, -np.inf, 0.    ], dtype=np.float32)
        # obs_upper_bound = np.array([np.inf,  np.inf,  np.inf,  1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,  np.inf,   np.inf,    np.inf,       np.inf,      np.inf, np.inf,  np.inf,  np.inf], dtype=np.float32)
        obs_lower_bound = np.array([-50.,  -400., 0.,     -1., -1., -1., -1., -np.pi, -np.pi, -np.pi, -20., -20., -20., -90., -90., -90.,   0.,           0.,           -950.,           -950., -2*np.pi,-2*np.pi,-2*np.pi, -goal_move-1., -3.34628852e+02 - goal_move, 0.    ], dtype=np.float32)
        obs_upper_bound = np.array([ 50.,   400., 60.,     1.,  1.,  1.,  1.,  np.pi,  np.pi,  np.pi,  20.,  20.,  20.,  90,  90.,  90.,  950.,         950.,               0,              0,   2*np.pi, 2*np.pi, 2*np.pi,  goal_move+1,  -3.34628852e+02 + goal_move,  0.], dtype=np.float32)
        self.observation_space = gym.spaces.box.Box(
            low= obs_lower_bound,#np.array([-1000, -1000, -1000, -7, -7, -7, -20, -20, -20, -1000, -1000, -10], dtype=np.float32),
            high= obs_upper_bound)#np.array([1000, 1000, 1000, 7, 7, 7, 7, 20, 20, 1000, 1000, 10], dtype=np.float32))
        self.np_random, _ = gym.utils.seeding.np_random()
        
        self.render = render
        self.sim = Bullet(render=self.render)
        self.client = self.sim.sim._client

        self.drone = None
        self.goal = None
        self.done = False
        self.prev_dist_to_goal = None
        self.rendered_img = None
        self.render_rot_matrix = None
        self.time_now = 0
        self.physical_frames = physical_frames
        self.goal_move = goal_move
        self.reset_counter = 0
        self.logging = logging
        print(logging)
        if self.logging:
            # self.actions_dict = {'reset_counter':{'actions':[],'goal':[x,y,z],'reward':[]}}
            self.actions_dict = {}
        self.prev_roll = 0
        self.prev_pitch = 0
        self.prev_yaw = 0 
        self.scaler = MinMaxScaler().fit([obs_lower_bound,obs_upper_bound])
        self.flip_occurred = False
        self.flip_counter = 0
        self.prev_motor_off = True

        self.reset()

    def step(self, action):
        # Feed action to the car and get observation of car's state
        ardupilot_alive = self.drone.apply_action(action)
        if not(ardupilot_alive):
            #check if PID ardupilot is up or wait for some time
            for i in range(15):
                self.autotest_pid = get_pid('autotest.py')
                if self.autotest_pid != None:
                    break
                time.sleep(0.01)
            if self.autotest_pid == None:
                print('[INFO] Ardupilot process died')
                self.done = True
        #we remove this because the enviroment is updated withing the action
        #p.stepSimulation()
        drone_ob = self.drone.get_observation()

        roll_diff = drone_ob[7] - self.prev_roll
        pitch_diff = drone_ob[8] - self.prev_pitch
        yaw_diff = drone_ob[9] - self.prev_yaw

        
        self.prev_roll = drone_ob[7]
        self.prev_pitch = drone_ob[8]
        self.prev_yaw = drone_ob[9]

        # print('drone_OBJ:', drone_ob)
        # Compute reward as L2 change in distance to goal
        dist_to_goal = math.sqrt(((drone_ob[0] - self.goal[0]) ** 2 
                                  ))

        delta_x = self.prev_dist_to_goal - dist_to_goal #max((self.prev_dist_to_goal - dist_to_goal), 0)
        
        self.prev_dist_to_goal = dist_to_goal
        #print('distance to goal: ',dist_to_goal)

        #done if the drone hitted the ground
        if (drone_ob[2] <= 0.5):
            print('[INFO]: drone hits the ground')
            self.done = True



        if numpy.dot(self.drone.drone.up_vector, [0,0,1]) >= 0:
            if np.sign(delta_x) >= 0:
                reward = 1.5
            else:
                reward = -0.5
        else:
            if np.sign(delta_x) >= 0:
                reward = 0.5
            else:
                reward = -1.5

        self.flip_occurred = any(np.absolute([roll_diff, pitch_diff, yaw_diff])>=np.pi)
        self.is_flipping = any(np.absolute(drone_ob[13:16])>=5)

        if self.is_flipping:
            #print('detected high angular velocities')
            if self.flip_occurred:
                #print('flip occurred')
                self.flip_counter += 1
            if self.flip_counter > 1:
                reward = -self.flip_counter
        else:
            #print('reset flip counter')
            self.flip_counter = 0


        #drone too close the desired position of the controller (we want to go away from here)
        if np.abs(drone_ob[0]) < 0.1:
            reward = reward - 2

        self.motor_off = not(any(drone_ob[16:20]))
        if self.motor_off and not(self.prev_motor_off):
            print('[INFO] Fail safe enabled')
            reward = -40
            self.done=True
        self.prev_motor_off = self.motor_off

        #upperbound y
        if drone_ob[1] <= -400:
            reward = -100
            self.done = True

        if dist_to_goal < 1:
            reward = 100
            self.done = True

        ob = np.array(drone_ob + tuple([roll_diff]) + tuple([pitch_diff]) + tuple([yaw_diff]) + self.goal, dtype=np.float32)
        ob = self.scaler.transform([ob])
        if self.logging:
            self.actions_dict[str(self.reset_counter)]['actions'].append(int(action))
            self.actions_dict[str(self.reset_counter)]['rewards'].append(reward)
            self.actions_dict[str(self.reset_counter)]['drone_ob'].append(drone_ob + tuple([roll_diff]) + tuple([pitch_diff]) + tuple([yaw_diff]))
        #print(ob)
        return ob, reward, self.done, dict()

    def seed(self, seed=None):
        self.np_random, seed = gym.utils.seeding.np_random(seed)
        return [seed]

    def reset(self):
        #try:
        self.autotest_pid = terminate_criu('autotest.py')
        if self.autotest_pid != None:
            while check_pid(self.autotest_pid):
                pass
            print('[INFO] Killed ardupilot')
        else:
            print('[INFO] No ardupilot process to kill')
        # except:
        #     print('[INFO] No ardupilot process to kill')

        p.resetSimulation(self.client)
        #close socket
        try:
            self.drone.sock.close()
        except:
            print("[INFO]Attempted closing Ardupilot socket: No socket to close")
        self.world = BasicWorld(self.sim)#, scaling=100)
        self.drone = Drone(self.sim, self.world, self.physical_frames)
        self.drone.init()


        # Set the goal to a random target
        base_x = -7.69452680e-03
        base_y = -3.34628852e+02
        x =(self.np_random.uniform(base_x-self.goal_move,base_x+self.goal_move))# if self.np_random.randint(2) else
            #self.np_random.uniform(-5, -9)) #(-7.69452680e-03) 
        x = x + np.sign(x)*10
        
        y = (self.np_random.uniform(base_y, base_y-2*self.goal_move))#(-3.34628852e+02)#(self.np_random.uniform(5, 9) if self.np_random.randint(2) else
             #self.np_random.uniform(-5, -9))
        z = (0)#(self.np_random.uniform(5, 9) if self.np_random.randint(2) else
             #self.np_random.uniform(-5, -9))
        print('++++++++++++')
        print('goal X = '+str(x))
        print('goal Y = '+str(y))
        print('++++++++++++')
        self.goal = (x, y, z)
        #print(self.goal)
        self.done = False

        # Visual element of the goal
        Goal(self.client, self.goal)
        #cocentric cyrcles around the goal
        if self.render:
            self.world.load_visual_cylinder(position= self.goal, radius=10, height = 0.1, color = (1,0,0,1) )
            self.world.load_visual_cylinder(position= self.goal, radius=9, height = 0.15, color =  (0,1,0,1) )
            self.world.load_visual_cylinder(position= self.goal, radius=8, height = 0.20, color =  (0,0,1,1) )
            self.world.load_visual_cylinder(position= self.goal, radius=7, height = 0.25, color =  (1,0,0,1) )
            self.world.load_visual_cylinder(position= self.goal, radius=6, height = 0.30, color =  (0,1,0,1) )
            self.world.load_visual_cylinder(position= self.goal, radius=5, height = 0.35, color =  (0,0,1,1) )
            self.world.load_visual_cylinder(position= self.goal, radius=4, height = 0.40, color =  (1,0,0,1) )
            self.world.load_visual_cylinder(position= self.goal, radius=3, height = 0.45, color =  (0,1,0,1) )
            self.world.load_visual_cylinder(position= self.goal, radius=2, height = 0.50, color =  (0,0,1,1) )
        print('[INFO] Reloaded world')

        # Get observation to return
        drone_ob = self.drone.get_observation()

        self.prev_dist_to_goal = math.sqrt(((drone_ob[0] - self.goal[0]) ** 2 #+
                                #   (drone_ob[1] - self.goal[1]) ** 2+
                                #   (drone_ob[2] - self.goal[2]) **2
                                  ))
       
    
        path3 = os.path.join(os.path.dirname(__file__), '../../../../restore_ardupilot.sh')
        #print(path2)
        ret3=popen(path3)
        print('[INFO] Ardupilot loaded')
        #print(ret2)
        if self.logging:
            print('open file')
            with open('result.json', 'w+') as fp:
                json.dump(self.actions_dict, fp, indent = 4)
            self.reset_counter = self.reset_counter + 1
            self.actions_dict[str(self.reset_counter)] = {}
            self.actions_dict[str(self.reset_counter)]['goal'] = list(self.goal)
            self.actions_dict[str(self.reset_counter)]['actions'] = []
            self.actions_dict[str(self.reset_counter)]['rewards'] = []
            self.actions_dict[str(self.reset_counter)]['drone_ob'] = []
        return np.array(drone_ob + tuple([0,0,0]) + self.goal, dtype=np.float32)

    def render(self, mode='human'):
        if self.rendered_img is None:
            self.rendered_img = plt.imshow(np.zeros((100, 100, 4)))

        # Base information
        drone_id, client_id = self.drone.get_ids()
        proj_matrix = p.computeProjectionMatrixFOV(fov=80, aspect=1,
                                                   nearVal=0.01, farVal=100)
        pos, ori = [list(l) for l in
                    p.getBasePositionAndOrientation(drone_id, client_id)]
        pos[2] = 0.2

        # Rotate camera direction
        rot_mat = np.array(p.getMatrixFromQuaternion(ori)).reshape(3, 3)
        camera_vec = np.matmul(rot_mat, [1, 0, 0])
        up_vec = np.matmul(rot_mat, np.array([0, 0, 1]))
        view_matrix = p.computeViewMatrix(pos, pos + camera_vec, up_vec)

        # Display image
        frame = p.getCameraImage(100, 100, view_matrix, proj_matrix)[2]
        frame = np.reshape(frame, (100, 100, 4))
        self.rendered_img.set_data(frame)
        plt.draw()
        plt.pause(.00001)

    def close(self):
        p.disconnect(self.client)

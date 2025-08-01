import pybullet as p
import os
import math
import socket
import time
import struct
import json
import numpy
from os import popen
# use pymavlink for ArduPilot convention transformations
from pymavlink.rotmat import Vector3, Matrix3
from pymavlink.quaternion import Quaternion

from pyrobolearn.robots import Quadcopter


class Drone:
    def __init__(self, sim, world, physical_frames =100):
        self.sim = sim
        self.world = world
        f_name = os.path.join(os.path.dirname(__file__), '../../../models/iris/iris.urdf')
        self.drone = Quadcopter(self.sim, urdf=f_name)
        self.dronebullet  = self.drone.id
        self.client = self.sim.sim._client
        
        self.motor_dir = [ 1, 1, -1, -1 ]
        self.motor_order = [ 0, 1, 2, 3 ]

        self.motors = None
        self.motor_speed = [ 0 ] * 4
        
        self.RATE_HZ = 1000
        self.TIME_STEP = 1.0 / self.RATE_HZ
        self.GRAVITY_MSS = 9.80665
        
        self.drone.print_info()
        self.sim.set_time_step(self.TIME_STEP)
        
        self.old_gyro = None
        self.old_euler = None
        self.old_accel = None
        
        
        ###SOCKET###
        self.time_now = 0
        self.last_velocity = None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', 9002))
        self.sock.settimeout(0.1)

        self.last_SITL_frame = -1
        self.connected = False
        self.frame_count = 0
        self.frame_time = time.time()
        self.print_frame_count = 1000

        self.move_accel = 0.0
        self.last_move = time.time()

        self.last_message = None
        self.physical_frames = physical_frames


    def get_ids(self):
        return self.dronebullet, self.client
    
    def constrain(self,v,min_v,max_v):
        '''constrain a value'''
        if v < min_v:
            v = min_v
        if v > max_v:
            v = max_v
        return v

    def apply_action(self, action):
        ATTACK_ON = False
        #print(action)
        if action == 1:
            ATTACK_ON = True
        #fail_counter = 0
        ardupilot_alive = True
        for i in range(self.physical_frames):
            signal = self.ardupilot_and_physical_frame(ATTACK_ON)
            if signal == 'No_message':
                ardupilot_alive = False
                return ardupilot_alive

        return ardupilot_alive


    def get_observation(self):

        #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ       P0            P1            P2            P3
        # X, Y, Z position in WORLD_FRAME (in meters, 3 values)
        # Quaternion orientation in WORLD_FRAME (4 values)
        # Roll, pitch and yaw angles in WORLD_FRAME (in radians, 3 values)
        # The velocity vector in WORLD_FRAME (in m/s, 3 values)
        # Angular velocity in WORLD_FRAME (3 values)
        # Motors' speeds (in RPMs, 4 values)

        # Get the position and orientation of the drone in the simulation
        pos, quaternion = p.getBasePositionAndOrientation(self.dronebullet, self.client)
        # print(ang)
        ang = p.getEulerFromQuaternion(quaternion)
        
        # Get the velocity of the drone
        vel, ang_vel = p.getBaseVelocity(self.dronebullet, self.client)

        # print(pos)
        # print(quaternion)
        # print(ang)
        # print(vel)
        # print(ang_vel)
        # print(self.motor_speed)
        observation = (pos + quaternion + ang + vel + ang_vel + tuple(self.motor_speed))
        return observation


    def control_quad(self, pwm):
        '''control quadcopter'''
        
        self.motors = pwm[0:4]
        
        for m in range(len(self.motors)):
            self.motor_speed[self.motor_order[m]] = self.constrain(self.motors[m] - 1000.0, 0, 1000) * self.motor_dir[self.motor_order[m]]

        self.drone.set_propeller_velocities(self.motor_speed)

        
    def init(self):
        self.time_now = 0
        self.drone.position = [0,0,0]
        self.drone.orientation = [0,0,0,1]

        self.time_now= 99.99999999991718
        self.drone.position =  [1.27328546e-02, -2.67921035e+02,  4.80269622e+01]
        self.drone.orientation =  [ 0.14939551,  0.14670488, -0.68295308,  0.6998098]
        self.drone.linear_velocity = [-1.23827095e-02, -1.00324182e+01, -4.66749270e-03]

        print('time now: {}'.format(self.time_now))
        print('R. Pos: {}'.format(self.drone.position))
        print('R. Or: {}'.format(self.drone.orientation))
    

    def physics_step(self, pwm_in):

        self.control_quad(pwm_in)

        self.world.follow(self.drone)

        self.world.step(sleep_dt=0)

        self.time_now += self.TIME_STEP

        # get the position orientation and velocity
        quaternion = quaternion_to_AP(self.drone.orientation)
        roll, pitch, yaw = quaternion.euler
        velocity = vector_to_AP(self.drone.linear_velocity)
        position = vector_to_AP(self.drone.position)

        # get ArduPilot DCM matrix (rotation matrix)
        dcm = quaternion.dcm

        # get gyro vector in body frame
        gyro = dcm.transposed() * vector_to_AP(self.drone.angular_velocity)
        
        # calculate acceleration
        #global last_velocity
        if self.last_velocity is None:
            self.last_velocity = velocity

        accel = (velocity - self.last_velocity) * (1.0 / self.TIME_STEP)
        self.last_velocity = velocity

        # add in gravity in earth frame
        accel.z -= self.GRAVITY_MSS

        # convert accel to body frame
        accel = dcm.transposed() * accel

        # convert to tuples
        accel = to_tuple(accel)
        gyro = to_tuple(gyro)
        position = to_tuple(position)
        velocity = to_tuple(velocity)
        euler = (roll, pitch, yaw)


        return self.time_now,gyro,accel,position,euler,velocity

    def ardupilot_and_physical_frame(self, ATTACK_ON):

        try:
            data,address = self.sock.recvfrom(100)
        except Exception as ex:
            print('[INFO] No JSON messages received')
            time.sleep(0.01)
            return 'No_message'

        parse_format = 'HHI16H'
        magic = 18458

        if len(data) != struct.calcsize(parse_format):
            print("got packet of len %u, expected %u" % (len(data), struct.calcsize(parse_format)))
            return 'ok'


        decoded = struct.unpack(parse_format,data)
        #print(decoded)

        if magic != decoded[0]:
            print("Incorrect protocol magic %u should be %u" % (decoded[0], magic))
            return 'ok'

        frame_rate_hz = decoded[1]
        frame_count = decoded[2]
        pwm = decoded[3:]
        

        # print(self.last_SITL_frame)
        # print(frame_count)

        if frame_rate_hz != self.RATE_HZ:
            print("New frame rate %u" % frame_rate_hz)
            self.RATE_HZ = frame_rate_hz
            self.TIME_STEP = 1.0 / self.RATE_HZ
            self.sim.set_time_step(self.TIME_STEP)

        # Check if the fame is in expected order
        if frame_count < self.last_SITL_frame:
            #Controller has reset, reset physics also
           #self.init()
           print('Controller reset')
        elif frame_count == self.last_SITL_frame:
            # duplicate frame, skip
            print('Duplicate input frame')
            #self.sock.sendto(bytes(self.last_message,"ascii"), address)
            return 'Duplicate'
        elif frame_count != self.last_SITL_frame + 1 and self.connected:
            print('[{1}]Missed {0} input frames'.format(frame_count - self.last_SITL_frame - 1, frame_count))
        self.last_SITL_frame = frame_count

        if not self.connected:
            self.connected = True
            print('Connected to {0}'.format(str(address)))
        frame_count += 1

        # physics time step
        phys_time,gyro,accel,pos,euler,velo = self.physics_step(pwm)


        if ATTACK_ON:
            if self.old_gyro == None:
                self.old_gyro = gyro
            if self.old_accel == None:
                self.old_accel = accel
            if self.old_euler == None:
                self.old_euler = euler
            gyro = self.old_gyro
            accel = self.old_accel
            euler = self.old_euler

        else:
            self.old_gyro = gyro
            self.old_accel = accel
            self.old_euler = euler


        # build JSON format
        IMU_fmt = {
            "gyro" : gyro,
            "accel_body" : accel
        }
        JSON_fmt = {
            "timestamp" : phys_time,
            "imu" : IMU_fmt,
            "position" : pos,
            "attitude" : euler,
            "velocity" : velo
        }
        JSON_string = "\n" + json.dumps(JSON_fmt,separators=(',', ':')) + "\n"

        # Send to AP
        self.sock.sendto(bytes(JSON_string,"ascii"), address)
        self.last_message = JSON_string

        # print(pwm)
        # print(self.last_message)
        return 'ok'
        
        
##CONVERSION FUNCTIONS##
def quaternion_to_AP(quaternion):
    '''convert pybullet quaternion to ArduPilot quaternion'''
    return Quaternion([quaternion[3], quaternion[0], -quaternion[1], -quaternion[2]])

def vector_to_AP(vec):
    '''convert pybullet vector tuple to ArduPilot Vector3'''
    return Vector3(vec[0], -vec[1], -vec[2])

def quaternion_from_AP(q):
    '''convert ArduPilot quaternion to pybullet quaternion'''
    return [q.q[1], -q.q[2], -q.q[3], q.q[0]]

def to_tuple(vec3):
    '''convert a Vector3 to a tuple'''
    return (vec3.x, vec3.y, vec3.z)









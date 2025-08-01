# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
import logging
import sys
import time
from threading import Event
import pandas as pd

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

attack_started = 0
deck_attached_event = Event()

logging.basicConfig(level=logging.ERROR)

position_estimate = [0, 0]


# def move_box_limit(scf):
#     with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
#         body_x_cmd = 0.2
#         body_y_cmd = 0.1
#         max_vel = 0.2

#         while (1):
#             '''if position_estimate[0] > BOX_LIMIT:
#                 mc.start_back()
#             elif position_estimate[0] < -BOX_LIMIT:
#                 mc.start_forward()
#             '''

#             if position_estimate[0] > BOX_LIMIT:
#                 body_x_cmd = -max_vel
#             elif position_estimate[0] < -BOX_LIMIT:
#                 body_x_cmd = max_vel
#             if position_estimate[1] > BOX_LIMIT:
#                 body_y_cmd = -max_vel
#             elif position_estimate[1] < -BOX_LIMIT:
#                 body_y_cmd = max_vel

#             mc.start_linear_motion(body_x_cmd, body_y_cmd, 0)

#             time.sleep(0.1)


# def move_linear_simple(scf):
#     with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
#         time.sleep(1)
#         mc.forward(0.5)
#         time.sleep(1)
#         mc.turn_left(180)
#         time.sleep(1)
#         mc.forward(0.5)
#         time.sleep(1)


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(40)
        mc.stop()


def log_pos_callback(timestamp, data, logconf):
    dataframe = pd.DataFrame(columns=['timestamp', 'stateEstimate.x','stateEstimate.y','stateEstimate.z','stabilizer.roll', 'stabilizer.pitch',	'stabilizer.yaw'])
    # print(f'[{timestamp}][{logconf.name}]: ', end='')
    dataframe['timestamp'] = [timestamp]
    for name, value in data.items():
        # print(f'{name}: {value:3.3f} ', end='')
        dataframe[name] = [value]
    # print()
    dataframe.to_csv('last_state.csv', index=False)
    # global position_estimate
    # position_estimate[0] = data['stateEstimate.x']
    # position_estimate[1] = data['stateEstimate.y']

def log_callback(timestamp, data, logconf):
    """Callback from a the log API when data arrives"""
    dataframe = pd.DataFrame(columns=['timestamp', 'acc.x',	'acc.y',	'acc.z',	'gyro.x',	'gyro.y',	'gyro.z', 'groundtruth'])
    # print(f'[{timestamp}][{logconf.name}]: ', end='')
    dataframe['timestamp'] = [timestamp]
    for name, value in data.items():
        # print(f'{name}: {value:3.3f} ', end='')
        dataframe[name] = [value]
    # print()
    dataframe['groundtruth'] = [attack_started]
    dataframe.to_csv('last_IMU.csv', index=False)

def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')
        
def console_callback(text: str):
    '''A callback to run when we get console text from Crazyflie'''
    global attack_started
    # We do not add newlines to the text received, we get them from the
    # Crazyflie at appropriate places.
    if "attack" in text:
        if "start" in text:
            attack_started = 1
        if "stop" in text:
            attack_started = 0
    print(text, end='')

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        scf.cf.console.receivedChar.add_callback(console_callback)
        time.sleep(1)

        logconf = LogConfig(name='Position', period_in_ms=10)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        logconf.add_variable('stabilizer.roll', 'float')
        logconf.add_variable('stabilizer.pitch', 'float')
        logconf.add_variable('stabilizer.yaw', 'float')
        logconf2 = LogConfig(name='Acc/Gyro', period_in_ms=10)
        logconf2.add_variable('acc.x', 'float')
        logconf2.add_variable('acc.y', 'float')
        logconf2.add_variable('acc.z', 'float')
        logconf2.add_variable('gyro.x', 'float')
        logconf2.add_variable('gyro.y', 'float')
        logconf2.add_variable('gyro.z', 'float')
        
        scf.cf.log.add_config(logconf)
        scf.cf.log.add_config(logconf2)
        
        logconf.data_received_cb.add_callback(log_pos_callback)
        logconf2.data_received_cb.add_callback(log_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf.start()
        logconf2.start()
        take_off_simple(scf)
        # move_linear_simple(scf)
        # move_box_limit(scf)
        #time.sleep(50)
        logconf.stop()
        logconf2.stop()
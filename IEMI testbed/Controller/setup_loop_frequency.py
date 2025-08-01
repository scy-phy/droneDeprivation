#  Communication to MPU6050
#  Based on tutorial: https://openest.io/en/services/mpu6050-accelerometer-on-raspberry-pi/

from smbus2 import SMBus
import time

import socket


# Registers
SMPLRT_DIV = 0x19
CONFIG = 0x1A
ACCEL_CONFIG = 0x1C
FIFO_EN = 0x23
INT_STATUS = 0x3A
GYRO_XOUT_H = 0x43
USER_CTRL = 0x6A
PWR_MGMT_1 = 0x6B
PWR_MGMT_2 = 0x6C
FIFO_R_W = 0x74
FIFO_COUNTH = 0x72

def get_fifo_speed():
    prev_fifo_len = 0
    t0 = time.time()
    for ix in range(1000):
        # acc1 = i2cbus.read_byte_data(IMU_address,FIFO_R_W)
        fifo_len = i2cbus.read_word_data(IMU_address,FIFO_COUNTH)
        sampling_rate = i2cbus.read_byte_data(IMU_address,SMPLRT_DIV)
        interrupt_status = i2cbus.read_byte_data(IMU_address,INT_STATUS)
        if (ix < 10):
            print('fifo len: ', fifo_len)
            fifo_delta = fifo_len-prev_fifo_len
            t_delta = time.time()-t0
            print('[{0:0.4f}] length delta: {1}'.format(t_delta,fifo_delta))
            print('Interrupt: ', interrupt_status)
        if (ix % 100 == 0):
            print('fifo len: ', fifo_len)
            print('Sampling rate: ', sampling_rate)
            print('Interrupt: ', interrupt_status)
        t0 = time.time()
        prev_fifo_len = fifo_len



i2cbus = SMBus(1)
IMU_address = 0x68  # MPU6050 I2C Address

# Setup power mode
i2cbus.write_byte_data(IMU_address, PWR_MGMT_1, 0x01)
i2cbus.write_byte_data(IMU_address, PWR_MGMT_2, 0x00)

# Setup accelerometer
i2cbus.write_byte_data(IMU_address, ACCEL_CONFIG, 0x00)

# Setup sampling rate and config
i2cbus.write_byte_data(IMU_address, CONFIG, 0x00)
i2cbus.write_byte_data(IMU_address, SMPLRT_DIV, 0x07)

# Setup FIFO
i2cbus.write_byte_data(IMU_address, USER_CTRL, 0x44)
i2cbus.write_byte_data(IMU_address, FIFO_EN, 0x08)

print('IMU configured')
time.sleep(5)
print('Starting loop')
write_done = False
t1 = time.time()
# for ix in range(50):
while(True):
    if time.time()-t1 > 1000:
        break

    t0 = time.time()
    try:
        i2cbus.write_byte_data(IMU_address, SMPLRT_DIV, 0x00)
        write_done = True
        print('write done')
        if write_done:
            sampling_rate = i2cbus.read_byte_data(IMU_address,SMPLRT_DIV)
            print('read: '+str(sampling_rate))
            if sampling_rate != 0:
                print('exit condition, sampling rate: '+ str(sampling_rate))
                break
    except Exception as e:
        print('Error: ', e)
    write_done = False
    
    time.sleep(0.1)
print("stop now the injection")
print('Sampling rate: ', sampling_rate)

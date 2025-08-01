#  Communication to MPU6050
#  Based on tutorial: https://openest.io/en/services/mpu6050-accelerometer-on-raspberry-pi/

from smbus2 import SMBus
import time

# Registers
SMPLRT_DIV = 0x19
CONFIG = 0x1A
ACCEL_CONFIG = 0x1C
FIFO_EN = 0x23
USER_CTRL = 0x6A
PWR_MGMT_1 = 0x6B
FIFO_R_W = 0x74
FIFO_COUNTH = 0x72


i2cbus = SMBus(1)
IMU_address = 0x68  # MPU6050 I2C Address


def launch_suspend_attack(i2cbus):
    i2cbus.write_byte_data(IMU_address, PWR_MGMT_1, 0x40)


t1 = time.time()
attack = True

while(True):
    if time.time()-t1 >50:
        break

    try:
        launch_suspend_attack(i2cbus)
        
    except Exception as e:
        print('Error: ',e)
    attack^=1

    time.sleep(0.005)
 

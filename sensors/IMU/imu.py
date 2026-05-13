from mpu9250_jmdev.mpu_9250 import MPU9250
from mpu9250_jmdev.registers import *
import time

"""
1. sudo apt install python3-smbus i2c-tools
2. pip3 install smbus2
3. sudo i2cdetect -y 1
"""

mpu = MPU9250(
    address_ak=AK8963_ADDRESS,
    address_mpu_master=MPU9050_ADDRESS_68,
    address_mpu_slave=None,
    bus=1,
    gfs=GFS_250,
    afs=AFS_2G,
    mfs=AK8963_BIT_16,
    mode=AK8963_MODE_C8HZ
)

# we might have to increase frequency
mpu.configureMPU6500(GFS_250, AFS_2G)

while True:
    accel = mpu.readAccelerometerMaster()
    gyro = mpu.readGyroscopeMaster()

    print("Accel:", accel)
    print("Gyro :", gyro)
    print("----------------")

    time.sleep(0.5)
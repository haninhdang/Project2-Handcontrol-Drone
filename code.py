import time
import board
import adafruit_lis3mdl
from adafruit_lis3mdl import LIS3MDL, Rate, PerformanceMode,Range
i2c = board.I2C()  # uses board.SCL and board.SDA
magnetic_sensor = adafruit_lis3mdl.LIS3MDL(i2c)
magnetic_sensor.range = Range.RANGE_12_GAUSS

from adafruit_lsm6ds.lsm6dsox import LSM6DSOX
from adafruit_lsm6ds import Rate, AccelRange, GyroRange
accel_gyro_sensor = LSM6DSOX(i2c)
accel_gyro_sensor.accelerometer_range = AccelRange.RANGE_8G
accel_gyro_sensor.gyro_range = GyroRange.RANGE_2000_DPS
accel_gyro_sensor.accelerometer_data_rate = Rate.RATE_1_66K_HZ
accel_gyro_sensor.gyro_data_rate = Rate.RATE_1_66K_HZ

while True:
    gyro_x, gyro_y, gyro_z = accel_gyro_sensor.gyro
    acc_x, acc_y, acc_z = accel_gyro_sensor.acceleration
    mag_x, mag_y, mag_z = magnetic_sensor.magnetic
    sensor_data=[gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, mag_x, mag_y, mag_z]
    print(*sensor_data)
    time.sleep(0.1)

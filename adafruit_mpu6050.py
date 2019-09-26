# The MIT License (MIT)
#
# Copyright (c) 2019 Bryan Siepert for Adafruit Industries
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""
`adafruit_mpu6050`
================================================================================

CircuitPython helper library for the MPU6050 6-axis Accelerometer and Gyroscope


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s). Use unordered list & hyperlink rST
   inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies based on the library's use of either.

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

# imports

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MPU6050.git"

from time import sleep
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from adafruit_register.i2c_struct_array import StructArray
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits
import adafruit_bus_device.i2c_device as i2c_device

# TODO: Trust but verify
_MPU6050_DEFAULT_ADDRESS  = 0x69  # MPU6050 default i2c address w/ AD0 high
_MPU6050_DEVICE_ID        = 0x68 # The correct MPU6050_WHO_AM_I value

_MPU6050_SELF_TEST_X      = 0x0D # Self test factory calibrated values register
_MPU6050_SELF_TEST_Y      = 0x0E # Self test factory calibrated values register
_MPU6050_SELF_TEST_Z      = 0x0F # Self test factory calibrated values register
_MPU6050_SELF_TEST_A      = 0x10 # Self test factory calibrated values register
_MPU6050_SMPLRT_DIV       = 0x19 # sample rate divisor register
_MPU6050_CONFIG           = 0x1A # General configuration register
_MPU6050_GYRO_CONFIG      = 0x1B # Gyro specfic configuration register
_MPU6050_ACCEL_CONFIG     = 0x1C # Accelerometer specific configration register
_MPU6050_INT_PIN_CONFIG   = 0x37 # Interrupt pin configuration register
_MPU6050_ACCEL_OUT        = 0x3B # base address for sensor data reads
_MPU6050_TEMP_H           = 0x41 # Temperature data high byte register
_MPU6050_TEMP_L           = 0x42 # Temperature data low byte register
_MPU6050_USER_CTRL        = 0x6A # FIFO and I2C Master control register
_MPU6050_PWR_MGMT_1       = 0x6B # Primary power/sleep control register
_MPU6050_PWR_MGMT_2       = 0x6C # Secondary power/sleep control register
_MPU6050_WHO_AM_I         = 0x75 # Divice ID register

STANDARD_GRAVITY = 9.80665
# typedef enum fsync_out {
#   MPU6050_FSYNC_OUT_DISABLED,
#   MPU6050_FSYNC_OUT_TEMP,
#   MPU6050_FSYNC_OUT_GYROX,
#   MPU6050_FSYNC_OUT_GYROY,
#   MPU6050_FSYNC_OUT_GYROZ,
#   MPU6050_FSYNC_OUT_ACCELX,
#   MPU6050_FSYNC_OUT_ACCELY,
#   MPU6050_FSYNC_OUT_ACCEL_Z,
# } mpu6050_fsync_out_t;

# /**
#  * @brief Clock source options
#  *
#  * Allowed values for `setClock`.
#  */
# typedef enum clock_select {
#   MPU6050_INTR_8MHz,
#   MPU6050_PLL_GYROX,
#   MPU6050_PLL_GYROY,
#   MPU6050_PLL_GYROZ,
#   MPU6050_PLL_EXT_32K,
#   MPU6050_PLL_EXT_19MHz,
#   MPU6050_STOP = 7,
# } mpu6050_clock_select_t;

# /**
#  * @brief Accelerometer range options
#  *
#  * Allowed values for `setAccelerometerRange`.
#  */
# typedef enum gyro_range {
MPU6050_RANGE_2_G = 0b00  # +/- 2g (default value)
MPU6050_RANGE_4_G = 0b01  # +/- 4g
MPU6050_RANGE_8_G = 0b10  # +/- 8g
MPU6050_RANGE_16_G = 0b11 # +/- 16g
# } mpu6050_accel_range_t;

# /**
#  * @brief Gyroscope range options
#  *
#  * Allowed values for `setGyroRange`.
#  */
# typedef enum gyro_range {
MPU6050_RANGE_250_DEG = 0  # +/- 250 deg/s (default value)
MPU6050_RANGE_500_DEG = 1  # +/- 500 deg/s
MPU6050_RANGE_1000_DEG = 2 # +/- 1000 deg/s
MPU6050_RANGE_2000_DEG = 3 # +/- 2000 deg/s
# } mpu6050_gyro_range_t;

# /**
#  * @brief Digital low pass filter bandthwidth options
#  *
#  * Allowed values for `setFilterBandwidth`.
#  */
# typedef enum bandwidth {
#   MPU6050_BAND_260_HZ, # Docs imply this disables the filter
#   MPU6050_BAND_184_HZ, # 184 Hz
#   MPU6050_BAND_94_HZ,  # 94 Hz
#   MPU6050_BAND_44_HZ,  # 44 Hz
#   MPU6050_BAND_21_HZ,  # 21 Hz
#   MPU6050_BAND_10_HZ,  # 10 Hz
#   MPU6050_BAND_5_HZ,   # 5 Hz
# } mpu6050_bandwidth_t;

# /**
#  * @brief Periodic measurement options
#  *
#  * Allowed values for `setCycleRate`.
#  */
# typedef enum cycle_rate {
#   MPU6050_CYCLE_1_25_HZ, # 1.25 Hz
#   MPU6050_CYCLE_5_HZ,    # 5 Hz
#   MPU6050_CYCLE_20_HZ,   # 20 Hz
#   MPU6050_CYCLE_40_HZ,   # 40 Hz
# } mpu6050_cycle_rate_t;
class MPU6050:

    def __init__(self, i2c_bus, address=_MPU6050_DEFAULT_ADDRESS):
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        if self._device_id != _MPU6050_DEVICE_ID:
            raise RuntimeError("Failed to find MPU6050 - check your wiring!")

        self.reset()

    def reset(self):
        self._reset = True
        while self._reset is True:
            sleep(0.10)

        self.sample_rate_divisor = 0
        self._gyro_range = 2
        # self._accel_range = MPU6050_RANGE_8_G
        self._filter_bandwidth = 0
        self._clock_source = 1
        sleep(0.1)
        self.sleep = False


    _device_id = UnaryStruct(_MPU6050_WHO_AM_I, ">B")
    _reset = RWBit(_MPU6050_PWR_MGMT_1, 7, 1)
    sample_rate_divisor = UnaryStruct(_MPU6050_SMPLRT_DIV, ">B")
    _gyro_range = RWBits(2, _MPU6050_GYRO_CONFIG, 3)
    _accel_range = RWBits(2,_MPU6050_ACCEL_CONFIG, 3)
    _filter_bandwidth = RWBits(2, _MPU6050_CONFIG, 3)
    _clock_source = RWBits(3, _MPU6050_PWR_MGMT_1, 0)
    sleep = RWBit(_MPU6050_PWR_MGMT_1, 6, 1)
    _raw_data_array = StructArray(_MPU6050_ACCEL_OUT, ">h", 7)

    @property
    def temperature(self):
        """docs"""
        raw_temperature = self._raw_data_array[3][0]
        temp = (raw_temperature + 12412.0) / 340.0
        return temp

    @property
    def acceleration(self):
        raw_data = self._raw_data_array
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]
        
        accel_range = self._accel_range
        accel_scale = 1
        if (accel_range == MPU6050_RANGE_16_G):
            accel_scale = 2048
        if (accel_range == MPU6050_RANGE_8_G):
            accel_scale = 4096
        if (accel_range == MPU6050_RANGE_4_G):
            accel_scale = 8192
        if (accel_range == MPU6050_RANGE_2_G):
            accel_scale = 16384

        # setup range dependant scaling
        accel_x = (raw_x / accel_scale) * STANDARD_GRAVITY
        accel_y = (raw_y / accel_scale) * STANDARD_GRAVITY
        accel_z = (raw_z / accel_scale) * STANDARD_GRAVITY
        
        return (accel_x, accel_y, accel_z)

    @property
    def gyro(self):
        raw_data = self._raw_data_array
        raw_x = raw_data[4][0]
        raw_y = raw_data[5][0]
        raw_z = raw_data[6][0]

        gyro_scale = 1
        gyro_range = self._gyro_range
        if gyro_range == MPU6050_RANGE_250_DEG:
            gyro_scale = 131
        if gyro_range == MPU6050_RANGE_500_DEG:
            gyro_scale = 65.5
        if gyro_range == MPU6050_RANGE_1000_DEG:
            gyro_scale = 32.8
        if gyro_range == MPU6050_RANGE_2000_DEG:
            gyro_scale = 16.4

        # setup range dependant scaling
        gyro_x = (raw_x / gyro_scale)
        gyro_y = (raw_y / gyro_scale)
        gyro_z = (raw_z / gyro_scale)
        
        return (gyro_x, gyro_y, gyro_z)

#   mpu6050_gyro_range_t gyro_range = getGyroRange();

#   float gyro_scale = 1;
#   if (gyro_range == MPU6050_RANGE_250_DEG)
#     gyro_scale = 131;
#   if (gyro_range == MPU6050_RANGE_500_DEG)
#     gyro_scale = 65.5;
#   if (gyro_range == MPU6050_RANGE_1000_DEG)
#     gyro_scale = 32.8;
#   if (gyro_range == MPU6050_RANGE_2000_DEG)
#     gyro_scale = 16.4;

#   gyroX = rawGyro / gyro_scale;
#   gyroY = rawGyro / gyro_scale;
#   gyroZ = rawGyro / gyro_scale;
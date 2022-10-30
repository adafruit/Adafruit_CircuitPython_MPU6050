# SPDX-FileCopyrightText: 2019 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_mpu6050`
================================================================================

CircuitPython helper library for the MPU6050 6-DoF Accelerometer and Gyroscope

WARNING: Reference(s) to "Master" / "Slave" used only to match original documentation
         by the manufacturer, TDK Invnesense.  Please feel free to rename these to
         something more agreeable like Leader/Follower if you feel so inclined.  


* Author(s): Bryan Siepert

Implementation Notes
--------------------

**Hardware:**

* `Adafruit MPU-6050 6-DoF Accel and Gyro Sensor
  <https://www.adafruit.com/product/3886>`_ (Product ID: 3886)

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://circuitpython.org/downloads

* Adafruit's Bus Device library:
  https://github.com/adafruit/Adafruit_CircuitPython_BusDevice

* Adafruit's Register library:
  https://github.com/adafruit/Adafruit_CircuitPython_Register
  
* Registers added without seeing concise documenation on their
  implementation.  Please use at your own risk until further
  development allows for safe usage.
  
  ** Datasheets, Register Maps, etc... **
  
* Datasheet: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet1.pdf

* Register Map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
  
* Offet Register Document: https://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers

"""

# imports

__version__ = "0.0.0+auto.0"
__repo__ = "https://github.com/adafruit/Adafruit_CircuitPython_MPU6050.git"

from math import radians
from time import sleep
from adafruit_register.i2c_struct import UnaryStruct, ROUnaryStruct
from adafruit_register.i2c_struct_array import StructArray
from adafruit_register.i2c_bit import RWBit
from adafruit_register.i2c_bits import RWBits
from adafruit_bus_device import i2c_device

try:
    from typing import Tuple
    from busio import I2C
except ImportError:
    pass

_MPU6050_DEVICE_ID_AD0_LO = 0x68
_MPU6050_DEVICE_ID_AD0_HI = 0x69

"""
_MPU6050_XG_OFFS_TC   = 0x00     
_MPU6050_YG_OFFS_TC   = 0x01       
_MPU6050_ZG_OFFS_TC   = 0x02    # FURTHER DEVELOPMENT NEEDED TO ENABLE THESE 
_MPU6050_FINE_GAIN    = 0x03   
_MPU6050_Y_FINE_GAIN  = 0x04   
_MPU6050_Z_FINE_GAIN  = 0x05   
"""

# Accelerometer Offset Registers
_MPU6050_XA_OFFS_USRH = 0x06
_MPU6050_XA_OFFS_USRL = 0x07
_MPU6050_YA_OFFS_USRH = 0x08
_MPU6050_YA_OFFS_USRL = 0x09
_MPU6050_ZA_OFFS_USRH = 0x0A
_MPU6050_ZA_OFFS_USRL = 0x0B
  
# Self Test Registers
_MPU6050_SELF_TEST_X = 0x0D
_MPU6050_SELF_TEST_Y = 0x0E
_MPU6050_SELF_TEST_Z = 0x0F
_MPU6050_SELF_TEST_A = 0x10

#  Gyroscope Offset Registers
_MPU6050_XG_OFFS_USRH = 0x13
_MPU6050_XG_OFFS_USRL = 0x14
_MPU6050_YG_OFFS_USRH = 0x15
_MPU6050_YG_OFFS_USRL = 0x16
_MPU6050_ZG_OFFS_USRH = 0x17
_MPU6050_ZG_OFFS_USRL = 0x18
    
# Configuration Registers
_MPU6050_SMPLRT_DIV   = 0x19
_MPU6050_CONFIG       = 0x1A
_MPU6050_GYRO_CONFIG  = 0x1B
_MPU6050_ACCEL_CONFIG = 0x1C

"""
# Freefall 
_MPU6050_FF_THR = 0x1D  # FURTHER DEVELOPMENT NEEDED TO ENABLE THESE
_MPU6050_FF_DUR = 0x1E

# Motion Detection
_MPU6050_MOT_DET_THRSHLD  = 0x1F
_MPU6050_MOT_DET_DURATION = 0x20
_MPU6050_ZRMOT_THR        = 0x21   
_MPU6050_ZRMOT_DUR        = 0x22     
"""

# FIFO Enable
_MPU6050_FIFO_EN  = 0x23

"""
# I2C Master/Slave Bus Control
_MPU6050_I2C_MST_CTRL    = 0x24 # FURTHER DEVELOPMENT NEEDED TO ENABLE THESE
_MPU6050_I2C_SLV0_ADDR   = 0x25
_MPU6050_I2C_SLV0_REG    = 0x26
_MPU6050_I2C_SLV0_CTRL   = 0x27    
_MPU6050_I2C_SLV1_ADDR   = 0x28
_MPU6050_I2C_SLV1_REG    = 0x29
_MPU6050_I2C_SLV1_CTRL   = 0x2A    
_MPU6050_I2C_SLV2_ADDR   = 0x2B
_MPU6050_I2C_SLV2_REG    = 0x2C
_MPU6050_I2C_SLV2_CTRL   = 0x2D       
_MPU6050_I2C_SLV3_ADDR   = 0x2E 
_MPU6050_I2C_SLV3_REG    = 0x2F
_MPU6050_I2C_SLV3_CTRL   = 0x30    
_MPU6050_I2C_SLV4_ADDR   = 0x31
_MPU6050_I2C_SLV4_REG    = 0x32
_MPU6050_I2C_SLV4_DO     = 0x33
_MPU6050_I2C_SLV4_CTRL   = 0x34
_MPU6050_I2C_SLV4_DI     = 0x35   
_MPU6050_I2C_MST_STATUS  = 0x36
"""

# Interrupt Configuration
_MPU6050_INT_PIN_CFG    = 0x37
_MPU6050_INT_ENABLE     = 0x38

# _DMP_INT_STATUS = 0x39   
_MPU6050_INT_STATUS     = 0x3A
    
# Sensor Outputs
_MPU6050_ACCEL_XOUT_H = 0x3B
_MPU6050_ACCEL_XOUT_L = 0x3C
_MPU6050_ACCEL_YOUT_H = 0x3D
_MPU6050_ACCEL_YOUT_L = 0x3E
_MPU6050_ACCEL_ZOUT_H = 0x3F
_MPU6050_ACCEL_ZOUT_L = 0x40
_MPU6050_TEMP_OUT_H   = 0x41
_MPU6050_TEMP_OUT_L   = 0x42
_MPU6050_GYRO_XOUT_H  = 0x43
_MPU6050_GYRO_XOUT_L  = 0x44
_MPU6050_GYRO_YOUT_H  = 0x45
_MPU6050_GYRO_YOUT_L  = 0x46
_MPU6050_GYRO_ZOUT_H  = 0x47
_MPU6050_GYRO_ZOUT_L  = 0x48

"""
# External Sensor Data  More development needed to enable these
_MPU6050_EXT_SENS_DATA_00 = 0x49
_MPU6050_EXT_SENS_DATA_01 = 0x4A
_MPU6050_EXT_SENS_DATA_02 = 0x4B
_MPU6050_EXT_SENS_DATA_03 = 0x4C
_MPU6050_EXT_SENS_DATA_04 = 0x4D
_MPU6050_EXT_SENS_DATA_05 = 0x4E
_MPU6050_EXT_SENS_DATA_06 = 0x4F
_MPU6050_EXT_SENS_DATA_07 = 0x50
_MPU6050_EXT_SENS_DATA_08 = 0x51
_MPU6050_EXT_SENS_DATA_09 = 0x52
_MPU6050_EXT_SENS_DATA_10 = 0x53
_MPU6050_EXT_SENS_DATA_11 = 0x54
_MPU6050_EXT_SENS_DATA_12 = 0x55
_MPU6050_EXT_SENS_DATA_13 = 0x56
_MPU6050_EXT_SENS_DATA_14 = 0x57
_MPU6050_EXT_SENS_DATA_15 = 0x58
_MPU6050_EXT_SENS_DATA_16 = 0x59
_MPU6050_EXT_SENS_DATA_17 = 0x5A
_MPU6050_EXT_SENS_DATA_18 = 0x5B
_MPU6050_EXT_SENS_DATA_19 = 0x5C
_MPU6050_EXT_SENS_DATA_20 = 0x5D
_MPU6050_EXT_SENS_DATA_21 = 0x5E
_MPU6050_EXT_SENS_DATA_22 = 0x5F
_MPU6050_EXT_SENS_DATA_23 = 0x60

# Motion Detect Status
# _MPU6050_MOT_DETECT_STATUS = 0x61   


# I2C Slave DO
_MPU6050_I2C_SLV0_DO = 0x63
_MPU6050_I2C_SLV1_DO = 0x64
_MPU6050_I2C_SLV2_DO = 0x65
_MPU6050_I2C_SLV3_DO = 0x66
        
# I2C Master Delay Control
_MPU6050_I2C_MT_DELAY_CTRL = 0x67
"""

# Resets / Enables
_MPU6050_SIGNAL_PATH_RESET = 0x68

# _MPU6050_MOT_DETECT_CTRL = 0x69

_MPU6050_USER_CTRL         = 0x6A
_MPU6050_PWR_MGMT_1        = 0x6B
_MPU6050_PWR_MGMT_2        = 0x6C

"""
# DMP
_MPU6050_BANK_SEL       = const(0x6D)        
_MPU6050_MEM_START_ADDR = const(0x6E) 
_MPU6050_MEM_R_W        = const(0x6F)      # FURTHER DEVELOPMENT NEEDED TO ENABLE THESE 
_MPU6050_DMP_CFG_1      = const(0x70)      
_MPU6050_DMP_CFG_2      = const(0x71)            
""" 

# FIFO Buffer Count
_MPU6050_FIFO_COUNTH = 0x72
_MPU6050_FIFO_COUNTL = 0x73
_MPU6050_FIFO_R_W =    0x74
        
# Who am I?  "To be... or not to be? That -is- the question"
_MPU6050_WHO_AM_I = 0x75

STANDARD_GRAVITY = 9.80665

class Range:  # pylint: disable=too-few-public-methods
    """Allowed values for `accelerometer_range`.

    * :attr:`Range.RANGE_2_G`
    * :attr:`Range.RANGE_4_G`
    * :attr:`Range.RANGE_8_G`
    * :attr:`Range.RANGE_16_G`

    """

    RANGE_2_G = 0  # +/- 2g (default value)
    RANGE_4_G = 1  # +/- 4g
    RANGE_8_G = 2  # +/- 8g
    RANGE_16_G = 3  # +/- 16g


class GyroRange:  # pylint: disable=too-few-public-methods
    """Allowed values for `gyro_range`.

    * :attr:`GyroRange.RANGE_250_DPS`
    * :attr:`GyroRange.RANGE_500_DPS`
    * :attr:`GyroRange.RANGE_1000_DPS`
    * :attr:`GyroRange.RANGE_2000_DPS`

    """

    RANGE_250_DPS = 0  # +/- 250 deg/s (default value)
    RANGE_500_DPS = 1  # +/- 500 deg/s
    RANGE_1000_DPS = 2  # +/- 1000 deg/s
    RANGE_2000_DPS = 3  # +/- 2000 deg/s


class Bandwidth:  # pylint: disable=too-few-public-methods
    """Allowed values for `filter_bandwidth`.

    * :attr:`Bandwidth.BAND_260_HZ`
    * :attr:`Bandwidth.BAND_184_HZ`
    * :attr:`Bandwidth.BAND_94_HZ`
    * :attr:`Bandwidth.BAND_44_HZ`
    * :attr:`Bandwidth.BAND_21_HZ`
    * :attr:`Bandwidth.BAND_10_HZ`
    * :attr:`Bandwidth.BAND_5_HZ`

    """

    BAND_260_HZ = 0  # Docs imply this disables the filter
    BAND_184_HZ = 1  # 184 Hz
    BAND_94_HZ = 2  # 94 Hz
    BAND_44_HZ = 3  # 44 Hz
    BAND_21_HZ = 4  # 21 Hz
    BAND_10_HZ = 5  # 10 Hz
    BAND_5_HZ = 6  # 5 Hz


class Rate:  # pylint: disable=too-few-public-methods
    """Allowed values for `cycle_rate`.

    * :attr:`Rate.CYCLE_1_25_HZ`
    * :attr:`Rate.CYCLE_5_HZ`
    * :attr:`Rate.CYCLE_20_HZ`
    * :attr:`Rate.CYCLE_40_HZ`

    """

    CYCLE_1_25_HZ = 0  # 1.25 Hz
    CYCLE_5_HZ = 1  # 5 Hz
    CYCLE_20_HZ = 2  # 20 Hz
    CYCLE_40_HZ = 3  # 40 Hz


class MPU6050:
    """Driver for the MPU6050 6-DoF accelerometer and gyroscope.

    :param ~busio.I2C i2c_bus: The I2C bus the device is connected to
    :param int address: The I2C device address. Defaults to :const:`0x68`

    **Quickstart: Importing and using the device**

        Here is an example of using the :class:`MPU6050` class.
        First you will need to import the libraries to use the sensor

        .. code-block:: python

            import board
            import adafruit_mpu6050

        Once this is done you can define your `board.I2C` object and define your sensor object

        .. code-block:: python

            i2c = board.I2C()  # uses board.SCL and board.SDA
            mpu = adafruit_mpu6050.MPU6050(i2c)

        Now you have access to the :attr:`acceleration`, :attr:`gyro`
        and :attr:`temperature` attributes

        .. code-block:: python

            acc_x, acc_y, acc_z = sensor.acceleration
            gyro_x, gyro_y, gyro_z = sensor.gyro
            temperature = sensor.temperature

    """

    def __init__(self, i2c_bus: I2C, address) -> None:
        
        # Attempt to establish an I2CDevice object
        try:
            self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)
        except:
            raise RuntimeError("Device address passed in is incorrect. Expecting 0x68 OR 0x69 (104 OR 105). "
                               "Check Datasheet / Register Map documentation for more information.")
        
        # Verify WHO_AM_I (register 0x75) responds with value of 104, regardless of AD0 state
        try :
            if self._who_am_i[0][0] == 104:
                print("Register ox75 (WHO_AM_I) responded with expected value.  Communication "
                "to mpu 6050 @", address, " confirmed.")
        except:
            raise RuntimeError("MPU6050 @ ", self.address, " invalid response from \"Who Am I\" "
                               "(104 0x68) not found. Possible a device, other than MPU6050, responding "
                               "from the same address. Check for I2C address conflicts on the same "
                               "I2C bus and try again.")
        
        self.reset()

        self._sample_rate_divisor = 0
        self._filter_bandwidth = Bandwidth.BAND_260_HZ
        self._gyro_range = GyroRange.RANGE_500_DPS
        self._accel_range = Range.RANGE_2_G
        sleep(0.100)
        self._clock_source = 1  # set to use gyro x-axis as reference
        sleep(0.100)
        self.sleep = False
        sleep(0.010)

    def reset(self) -> None:
        """Reinitialize the sensor"""
        self._reset = True
        while self._reset is True:
            sleep(0.001)
        sleep(0.100)

        _signal_path_reset = 0b111  # reset all sensors
        sleep(0.100)

    _clock_source = RWBits(3, _MPU6050_PWR_MGMT_1, 0)
    _device_id = ROUnaryStruct(_MPU6050_WHO_AM_I, ">B")

    _reset = RWBit(_MPU6050_PWR_MGMT_1, 7, 1)
    _signal_path_reset = RWBits(3, _MPU6050_SIGNAL_PATH_RESET, 3)

    _gyro_range = RWBits(2, _MPU6050_GYRO_CONFIG, 3)
    _accel_range = RWBits(2, _MPU6050_ACCEL_CONFIG, 3)

    _filter_bandwidth = RWBits(2, _MPU6050_CONFIG, 3)

    _raw_accel_data = StructArray(_MPU6050_ACCEL_XOUT_H, ">h", 3)
    _raw_gyro_data = StructArray(_MPU6050_GYRO_XOUT_H, ">h", 3)
    _raw_temp_data = ROUnaryStruct(_MPU6050_TEMP_OUT_H, ">h")
    
    _who_am_i = StructArray(_MPU6050_WHO_AM_I,'B',1)
    _cycle = RWBit(_MPU6050_PWR_MGMT_1, 5)
    _cycle_rate = RWBits(2, _MPU6050_PWR_MGMT_2, 6, 1)

    sleep = RWBit(_MPU6050_PWR_MGMT_1, 6, 1)
    """Shuts down the accelerometers and gyroscopes, saving power. No new data will
    be recorded until the sensor is taken out of sleep by setting to `False`"""
    sample_rate_divisor = UnaryStruct(_MPU6050_SMPLRT_DIV, ">B")
    """The sample rate divisor. See the datasheet for additional detail"""

    @property
    def temperature(self) -> float:
        """The current temperature in  º Celsius"""
        raw_temperature = self._raw_temp_data
        temp = (raw_temperature / 340.0) + 36.53
        return temp

    @property
    def acceleration(self) -> Tuple[float, float, float]:
        """Acceleration X, Y, and Z axis data in :math:`m/s^2`"""
        raw_data = self._raw_accel_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        accel_range = self._accel_range
        accel_scale = 1
        if accel_range == Range.RANGE_16_G:
            accel_scale = 2048
        if accel_range == Range.RANGE_8_G:
            accel_scale = 4096
        if accel_range == Range.RANGE_4_G:
            accel_scale = 8192
        if accel_range == Range.RANGE_2_G:
            accel_scale = 16384

        # setup range dependant scaling
        accel_x = (raw_x / accel_scale) * STANDARD_GRAVITY
        accel_y = (raw_y / accel_scale) * STANDARD_GRAVITY
        accel_z = (raw_z / accel_scale) * STANDARD_GRAVITY

        return (accel_x, accel_y, accel_z)

    @property
    def gyro(self) -> Tuple[float, float, float]:
        """Gyroscope X, Y, and Z axis data in :math:`º/s`"""
        raw_data = self._raw_gyro_data
        raw_x = raw_data[0][0]
        raw_y = raw_data[1][0]
        raw_z = raw_data[2][0]

        gyro_scale = 1
        gyro_range = self._gyro_range
        if gyro_range == GyroRange.RANGE_250_DPS:
            gyro_scale = 131
        if gyro_range == GyroRange.RANGE_500_DPS:
            gyro_scale = 65.5
        if gyro_range == GyroRange.RANGE_1000_DPS:
            gyro_scale = 32.8
        if gyro_range == GyroRange.RANGE_2000_DPS:
            gyro_scale = 16.4

        # setup range dependant scaling
        gyro_x = radians(raw_x / gyro_scale)
        gyro_y = radians(raw_y / gyro_scale)
        gyro_z = radians(raw_z / gyro_scale)

        return (gyro_x, gyro_y, gyro_z)

    @property
    def cycle(self) -> bool:
        """Enable or disable periodic measurement at a rate set by :meth:`cycle_rate`.
        If the sensor was in sleep mode, it will be waken up to cycle"""
        return self._cycle

    @cycle.setter
    def cycle(self, value: bool) -> None:
        self.sleep = not value
        self._cycle = value

    @property
    def gyro_range(self) -> int:
        """The measurement range of all gyroscope axes. Must be a `GyroRange`"""
        return self._gyro_range

    @gyro_range.setter
    def gyro_range(self, value: int) -> None:
        if (value < 0) or (value > 3):
            raise ValueError("gyro_range must be a GyroRange")
        self._gyro_range = value
        sleep(0.01)

    @property
    def accelerometer_range(self) -> int:
        """The measurement range of all accelerometer axes. Must be a `Range`"""
        return self._accel_range

    @accelerometer_range.setter
    def accelerometer_range(self, value: int) -> None:
        if (value < 0) or (value > 3):
            raise ValueError("accelerometer_range must be a Range")
        self._accel_range = value
        sleep(0.01)

    @property
    def filter_bandwidth(self) -> int:
        """The bandwidth of the gyroscope Digital Low Pass Filter. Must be a `GyroRange`"""
        return self._filter_bandwidth

    @filter_bandwidth.setter
    def filter_bandwidth(self, value: int) -> None:
        if (value < 0) or (value > 6):
            raise ValueError("filter_bandwidth must be a Bandwidth")
        self._filter_bandwidth = value
        sleep(0.01)

    @property
    def cycle_rate(self) -> int:
        """The rate that measurements are taken while in `cycle` mode. Must be a `Rate`"""
        return self._cycle_rate

    @cycle_rate.setter
    def cycle_rate(self, value: int) -> None:
        if (value < 0) or (value > 3):
            raise ValueError("cycle_rate must be a Rate")
        self._cycle_rate = value
        sleep(0.01)



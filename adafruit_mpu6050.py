# SPDX-FileCopyrightText: 2019 Bryan Siepert for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
`adafruit_mpu6050`
================================================================================

CircuitPython helper library for the MPU6050 6-DoF Accelerometer and Gyroscope


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

_MPU6050_DEFAULT_ADDRESS = 0x68  # MPU6050 default i2c address w/ AD0 low
_MPU6050_DEVICE_ID = 0x68  # The correct MPU6050_WHO_AM_I value

_MPU6050_SELF_TEST_X = 0x0D  # Self test factory calibrated values register
_MPU6050_SELF_TEST_Y = 0x0E  # Self test factory calibrated values register
_MPU6050_SELF_TEST_Z = 0x0F  # Self test factory calibrated values register
_MPU6050_SELF_TEST_A = 0x10  # Self test factory calibrated values register
_MPU6050_SMPLRT_DIV = 0x19  # sample rate divisor register
_MPU6050_CONFIG = 0x1A  # General configuration register
_MPU6050_GYRO_CONFIG = 0x1B  # Gyro specfic configuration register
_MPU6050_ACCEL_CONFIG = 0x1C  # Accelerometer specific configration register
_MPU6050_INT_PIN_CONFIG = 0x37  # Interrupt pin configuration register
_MPU6050_ACCEL_OUT = 0x3B  # base address for sensor data reads
_MPU6050_TEMP_OUT = 0x41  # Temperature data high byte register
_MPU6050_GYRO_OUT = 0x43  # base address for sensor data reads
_MPU6050_SIG_PATH_RESET = 0x68  # register to reset sensor signal paths
_MPU6050_USER_CTRL = 0x6A  # FIFO and I2C Master control register
_MPU6050_PWR_MGMT_1 = 0x6B  # Primary power/sleep control register
_MPU6050_PWR_MGMT_2 = 0x6C  # Secondary power/sleep control register
_MPU6050_WHO_AM_I = 0x75  # Divice ID register

STANDARD_GRAVITY = 9.80665


class Range:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`accelerometer_range`.

    * :py:attr:`Range.RANGE_2_G`
    * :py:attr:`Range.RANGE_4_G`
    * :py:attr:`Range.RANGE_8_G`
    * :py:attr:`Range.RANGE_16_G`

    """

    RANGE_2_G = 0  # +/- 2g (default value)
    RANGE_4_G = 1  # +/- 4g
    RANGE_8_G = 2  # +/- 8g
    RANGE_16_G = 3  # +/- 16g


class GyroRange:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`gyro_range`.

    * :py:attr:`GyroRange.RANGE_250_DPS`
    * :py:attr:`GyroRange.RANGE_500_DPS`
    * :py:attr:`GyroRange.RANGE_1000_DPS`
    * :py:attr:`GyroRange.RANGE_2000_DPS`

    """

    RANGE_250_DPS = 0  # +/- 250 deg/s (default value)
    RANGE_500_DPS = 1  # +/- 500 deg/s
    RANGE_1000_DPS = 2  # +/- 1000 deg/s
    RANGE_2000_DPS = 3  # +/- 2000 deg/s


class Bandwidth:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`filter_bandwidth`.

    * :py:attr:`Bandwidth.BAND_260_HZ`
    * :py:attr:`Bandwidth.BAND_184_HZ`
    * :py:attr:`Bandwidth.BAND_94_HZ`
    * :py:attr:`Bandwidth.BAND_44_HZ`
    * :py:attr:`Bandwidth.BAND_21_HZ`
    * :py:attr:`Bandwidth.BAND_10_HZ`
    * :py:attr:`Bandwidth.BAND_5_HZ`

    """

    BAND_260_HZ = 0  # Docs imply this disables the filter
    BAND_184_HZ = 1  # 184 Hz
    BAND_94_HZ = 2  # 94 Hz
    BAND_44_HZ = 3  # 44 Hz
    BAND_21_HZ = 4  # 21 Hz
    BAND_10_HZ = 5  # 10 Hz
    BAND_5_HZ = 6  # 5 Hz


class Rate:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`cycle_rate`.

    * :py:attr:`Rate.CYCLE_1_25_HZ`
    * :py:attr:`Rate.CYCLE_5_HZ`
    * :py:attr:`Rate.CYCLE_20_HZ`
    * :py:attr:`Rate.CYCLE_40_HZ`

    """

    CYCLE_1_25_HZ = 0  # 1.25 Hz
    CYCLE_5_HZ = 1  # 5 Hz
    CYCLE_20_HZ = 2  # 20 Hz
    CYCLE_40_HZ = 3  # 40 Hz


class MPU6050:  # pylint: disable=too-many-instance-attributes
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

    def __init__(self, i2c_bus: I2C, address: int = _MPU6050_DEFAULT_ADDRESS) -> None:
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        if self._device_id != _MPU6050_DEVICE_ID:
            raise RuntimeError("Failed to find MPU6050 - check your wiring!")

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

    def low_power_mode_enable(self):
        """Puts MPU6050 into low power mode.
        Accelerometer readings are updated at
        :py:attr:cycle_rate. Temperature sensor and gyroscope
        are disabled"""
        self.cycle = True
        self.sleep = False
        self.temp_disable = True
        self.stby_xg = True
        self.stby_yg = True
        self.stby_zg = True

    def low_power_mode_disable(self):
        """Restores MPU6050 to normal"""
        self.cycle = False
        self.temp_disable = False
        self.stby_xg = False
        self.stby_yg = False
        self.stby_zg = False

    _clock_source = RWBits(3, _MPU6050_PWR_MGMT_1, 0)
    _device_id = ROUnaryStruct(_MPU6050_WHO_AM_I, ">B")

    _reset = RWBit(_MPU6050_PWR_MGMT_1, 7, 1)
    _signal_path_reset = RWBits(3, _MPU6050_SIG_PATH_RESET, 3)

    _gyro_range = RWBits(2, _MPU6050_GYRO_CONFIG, 3)
    _accel_range = RWBits(2, _MPU6050_ACCEL_CONFIG, 3)

    _filter_bandwidth = RWBits(2, _MPU6050_CONFIG, 3)

    _raw_accel_data = StructArray(_MPU6050_ACCEL_OUT, ">h", 3)
    _raw_gyro_data = StructArray(_MPU6050_GYRO_OUT, ">h", 3)
    _raw_temp_data = ROUnaryStruct(_MPU6050_TEMP_OUT, ">h")

    _temp_dis = RWBit(_MPU6050_PWR_MGMT_1, 3)
    _cycle = RWBit(_MPU6050_PWR_MGMT_1, 5)
    _cycle_rate = RWBits(2, _MPU6050_PWR_MGMT_2, 6, 1)
    _stby_zg = RWBit(_MPU6050_PWR_MGMT_2, 0)
    _stby_yg = RWBit(_MPU6050_PWR_MGMT_2, 1)
    _stby_xg = RWBit(_MPU6050_PWR_MGMT_2, 2)
    _stby_za = RWBit(_MPU6050_PWR_MGMT_2, 3)
    _stby_ya = RWBit(_MPU6050_PWR_MGMT_2, 4)
    _stby_xa = RWBit(_MPU6050_PWR_MGMT_2, 5)

    _lp_wake_ctrl = RWBits(2, _MPU6050_PWR_MGMT_2, 6)

    _sleep = RWBit(_MPU6050_PWR_MGMT_1, 6, 1)
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

    @property
    def temp_disable(self) -> bool:
        """Returns True if temperature sensor is disabled
        and False if it is active"""
        return self._temp_dis

    @temp_disable.setter
    def temp_disable(self, value: bool) -> None:
        """Set True to disable temperature sensor
        or set False to make it active"""
        self._temp_dis = int(value)

    @property
    def stby_zg(self) -> bool:
        """Returns True if gryoscope Z-axis is in stand-by mode
        or returns False if it is NOT in stand-by mode."""
        return self._stby_zg

    @stby_zg.setter
    def stby_zg(self, value: bool) -> None:
        """Set True to put gryoscope Z-axis in stand-by mode
        or set False to return the gryoscope Z-axis to an active state"""
        self._stby_zg = int(value)

    @property
    def stby_yg(self) -> bool:
        """Returns True if gryoscope Y-axis is in stand-by mode
        or returns False if it is NOT in stand-by mode."""
        return self._stby_yg

    @stby_yg.setter
    def stby_yg(self, value: bool) -> None:
        """Set True to put gryoscope Y-axis in stand-by mode
        or set False to return the gryoscope Y-axis to an active state"""
        self._stby_yg = int(value)

    @property
    def stby_xg(self) -> bool:
        """Returns True if gryoscope Y-axis is in stand-by mode
        or returns False if it is NOT in stand-by mode."""
        return self._stby_xg

    @stby_xg.setter
    def stby_xg(self, value: bool) -> None:
        """Set True to put gryoscope X-axis in stand-by mode
        or set False to return the gryoscope X-axis to an active state"""
        self._stby_xg = int(value)

    @property
    def stby_za(self) -> bool:
        """Returns True if accelerometer Z-axis is in stand-by mode
        or returns False if it is NOT in stand-by mode."""
        return self._stby_za

    @stby_za.setter
    def stby_za(self, value: bool) -> None:
        """Set True to put accelerometer Z-axis in stand-by mode
        or set False to return the accelerometer Z-axis to an active state"""
        self._stby_za = int(value)

    @property
    def stby_ya(self) -> bool:
        """Returns True if accelerometer Y-axis is in stand-by mode
        or returns False if it is NOT in stand-by mode."""
        return self._stby_ya

    @stby_ya.setter
    def stby_ya(self, value: bool) -> None:
        """Set True to put accelerometer Y-axis in stand-by mode
        or set False to return the accelerometer Y-axis to an active state"""
        self._stby_ya = int(value)

    @property
    def stby_xa(self) -> bool:
        """Returns True if accelerometer X-axis is in stand-by mode
        or returns False if it is NOT in stand-by mode."""
        return self._stby_xa

    @stby_xa.setter
    def stby_xa(self, value: bool) -> None:
        """Set True to put accelerometer Y-axis in stand-by mode
        or set False to return the accelerometer Y-axis to an active state"""
        self._stby_xa = int(value)

    @property
    def sleep(self) -> bool:
        """Returns True if device is in 'SLEEP MODE' or
        False if it is active"""
        return self._sleep

    @sleep.setter
    def sleep(self, value: bool) -> None:
        """Set True to place device into sleep mode (sensor outputs will stop),
        or set False to place the sensor into an active state"""
        self._sleep = int(value)

    @property
    def low_power_mode(self) -> bool:
        """Returns True if in Low Power Mode"""
        if self.cycle is True and self.sleep is False:

            if (
                self.temp_disable is True
                and self.stby_yg is True
                and self.stby_zg is True
                and self.stby_xg is True
            ):
                return True

        return False

    @low_power_mode.setter
    def low_power_mode(self, value: True) -> None:
        """Set :py:attr:state_select True/False to Enter/Exit Low Power Mode.  Defaults to True.
        Set :py:attr:rate to desired :py:attr:cycle_rate
        """
        if value is False:
            self.low_power_mode_disable()
        else:
            self.low_power_mode_enable()

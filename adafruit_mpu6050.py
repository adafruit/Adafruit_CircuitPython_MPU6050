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
_MPU6050_INT_PIN_CFG = 0x37  # Interrupt pin configuration register
_MPU6050_INT_ENABLE = 0x38  # Interrupt Enable register
_MPU6050_INT_STATUS = 0x3A  #  Interrupt Status
_MPU6050_ACCEL_OUT = 0x3B  # base address for sensor data reads
_MPU6050_TEMP_OUT = 0x41  # Temperature data high byte register
_MPU6050_GYRO_OUT = 0x43  # base address for sensor data reads
_MPU6050_SIG_PATH_RESET = 0x68  # register to reset sensor signal paths
_MPU6050_USER_CTRL = 0x6A  # FIFO and I2C Master control register
_MPU6050_PWR_MGMT_1 = 0x6B  # Primary power/sleep control register
_MPU6050_PWR_MGMT_2 = 0x6C  # Secondary power/sleep control register
_MPU6050_WHO_AM_I = 0x75  # Divice ID register

STANDARD_GRAVITY = 9.80665


class ExtSyncSet:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`ext_sync_set` property.

    * :attr:'ExtSyncSet.DISABLED
    * :attr:'ExtSyncSet.TEMP_OUT_L
    * :attr:'ExtSyncSet.GYRO_XOUT_L
    * :attr:'ExtSyncSet.GYRO_YOUT_L
    * :attr:'ExtSyncSet.GYRO_ZOUT_L
    * :attr:'ExtSyncSet.ACCEL_XOUT_L
    * :attr:'ExtSyncSet.ACCEL_YOUT_L
    * :attr:'ExtSyncSet.ACCEL_ZOUT_L
    """

    DISABLED = 0  # FSYNC Input disabed
    TEMP_OUT = 1  # Data found at MPU6050_TEMP_OUT_L
    GYRO_XOUT = 2  # Data found at MPU6050_GYRO_XOUT
    GYRO_YOUT = 3  # Data found at MPU6050_GYRO_YOUT
    GYRO_ZOUT = 4  # Data found at MPU6050_GYRO_ZOUT
    ACCEL_XOUT = 5  # Data found at MPU6050_ACCEL_XOUT
    ACCEL_YOUT = 6  # Data found at MPU6050_ACCEL_YOUT
    ACCEL_ZOUT = 7  # Data found at MPU6050_ACCEL_ZOUT


class ClockSource:  # pylint: disable=too-few-public-methods
    """Allowed values for :py:attr:`clock_source`.

    * :py:attr:'ClockSource.CLKSEL_INTERNAL_8MHz
    * :py:attr:'ClockSource.CLKSEL_INTERNAL_X
    * :py:attr:'ClockSource.CLKSEL_INTERNAL_Y
    * :py:attr:'ClockSource.CLKSEL_INTERNAL_Z
    * :py:attr:'ClockSource.CLKSEL_EXTERNAL_32
    * :py:attr:'ClockSource.CLKSEL_EXTERNAL_19
    * :py:attr:'ClockSource.CLKSEL_RESERVED
    * :py:attr:'ClockSource.CLKSEL_STOP
    """

    CLKSEL_INTERNAL_8MHz = 0  # Internal 8MHz oscillator
    CLKSEL_INTERNAL_X = 1  # PLL with X Axis gyroscope reference
    CLKSEL_INTERNAL_Y = 2  # PLL with Y Axis gyroscope reference
    CLKSEL_INTERNAL_Z = 3  # PLL with Z Axis gyroscope reference
    CLKSEL_EXTERNAL_32 = 4  # External 32.768 kHz reference
    CLKSEL_EXTERNAL_19 = 5  # External 19.2 MHz reference
    CLKSEL_RESERVED = 6  # Reserved
    CLKSEL_STOP = 7  # Stops the clock, constant reset mode


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
            i2c = board.I2C()  # uses board.SCL and board.SDA
            mpu = adafruit_mpu6050.MPU6050(i2c)

            acc_x, acc_y, acc_z = mpu.acceleration
            gyro_x, gyro_y, gyro_z = mpu.gyro
            temperature = mpu.temperature
        
        You can also configure the interrupt pin to respond as desired when certain conditions
        are met.  This can be very useful when you are reading data from multiple different sensors.
        Simply configure the interrupt pin, connect it electrically to a GPIO pin on your 
        microcontroller.  
        
        For example:
        
        ..code-block:: python
        
            # Import necessary modules/packages
            import digitalio
            import board
            import adafruit_mpu6050
            
            # Configure I2C bus and instantiate mpu6050 object
            i2c = board.I2C()  # uses board.SCL and board.SDA
            mpu = adafruit_mpu6050.MPU6050(i2c)
            
            # Configure MPU6050 interrupt pin registers
            mpu.int_read_clear = True   # Clear interrupts after ANY I2C read 
            mpu.int_latch_enable = True   # Interrupt enabled until cleared by an I2C read
            mpu.int_level = False   # Interrupt Pin state goes HI when an interrupt has been generated
            mpu.int_open = False   # Push-Pull configuration
            mpu.data_rdy_en = True   # Enable 'Data Ready Interrupts'
            
            # Assign an interrupt pin of your choice on the microcontroller, Pin number will vary by board
            int_pin = digitalio.DigitalInOut(board.D5)  # Create a pin object, INPUT by default assignment
            
            # If interrupt pin is HI / True, read sensor data
            # Else, interrupt pin is LO / False, pass and check again next time through the loop
            while(True):
                if int_pin.value == True:
                    acc_x, acc_y, acc_z = mpu.acceleration
                else:                
                    pass        
        
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
        self.clock_source = (
            ClockSource.CLKSEL_INTERNAL_X
        )  # set to use gyro x-axis as reference
        sleep(0.100)
        self.sleep = False
        sleep(0.010)

    def reset(self) -> None:
        """Restores the sensor to factory default settings"""
        self._reset = True
        while self._reset is True:
            sleep(0.001)
        sleep(0.100)

        _signal_path_reset = 0b111  # reset all sensors
        sleep(0.100)

    _clksel = RWBits(3, _MPU6050_PWR_MGMT_1, 0)
    _device_id = ROUnaryStruct(_MPU6050_WHO_AM_I, ">B")

    _reset = RWBit(_MPU6050_PWR_MGMT_1, 7, 1)
    _signal_path_reset = RWBits(3, _MPU6050_SIG_PATH_RESET, 3)

    _gyro_range = RWBits(2, _MPU6050_GYRO_CONFIG, 3)
    _accel_range = RWBits(2, _MPU6050_ACCEL_CONFIG, 3)

    _filter_bandwidth = RWBits(3, _MPU6050_CONFIG, 0)
    _ext_sync_set = RWBits(3, _MPU6050_CONFIG, 3)

    _fsync_int_en_ = RWBit(_MPU6050_INT_PIN_CFG, 3)
    _fsync_int_level = RWBit(_MPU6050_INT_PIN_CFG, 4)
    _int_rd_clear = RWBit(_MPU6050_INT_PIN_CFG, 5)
    _int_open = RWBit(_MPU6050_INT_PIN_CFG, 6)
    _int_level = RWBit(_MPU6050_INT_PIN_CFG, 7)

    _data_rdy_en = RWBit(_MPU6050_INT_ENABLE, 0)
    _fifo_oflow_en = RWBit(_MPU6050_INT_ENABLE, 4)

    _data_rdy_int = RWBit(_MPU6050_INT_STATUS, 0)
    _fifo_oflow_int = RWBit(_MPU6050_INT_STATUS, 4)

    _raw_accel_data = StructArray(_MPU6050_ACCEL_OUT, ">h", 3)
    _raw_gyro_data = StructArray(_MPU6050_GYRO_OUT, ">h", 3)
    _raw_temp_data = ROUnaryStruct(_MPU6050_TEMP_OUT, ">h")

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

    @property
    def clock_source(self) -> int:
        """The clock source for the sensor"""
        return self._clksel

    @clock_source.setter
    def clock_source(self, value: int) -> None:
        """Select between Internal/External clock sources"""
        if value not in range(8):
            raise ValueError(
                "clock_source must be ClockSource value, integer from 0 - 7."
            )
        self._clksel = value

    @property
    def fsync_int_enable(self) -> bool:
        """True = Frame Sync Interrupt Enabled
        False = Frame Sync Interrupt Disabled
        NOTE: FSYNC pin used as output"""
        return self._fsync_int_en

    @fsync_int_enable.setter
    def fsync_int_enable(self, value: bool) -> None:
        self._fsync_int_en = int(value)

    @property
    def fsync_int_level(self) -> bool:
        """Deterimines the behaviour of the FSYNC pin as follows:
        True: Active when FSYNC pin LO
        False: Active when FSYNC pin HI"""
        return self._fsync_int_lvl

    @fsync_int_level.setter
    def fsync_int_level(self, value: bool) -> None:
        self._fsync_int_lvl = int(value)

    @property
    def int_read_clear(self) -> bool:
        """True : Interrupt Status bits cleared by ANY I2C Read operation
        False: Interrupt Status bits cleared  ONLY reading from
        _MPU6050_INT_STATUS REGISTER Register dec(58)"""
        return self._int_rd_clr

    @int_read_clear.setter
    def int_read_clear(self, value: bool) -> None:
        self._int_rd_clr = int(value)

    @property
    def int_latch_enable(self) -> bool:
        """Sets state of _latch_int_en, Reg Map Pg 26
        True : INT pin held to int_level setting until cleared
        False: INT pin held to int_level for 50us pulse"""
        return self._latch_int_en

    @int_latch_enable.setter
    def int_latch_enable(self, value: bool) -> None:
        self._latch_int_en = int(value)

    @property
    def int_open(self) -> bool:
        """Sets state of _int_open, Reg Map Pg 26
        True : INT pin configured as open drain
        False: INT pin configured as push-pull"""
        return self._int_open

    @int_open.setter
    def int_open(self, value: bool) -> None:
        self._int_open = int(value)

    @property
    def int_level(self) -> bool:
        """Sets state of _int_level, Reg Map Pg 26
        True : INT pin active when LO
        False: INT pin active when HI"""
        return self._int_level

    @int_level.setter
    def int_level(self, value: bool) -> None:
        self._int_level = int(value)

    @property
    def dataready_int(self) -> bool:
        """Returns state _int_level, Reg Map Pg 28
        True:"Data Ready Interrupt" has been generated
        False: "Data Ready Interrupt" has NOT been generated
        NOTE: Clears to 0 after the register has been read"""
        return self._data_rdy_int

    @property
    def fifo_overflow_int(self) -> bool:
        """Returns state _fifo_oflow_int, Reg Map Pg 28
        True: "FIFO Buffer Overflow Interrupt" has been generated
        False: "FIFO Buffer Overflow Interrupt" has NOT been generated
        NOTE: Clears to 0 after the register has been read"""
        return self._fifo_oflow_int

    @property
    def ext_sync_set(self) -> int:
        """Configures the FSYNC) pin as an input.  The sampled value will
        be reported in place of the least significant bit in a sensor
        data register determined by the value of ExtSyncSet"""
        return self._ext_sync_set

    @ext_sync_set.setter
    def ext_sync_set(self, value: int) -> None:
        if value not in range(0, 8):
            raise ValueError("setting must be ExtSyncSet, integer from 0 - 7")
        self._ext_sync_set = value
    
    @property
    def data_rdy_en(self) -> bool:
        """Returns state _data_rdy_en , Reg Map Pg 27
        True: Data Ready Interrupt generation enabled
        False: Data Ready Interrupt generation Disabled"""
        return self._data_rdy_en

    @data_rdy_en.setter
    def data_rdy_en(self, value:bool) -> None:
        self._data_rdy_en = value
        
    @property
    def fifo_oflow_en(self) -> bool:
        """Returns state _fifo_oflow_en , Reg Map Pg 27
        True: FIFO Buffer OverFlow Interrupt generation enabled
        False: FIFO Buffer OverFlow Interrupt generation enabled"""
        return self._fifo_oflow_en

    @fifo_oflow_en.setter
    def fifo_oflow_en(self, value:bool) -> None:
        self._fifo_oflow_en = value

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
_MPU6050_SMPLRT_DIV = 0x19
_MPU6050_CONFIG = 0x1A
_MPU6050_GYRO_CONFIG = 0x1B
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
_MPU6050_FIFO_EN = 0x23

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
_MPU6050_INT_PIN_CFG = 0x37
_MPU6050_INT_ENABLE = 0x38

# _DMP_INT_STATUS = 0x39
_MPU6050_INT_STATUS = 0x3A

# Sensor Outputs
_MPU6050_ACCEL_XOUT_H = 0x3B
_MPU6050_ACCEL_XOUT_L = 0x3C
_MPU6050_ACCEL_YOUT_H = 0x3D
_MPU6050_ACCEL_YOUT_L = 0x3E
_MPU6050_ACCEL_ZOUT_H = 0x3F
_MPU6050_ACCEL_ZOUT_L = 0x40
_MPU6050_TEMP_OUT_H = 0x41
_MPU6050_TEMP_OUT_L = 0x42
_MPU6050_GYRO_XOUT_H = 0x43
_MPU6050_GYRO_XOUT_L = 0x44
_MPU6050_GYRO_YOUT_H = 0x45
_MPU6050_GYRO_YOUT_L = 0x46
_MPU6050_GYRO_ZOUT_H = 0x47
_MPU6050_GYRO_ZOUT_L = 0x48

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

_MPU6050_USER_CTRL = 0x6A
_MPU6050_PWR_MGMT_1 = 0x6B
_MPU6050_PWR_MGMT_2 = 0x6C

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
_MPU6050_FIFO_R_W = 0x74

# Who am I?  "To be... or not to be? That -is- the question"
_MPU6050_WHO_AM_I = 0x75

STANDARD_GRAVITY = 9.80665


class Clksel:  # pylint: disable=too-few-public-methods
    """Allowed values for `clock_source` Register object.

    * :attr:'Clksel.CLKSEL_INTERNAL_8MHz
    * :attr:'Clksel.CLKSEL_INTERNAL_X
    * :attr:'Clksel.CLKSEL_INTERNAL_Y
    * :attr:'Clksel.CLKSEL_INTERNAL_Z
    * :attr:'Clksel.CLKSEL_EXTERNAL_32
    * :attr:'Clksel.CLKSEL_EXTERNAL_19
    * :attr:'Clksel.CLKSEL_RESERVED
    * :attr:'Clksel.CLKSEL_STOP
    """

    CLKSEL_INTERNAL_8MHz = 0  # Internal 8MHz oscillator
    CLKSEL_INTERNAL_X = 1  # PLL with X Axis gyroscope reference
    CLKSEL_INTERNAL_Y = 2  # PLL with Y Axis gyroscope reference
    CLKSEL_INTERNAL_Z = 3  # PLL with Z Axis gyroscope reference
    CLKSEL_EXTERNAL_32 = 4  # External 32.768 kHz reference
    CLKSEL_EXTERNAL_19 = 5  # External 19.2 MHz reference
    CLKSEL_RESERVED = 6  # Reserved
    CLKSEL_STOP = 7  # Stops the clock, constant reset mode


class Ext_Sync_Set:  # pylint: disable=too-few-public-methods
    """Allowed values for `ext_sync_set` Register object.

    * :attr:'Ext_Sync_Set.DISABLED
    * :attr:'Ext_Sync_Set.TEMP_OUT_L
    * :attr:'Ext_Sync_Set.GYRO_XOUT_L
    * :attr:'Ext_Sync_Set.GYRO_YOUT_L
    * :attr:'Ext_Sync_Set.GYRO_ZOUT_L
    * :attr:'Ext_Sync_Set.ACCEL_XOUT_L
    * :attr:'Ext_Sync_Set.ACCEL_YOUT_L
    * :attr:'Ext_Sync_Set.ACCEL_ZOUT_L
    """

    DISABLED = 0  # FSYNC Input disabed
    TEMP_OUT = 1  # Data found at MPU6050_TEMP_OUT_L
    GYRO_XOUT = 2  # Data found at MPU6050_GYRO_XOUT
    GYRO_YOUT = 3  # Data found at MPU6050_GYRO_YOUT
    GYRO_ZOUT = 4  # Data found at MPU6050_GYRO_ZOUT
    ACCEL_XOUT = 5  # Data found at MPU6050_ACCEL_XOUT
    ACCEL_YOUT = 6  # Data found at MPU6050_ACCEL_YOUT
    ACCEL_ZOUT = 7  # Data found at MPU6050_ACCEL_ZOUT


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
    """Allowed values for `lp_wake_ctrl`.

    * :attr:`Rate.CYCLE_1_25_HZ`
    * :attr:`Rate.CYCLE_5_HZ`
    * :attr:`Rate.CYCLE_20_HZ`
    * :attr:`Rate.CYCLE_40_HZ`

    """

    CYCLE_1_25_HZ = 0  # 1.25 Hz
    CYCLE_5_HZ = 1  # 5 Hz
    CYCLE_20_HZ = 2  # 20 Hz
    CYCLE_40_HZ = 3  # 40 Hz


# pylint: disable=too-many-instance-attributes
# pylint: disable=too-many-public-methods
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

    def __init__(self, i2c_bus: I2C, address: int = _MPU6050_DEVICE_ID_AD0_LO) -> None:

        # Check user input
        self.check_valid_address(address)

        # Verify i2c_bus is not already locked to prevent a hang
        i2c_bus.unlock()

        # Attempt to establish an I2CDevice object
        self.i2c_device = i2c_device.I2CDevice(i2c_bus, address)

        # Verify MPU-6050 is responding to I2C traffic
        self.check_communication(address)

        # Reset to clear any previously stored settings
        self.device_reset()

        # Configure sensor as desired
        self.sample_rate = 0
        self.filter_bandwidth = Bandwidth.BAND_260_HZ
        self.gyro_range = GyroRange.RANGE_500_DPS
        self.accelerometer_range = Range.RANGE_2_G
        sleep(0.100)
        self.clock_source = Clksel.CLKSEL_INTERNAL_X  # CTRL-F "Class CLKSEL"
        sleep(0.500)

        # Take sensor out of sleep to enable output
        self.sleep = False

    def device_reset(self) -> None:
        """Resets all internal regi's to their default
        Automatically clears to 0"""
        self._device_reset = True
        while self._device_reset is True:
            sleep(0.01)

    def temp_path_reset(self):
        """Resets the temperature sensor signal path
        Register values are retained"""
        self._temp_reset = True
        while self._temp_reset is True:
            sleep(0.01)

    def accel_path_reset(self):
        """Resets the accelerometer sensor signal path
        Register values are retained"""
        self._accel_reset = True
        while self._accel_reset is True:
            sleep(0.01)

    def gyro_path_reset(self):
        """Resets the gyroscope sensor signal path
        Register values are retained"""
        self._temp_reset = True
        while self._temp_reset is True:
            sleep(0.01)

    def all_sensor_path_reset(self):
        """Resets the all sensor signal paths
        Register values are retained"""
        self.temp_path_reset()
        self.accel_path_reset()
        self.gyro_path_reset()

    def all_signal_condtion_reset(self):
        """Resets ALL sensor signal paths
        Register values are NOT retained"""
        self._sig_cond_reset = True
        while self._temp_reset is True:
            sleep(0.01)

    # pylint: disable=no-self-use
    def check_valid_address(self, address_to_check):
        """Check user input for correctness correctness"""
        if (
            address_to_check is not _MPU6050_DEVICE_ID_AD0_LO
            and address_to_check is not _MPU6050_DEVICE_ID_AD0_HI
        ):
            raise RuntimeError(
                "MPU6050 address passed in is incorrect. Expecting "
                "0x68 OR 0x69 (104 OR 105).  See Data Sheet / Register Map for details"
            )

    def check_communication(self, address_to_check):
        """Verify WHO_AM_I (register 0x75) responds with value of 104, regardless of AD0 state.
        This double checks the device responding is a MPU6050"""
        if self.whoami is _MPU6050_DEVICE_ID_AD0_LO:
            print("Communication to MPU6050 @", address_to_check, " confirmed.")
        else:
            raise RuntimeError(
                "MPU6050 @ ",
                address_to_check,
                " invalid response from 'Who Am I' "
                "(104 0x68) not found. Possible a device, other than MPU6050, responding "
                "from the same address. Check for I2C address conflicts on the same "
                "I2C bus and try again.",
            )

    def low_power_mode_enable(self, rate=Rate.CYCLE_40_HZ):
        """Places MPU6050 into low power mode."""
        self.cycle_rate = rate
        self.cycle_mode = True
        self.sleep = False
        self.temp_disable = True
        self.stby_xg = True
        self.stby_yg = True
        self.stby_zg = True

    _smplrt_div = UnaryStruct(_MPU6050_SMPLRT_DIV, ">B")
    _dlpf_config = RWBits(3, _MPU6050_CONFIG, 0, 3)
    _fs_sel = RWBits(2, _MPU6050_GYRO_CONFIG, 3, 2)
    _afs_sel = RWBits(2, _MPU6050_ACCEL_CONFIG, 3, 2)
    # _slv0_fifo_en = RWBit(_MPU6050_FIFO_EN, 0)
    # _slv1_fifo_en = RWBit(_MPU6050_FIFO_EN, 1)
    # _slv2_fifo_en = RWBit(_MPU6050_FIFO_EN, 2)
    _accel_fifo_en = RWBit(_MPU6050_FIFO_EN, 3)
    _zg_fifo_en = RWBit(_MPU6050_FIFO_EN, 4)
    _yg_fifo_en = RWBit(_MPU6050_FIFO_EN, 5)
    _xg_fifo_en = RWBit(_MPU6050_FIFO_EN, 6)
    _temp_fifo_en = RWBit(_MPU6050_FIFO_EN, 7)
    _i2c_bypass_en = RWBit(_MPU6050_INT_PIN_CFG, 2)
    _fsync_int_en_ = RWBit(_MPU6050_INT_PIN_CFG, 3)
    _fsync_int_level = RWBit(_MPU6050_INT_PIN_CFG, 4)
    _int_rd_clear = RWBit(_MPU6050_INT_PIN_CFG, 5)
    _int_open = RWBit(_MPU6050_INT_PIN_CFG, 6)
    _int_level = RWBit(_MPU6050_INT_PIN_CFG, 7)
    _data_rdy_en = RWBit(_MPU6050_INT_ENABLE, 0)
    # _i2c_mst_int_en = RWBit(_MPU6050_INT_ENABLE, 3)
    _fifo_oflow_en = RWBit(_MPU6050_INT_ENABLE, 4)
    _data_rdy_int = RWBit(_MPU6050_INT_STATUS, 0)
    # _i2c_mst_int = RWBit(_MPU6050_INT_STATUS, 3)
    _fifo_oflow_int = RWBit(_MPU6050_INT_STATUS, 4)
    _raw_accel_data = StructArray(_MPU6050_ACCEL_XOUT_H, ">h", 3)
    _raw_temp_data = ROUnaryStruct(_MPU6050_TEMP_OUT_H, ">h")
    _raw_gyro_data = StructArray(_MPU6050_GYRO_XOUT_H, ">h", 3)
    _temp_reset = RWBit(_MPU6050_SIGNAL_PATH_RESET, 0)
    _accel_reset = RWBit(_MPU6050_SIGNAL_PATH_RESET, 1)
    _gyro_reset = RWBit(_MPU6050_SIGNAL_PATH_RESET, 2)
    _sig_cond_reset = RWBit(_MPU6050_USER_CTRL, 0)
    # _i2c_mst_reset = RWBit(_MPU6050_USER_CTRL, 1)
    _fifo_reset = RWBit(_MPU6050_USER_CTRL, 2)
    # _i2c_mst_en = RWBit(_MPU6050_USER_CTRL, 5)
    _fifo_en = RWBit(_MPU6050_USER_CTRL, 6)
    _signal_path_reset = RWBits(
        3, _MPU6050_SIGNAL_PATH_RESET, 3
    )  # Remove ?? - make into function
    _clksel = RWBits(3, _MPU6050_PWR_MGMT_1, 0)
    _temp_dis = RWBit(_MPU6050_PWR_MGMT_1, 3)
    _cycle = RWBit(_MPU6050_PWR_MGMT_1, 5)
    _sleep = RWBit(_MPU6050_PWR_MGMT_1, 6)
    _device_reset = RWBit(_MPU6050_PWR_MGMT_1, 7, 1)
    _stby_zg = RWBit(_MPU6050_PWR_MGMT_2, 0)
    _stby_yg = RWBit(_MPU6050_PWR_MGMT_2, 1)
    _stby_xg = RWBit(_MPU6050_PWR_MGMT_2, 2)
    _stby_za = RWBit(_MPU6050_PWR_MGMT_2, 3)
    _stby_ya = RWBit(_MPU6050_PWR_MGMT_2, 4)
    _stby_xa = RWBit(_MPU6050_PWR_MGMT_2, 5)
    _lp_wake_ctrl = RWBits(2, _MPU6050_PWR_MGMT_2, 6)
    _fifo_count = ROUnaryStruct(_MPU6050_FIFO_COUNTH, ">H")
    _device_id = ROUnaryStruct(_MPU6050_WHO_AM_I, ">B")
    _who_am_i = StructArray(_MPU6050_WHO_AM_I, "B", 1)

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

        accel_range = self._afs_sel
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
        gyro_range = self._fs_sel
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
    def cycle_mode(self) -> bool:
        """Returns state of _cycle"""
        return self._cycle

    @cycle_mode.setter
    def cycle_mode(self, value: bool) -> None:
        """Sets state of _cycle, Reg Map Pg 40"""
        self._cycle = int(value)

    @property
    def device_sleep(self) -> bool:
        """Returns state of _sleep, Reg Map Pg 40"""
        return self._sleep

    @device_sleep.setter
    def device_sleep(self, value: bool) -> None:
        """Sets state of _sleep, Reg Map Pg 40"""
        self._sleep = int(value)

    @property
    def temp_disable(self) -> bool:
        """Returns state of _temp_dis, Reg Map Pg 40"""
        return self._temp_dis

    @temp_disable.setter
    def temp_disable(self, value: bool) -> None:
        """Sets state of _temp_dis, Reg Map Pg 40"""
        self._temp_dis = int(value)

    @property
    def stby_zg(self) -> bool:
        """Returns _stby_zg state, Reg Map Pg 42"""
        return self._stby_zg

    @stby_zg.setter
    def stby_zg(self, value: bool) -> None:
        """Sets state of _stby_zg, Reg Map Pg 42"""
        self._stby_zg = int(value)

    @property
    def stby_yg(self) -> bool:
        """Returns _stby_yg state, Reg Map Pg 42"""
        return self._stby_yg

    @stby_yg.setter
    def stby_yg(self, value: bool) -> None:
        """Sets state of _stby_yg, Reg Map Pg 42"""
        self._stby_yg = int(value)

    @property
    def stby_xg(self) -> bool:
        """Returns _stby_xg state, Reg Map Pg 42"""
        return self._stby_xg

    @stby_xg.setter
    def stby_xg(self, value: bool) -> None:
        """Sets state of _stby_xg, Reg Map Pg 42"""
        self._stby_xg = int(value)

    @property
    def stby_za(self) -> bool:
        """Returns _stby_ya state, Reg Map Pg 42"""
        return self._stby_za

    @stby_za.setter
    def stby_za(self, value: bool) -> None:
        """Sets state of _stby_za, Reg Map Pg 42"""
        self._stby_za = int(value)

    @property
    def stby_ya(self) -> bool:
        """Returns _stby_ya state, Reg Map Pg 42"""
        return self._stby_ya

    @stby_ya.setter
    def stby_ya(self, value: bool) -> None:
        # Sets state of _stby_ya, Reg Map Pg 42
        self._stby_ya = int(value)

    @property
    def stby_xa(self) -> bool:
        """Returns _stby_xa state, Reg Map Pg 42"""
        return self._stby_xa

    @stby_xa.setter
    def stby_xa(self, value: bool) -> None:
        """Sets state of _stby_xa, Reg Map Pg 42"""
        self._stby_xa = int(value)

    @property
    def sample_rate(self) -> int:
        """Returns _sample_rate_divisor, Reg Map Pg 11"""
        return self._smplrt_div

    @sample_rate.setter
    def sample_rate(self, value: int) -> None:
        """Sets sample_rate, Reg Map Pg 11
        *
        * Sets/Gets Gyroscope Sample Rate
        *
        * Equation shown as (Reg Map Pg 11):
        *        * Gyro Sample Rate = Gyro Output Rate / (1 + Smplrt_div)
        *
        * Solve for Smplrt_div:
        *        * Smplrt_div = (Gyro Output Rate / Gyro Sample Rate) - 1
        *
        * = Notes =
        * 1. Gyro Output Rate = 8 kHz, when dlpf_cfg = 0 or 7
        * 2. Gyro Output Rate = 1 kHz, when dlpf_cfg = 1 thru 6
        * 3. dlpf_cfg set via 'MPU6050.filter_bandwidth = X' property
        * 4. 31.4 samples/sec bit resolution @ 8 kHz Gyro_Output
        * 5.  3.9 samples/sec bit resolution @ 1 kHz Gyro_Output
        *
        * = Sample Rate Example =
        *     - Desired sample Rate = 100
        *     - dlpf_cfg = 0
        *         -Gyro Output Rate 8 kHz (See note 1, above)
        *
        *     Smplrt_div = (8000 / 100) - 1
        *     Smplrt_div = 80 - 1
        *     Smplrt_div = 79
        *
        *     Unsigned Integer 79 is written to _smplrt_div
        *"""

        # Check entry for positive validity
        if value <= 0:
            raise ValueError("sample_rate must be an integer greater than 0")

        # Check rates are possible with 0 - 255 bit resolution
        if (
            self.filter_bandwidth is Bandwidth.BAND_260_HZ
            or self.filter_bandwidth is Bandwidth.BAND_5_HZ
        ) and (value <= 31 or value >= 8001):
            raise ValueError("sample_rate must be greater than 31 and less than 8001")
        if value <= 3 or value >= 1001:
            raise ValueError("sample_rate must be greater than 3 and less than 1001")

        # Instantiate gyro_output
        gyro_output = 0

        if self.filter_bandwidth in (0, 7):
            gyro_output = 8000
        else:
            gyro_output = 1000

        self._smplrt_div = int((gyro_output / value) - 1)

    @property
    def gyro_range(self) -> int:
        """Returns _fs_sel, Reg Map Pg 14"""
        return self._fs_sel

    @gyro_range.setter
    def gyro_range(self, value: int) -> None:
        """Sets _fs_sel, Reg Map Pg 14"""
        if (value < 0) or (value > 3):
            raise ValueError("gyro_range must be a GyroRange")
        self._fs_sel = value
        sleep(0.01)

    @property
    def accelerometer_range(self) -> int:
        """Returns _afs_sel, Reg Map Pg 15"""
        return self._afs_sel

    @accelerometer_range.setter
    def accelerometer_range(self, value: int) -> None:
        """Sets _afs_sel, Reg Map Pg 15"""
        if (value < 0) or (value > 3):
            raise ValueError("accelerometer_range must be a Range")
        self._afs_sel = value
        sleep(0.01)

    @property
    def filter_bandwidth(self) -> int:
        """Returns _dlpf_config, Reg Map 13"""
        return self._dlpf_config

    @filter_bandwidth.setter
    def filter_bandwidth(self, value: int) -> None:
        """Sets lp_wake_ctrl, Reg Map 13"""
        if (value < 0) or (value > 6):
            raise ValueError("filter_bandwidth must be a Bandwidth")
        self._dlpf_config = value
        sleep(0.01)

    @property
    def cycle_rate(self) -> int:
        """Returns lp_wake_ctrl, Reg Map 42"""
        return self._lp_wake_ctrl

    @cycle_rate.setter
    def cycle_rate(self, value: int) -> None:
        """Sets lp_wake_ctrl, Reg Map 42"""
        if (value < 0) or (value > 3):
            raise ValueError("cycle_rate must be a Rate")
        self._lp_wake_ctrl = value

    @property
    def ext_sync_set(self) -> int:
        """Returns ext_sync_set value, Reg Map Pg 13"""
        return self._ext_sync_set

    @ext_sync_set.setter
    def ext_sync_set(self, value: int) -> None:
        """Sets ext_sync_set, Reg Map Pg 13"""
        if (value < 0) or (value > 7):
            raise ValueError("setting must be Ext_Sync_Set")
        self._ext_sync_set = value

    @property
    def clock_source(self) -> int:
        """Returns current CLKSEL value, Reg Map Pg 40"""
        return self._clksel

    @clock_source.setter
    def clock_source(self, value: int) -> None:
        """Sets CLKSEL value, Reg Map Pg 40"""
        if (value < 0) or (value > 7):
            raise ValueError("setting must be Clksel")
        self._clksel = value

    @property
    def bytes_infifo(self) -> int:
        """Returns value in _fifo_count"""
        return self._fifo_count

    @property
    def whoami(self) -> int:
        """Returns _who_am_i"""
        return self._who_am_i[0][0]

    @property
    def fifo_enable(self) -> bool:
        """Returns FIFO output status"""
        return self._fifo_en

    @fifo_enable.setter
    def fifo_enable(self, value: bool) -> None:
        """enables/disable fifo output"""
        self._fifo_en = int(value)

    @property
    def fifo_reset(self) -> bool:
        """Returns state of bit"""
        return self._fifo_reset

    @fifo_reset.setter
    def fifo_reset(self, value: bool) -> None:
        """Set True to reset FIFO"""
        self._fifo_reset = int(value)

    @property
    def fifo_xg(self) -> bool:
        """Returns state of bit"""
        return self._xg_fifo_en

    @fifo_xg.setter
    def fifo_xg(self, value: bool) -> None:
        """Set True to have X-Axis gyro reading in FIFO"""
        self._xg_fifo_en = int(value)

    @property
    def fifo_yg(self) -> bool:
        """Returns state of bit"""
        return self._yg_fifo_en

    @fifo_yg.setter
    def fifo_yg(self, value: bool) -> None:
        #  Set True to have Y-Axis gyro reading in FIFO
        self._yg_fifo_en = int(value)

    @property
    def fifo_zg(self) -> bool:
        """Returns state of bit"""
        return self._zg_fifo_en

    @fifo_zg.setter
    def fifo_zg(self, value: bool) -> None:
        """Set True to have Z-Axis gyro reading in FIFO"""
        self._zg_fifo_en = int(value)

    @property
    def fifo_accel(self) -> bool:
        """Returns state of bit"""
        return self._accel_fifo_en

    @fifo_accel.setter
    def fifo_accel(self, value: bool) -> None:
        """Set True to have accelerometers readings in FIFO"""
        self._accel_fifo_en = int(value)

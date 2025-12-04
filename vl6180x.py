from machine import I2C, Pin
import time


class VL6180X:
    """Simple MicroPython driver for the VL6180X."""

    DEFAULT_ADDRESS = 0x29

    _SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0014
    _SYSTEM_INTERRUPT_CLEAR = 0x0015
    _SYSTEM_FRESH_OUT_OF_RESET = 0x0016
    _SYSRANGE_START = 0x0018
    _SYSRANGE_INTERMEASUREMENT_PERIOD = 0x001B
    _SYSALS_INTERMEASUREMENT_PERIOD = 0x003E
    _RESULT_INTERRUPT_STATUS_GPIO = 0x004F
    _RESULT_RANGE_VAL = 0x0062
    _I2C_SLAVE_DEVICE_ADDRESS = 0x0212

    # This table is the one provided by ST in AN4545 and also used by Adafruit's driver.
    _DEFAULT_CONFIGURATION = (
        (0x0207, 0x01),
        (0x0208, 0x01),
        (0x0096, 0x00),
        (0x0097, 0xfd),
        (0x00e3, 0x00),
        (0x00e4, 0x04),
        (0x00e5, 0x02),
        (0x00e6, 0x01),
        (0x00e7, 0x03),
        (0x00f5, 0x02),
        (0x00d9, 0x05),
        (0x00db, 0xce),
        (0x00dc, 0x03),
        (0x00dd, 0xf8),
        (0x009f, 0x00),
        (0x00a3, 0x3c),
        (0x00b7, 0x00),
        (0x00bb, 0x3c),
        (0x00b2, 0x09),
        (0x00ca, 0x09),
        (0x0198, 0x01),
        (0x01b0, 0x17),
        (0x01ad, 0x00),
        (0x00ff, 0x05),
        (0x0100, 0x05),
        (0x0199, 0x05),
        (0x01a6, 0x1b),
        (0x01ac, 0x3e),
        (0x01a7, 0x1f),
        (0x0030, 0x00),
    )

    def __init__(self, i2c: I2C, address: int = DEFAULT_ADDRESS):
        self.i2c = i2c
        self.address = address

    # --- Low level helpers -------------------------------------------------

    def _write(self, register: int, data: bytes) -> None:
        self.i2c.writeto(self.address, bytes([register >> 8, register & 0xFF]) + data)

    def _write_u8(self, register: int, value: int) -> None:
        self._write(register, bytes([value & 0xFF]))

    def _read(self, register: int, length: int) -> bytes:
        self.i2c.writeto(self.address, bytes([register >> 8, register & 0xFF]), False)
        return self.i2c.readfrom(self.address, length)

    def _read_u8(self, register: int) -> int:
        return self._read(register, 1)[0]

    # --- Public API --------------------------------------------------------

    def change_address(self, new_address: int) -> None:
        """Change the sensor I2C address. new_address must be 7-bit (0x08..0x77)."""
        self._write_u8(self._I2C_SLAVE_DEVICE_ADDRESS, new_address & 0x7F)
        self.address = new_address & 0x7F
        time.sleep_ms(5)

    def init_defaults(self) -> None:
        """Load the recommended configuration and enable range mode."""
        for register, value in self._DEFAULT_CONFIGURATION:
            self._write_u8(register, value)

        # Additional standard settings
        self._write_u8(0x0011, 0x10)  # Enables polling for new samples
        self._write_u8(self._SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)  # Range interrupt on new sample ready
        self._write_u8(0x003F, 0x46)  # Configure ADCs
        self._write_u8(0x0031, 0xFF)
        self._write_u8(0x0032, 0x00)
        self._write_u8(0x0030, 0x00)
        self._write_u8(0x001B, 0x09)  # Range inter-measurement period
        self._write_u8(self._SYSALS_INTERMEASUREMENT_PERIOD, 0x31)
        self._write_u8(0x0040, 0x63)
        self._write_u8(0x002E, 0x01)
        self._write_u8(self._SYSTEM_INTERRUPT_CLEAR, 0x07)
        self._write_u8(self._SYSTEM_FRESH_OUT_OF_RESET, 0x00)

    def read_range(self, timeout_ms: int = 20) -> int:
        """Trigger a single measurement and return the range in millimetres."""
        self._write_u8(self._SYSRANGE_START, 0x01)
        elapsed = 0
        while elapsed < timeout_ms:
            status = self._read_u8(self._RESULT_INTERRUPT_STATUS_GPIO)
            if status & 0x04:
                distance = self._read_u8(self._RESULT_RANGE_VAL)
                self._write_u8(self._SYSTEM_INTERRUPT_CLEAR, 0x07)
                return distance
            time.sleep_ms(1)
            elapsed += 1

        raise RuntimeError("Timeout waiting for VL6180X measurement")


def initialise_sensors(i2c: I2C):
    """Create VL6180X instances on the shared I2C bus."""
    sensor_defs = (
        ("left", 20, 0x2A),
        ("middle", 19, 0x2B),
        ("right", 18, 0x2C),
    )

    xshut_pins = []
    for _, pin_num, _ in sensor_defs:
        pin = Pin(pin_num, Pin.OUT)
        pin.value(0)
        xshut_pins.append(pin)
    time.sleep_ms(10)

    sensors = []
    for idx, (name, _, address) in enumerate(sensor_defs):
        pin = xshut_pins[idx]
        pin.value(1)
        time.sleep_ms(10)
        sensor = VL6180X(i2c)
        sensor.change_address(address)
        sensor.init_defaults()
        sensors.append((name, sensor))

    return sensors


def setup_sensors():
    """Initialise I2C bus and return sensors mapped by name."""
    i2c = I2C(0, sda=Pin(16), scl=Pin(17), freq=400000) #400000
    return {name: sensor for name, sensor in initialise_sensors(i2c)}


def _safe_read(sensor, timeout_ms=20, fallback_value=255):
    try:
        value = sensor.read_range(timeout_ms=timeout_ms)
        sensor._last_value = value
        return value
    except (RuntimeError, OSError):
        return fallback_value


def read_left(sensor_map, timeout_ms=20, fallback_value=80):
    return _safe_read(sensor_map["left"], timeout_ms=timeout_ms, fallback_value=fallback_value)


def read_middle(sensor_map, timeout_ms=20, fallback_value=60): #20
    return _safe_read(sensor_map["middle"], timeout_ms=timeout_ms, fallback_value=fallback_value)


def read_right(sensor_map, timeout_ms=20, fallback_value=80):
    return _safe_read(sensor_map["right"], timeout_ms=timeout_ms, fallback_value=fallback_value)

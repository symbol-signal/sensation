"""
remote_board.py - CircuitPython/Adafruit board-compatible interface for remote I2C

This module provides a drop-in replacement for CircuitPython's board.I2C() that
works with remote I2C devices over TCP. It implements the busio.I2C interface
used by Adafruit CircuitPython libraries.

Example usage:
    import remote_board
    from adafruit_bme280 import basic as adafruit_bme280

    # Configure remote board
    remote_board.set_host("192.168.1.100", 20203)

    # Use exactly like CircuitPython
    i2c = remote_board.I2C()
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

    print(f"Temperature: {bme280.temperature}°C")
"""

import threading
from typing import Optional, List, Union

import time

from sensation.remote_i2c.smbus2 import RemoteSMBus

# Global configuration for remote connection
_remote_host = None
_remote_port = 20203
_i2c_instance = None
_lock = threading.Lock()


def set_host(host: str, port: int = 20203):
    """
    Set the remote host for board.I2C() connections

    Args:
        host: IP address or hostname of the remote I2C bridge
        port: TCP port of the remote I2C bridge (default: 20203)
    """
    global _remote_host, _remote_port, _i2c_instance
    _remote_host = host
    _remote_port = port
    # Clear cached instance when host changes
    with _lock:
        if _i2c_instance:
            _i2c_instance._bus.close()
            _i2c_instance = None


class RemoteI2C:
    """
    CircuitPython busio.I2C compatible interface for remote I2C access

    This class implements the busio.I2C interface used by Adafruit CircuitPython
    libraries, allowing them to work with remote I2C devices over TCP.
    """

    def __init__(self, host: Optional[str] = None, port: Optional[int] = None):
        """
        Initialize remote I2C connection

        Args:
            host: IP address or hostname (uses global if not specified)
            port: TCP port (uses global if not specified)
        """
        if host is None:
            if _remote_host is None:
                raise ValueError("Remote host not configured. Call set_host() first.")
            host = _remote_host
        if port is None:
            port = _remote_port

        self._bus = RemoteSMBus(host, port)
        self._locked = False

    def deinit(self):
        """Deinitialize the I2C bus"""
        if self._bus:
            self._bus.close()
            self._bus = None

    def __enter__(self):
        """Context manager entry"""
        while not self.try_lock():
            time.sleep(0.001)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.unlock()

    def scan(self) -> List[int]:
        """
        Scan for I2C devices

        Returns:
            List of device addresses found
        """
        if not self._locked:
            raise RuntimeError("I2C bus must be locked to scan")
        return self._bus.scan()

    def try_lock(self) -> bool:
        """
        Attempt to lock the I2C bus

        Returns:
            True if lock acquired, False otherwise
        """
        if self._locked:
            return False
        self._locked = True
        return True

    def unlock(self):
        """Release the I2C bus lock"""
        self._locked = False

    def readfrom_into(self, address: int, buffer: Union[bytearray, memoryview],
                      start: int = 0, end: Optional[int] = None):
        """
        Read from an I2C device into a buffer

        Args:
            address: I2C device address
            buffer: Buffer to read into
            start: Start index in buffer
            end: End index in buffer (None for length of buffer)
        """
        if not self._locked:
            raise RuntimeError("I2C bus must be locked")

        if end is None:
            end = len(buffer)

        length = end - start
        if length <= 0:
            return

        # For raw I2C reads (no register), we need to use a special command
        # We'll use the raw read functionality
        try:
            data = self._bus.read_raw(address, length)
            for i in range(min(length, len(data))):
                buffer[start + i] = data[i]
        except AttributeError:
            # If read_raw is not available, fall back to byte-by-byte reading
            for i in range(length):
                try:
                    buffer[start + i] = self._bus.read_byte(address)
                except:
                    # If we can't read more bytes, stop
                    break

    def writeto(self, address: int, buffer: Union[bytes, bytearray, memoryview],
                start: int = 0, end: Optional[int] = None):
        """
        Write data from a buffer to an I2C device

        Args:
            address: I2C device address
            buffer: Buffer containing data to write
            start: Start index in buffer
            end: End index in buffer (None for length of buffer)
        """
        if not self._locked:
            raise RuntimeError("I2C bus must be locked")

        if end is None:
            end = len(buffer)

        data = bytes(buffer[start:end])
        if not data:
            return

        # For raw I2C writes (no register), use write_raw if available
        try:
            self._bus.write_raw(address, data)
        except AttributeError:
            # Fall back to byte-by-byte writing
            for byte in data:
                self._bus.write_byte(address, byte)

    def writeto_then_readfrom(self, address: int,
                              out_buffer: Union[bytes, bytearray, memoryview],
                              in_buffer: Union[bytearray, memoryview],
                              out_start: int = 0, out_end: Optional[int] = None,
                              in_start: int = 0, in_end: Optional[int] = None):
        """
        Write data to an I2C device and then read from it

        This is commonly used to write a register address and then read data.

        Args:
            address: I2C device address
            out_buffer: Buffer containing data to write
            in_buffer: Buffer to read into
            out_start: Start index in output buffer
            out_end: End index in output buffer
            in_start: Start index in input buffer
            in_end: End index in input buffer
        """
        if not self._locked:
            raise RuntimeError("I2C bus must be locked")

        if out_end is None:
            out_end = len(out_buffer)
        if in_end is None:
            in_end = len(in_buffer)

        out_data = bytes(out_buffer[out_start:out_end])
        in_length = in_end - in_start

        if len(out_data) == 1 and in_length > 0:
            # Common case: write register address, read data
            register = out_data[0]

            if in_length == 1:
                # Read single byte
                in_buffer[in_start] = self._bus.read_byte_data(address, register)
            elif in_length == 2:
                # Read word
                word = self._bus.read_word_data(address, register)
                in_buffer[in_start] = word & 0xFF
                in_buffer[in_start + 1] = (word >> 8) & 0xFF
            else:
                # Read block
                data = self._bus.read_i2c_block_data(address, register, in_length)
                for i, byte in enumerate(data):
                    in_buffer[in_start + i] = byte
        else:
            # General case: write then read with repeated start
            # This requires a combined write-read transaction
            try:
                data = self._bus.write_then_read(address, out_data, in_length)
                for i, byte in enumerate(data):
                    in_buffer[in_start + i] = byte
            except AttributeError:
                # Fall back to separate write and read if combined operation not available
                self.writeto(address, out_buffer, out_start, out_end)
                # Add a small delay to ensure write completes
                time.sleep(0.001)
                self.readfrom_into(address, in_buffer, in_start, in_end)


def I2C() -> RemoteI2C:
    """
    Get the singleton I2C interface for the board

    This mimics CircuitPython's board.I2C() behavior, returning a singleton
    instance that is shared across the application.

    Returns:
        RemoteI2C instance
    """
    global _i2c_instance

    with _lock:
        if _i2c_instance is None:
            _i2c_instance = RemoteI2C()
        return _i2c_instance


# Additional board-like attributes for compatibility
SCL = "SCL"  # Dummy pin objects
SDA = "SDA"


class RemoteBusio:
    """Module to mimic busio for imports like 'from busio import I2C'"""
    I2C = RemoteI2C


# Create a busio-like module
busio = RemoteBusio()



if __name__ == "__main__":
    # example_circuitpython_usage()
    import adafruit_sht4x

    set_host('192.168.1.105', 20203)
    sensor = adafruit_sht4x.SHT4x(I2C())

    while True:
        temperature, humidity = sensor.measurements
        print(f"Temperature: {temperature:.2f} °C, Humidity: {humidity:.2f} %")
        time.sleep(2)

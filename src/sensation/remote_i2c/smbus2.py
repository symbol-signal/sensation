"""
remote_smbus2.py - SMBus2-compatible interface for remote I2C devices

This module provides a drop-in replacement for smbus2.SMBus that works with
remote I2C devices over TCP using the Remote I2C Protocol.

Example usage:
    from remote_smbus2 import RemoteSMBus
    
    # Connect to remote I2C bridge
    bus = RemoteSMBus("192.168.1.100", 20203)
    
    # Use exactly like smbus2
    data = bus.read_byte_data(0x48, 0x00)
    bus.write_byte_data(0x48, 0x01, 0xFF)
    
    bus.close()
"""

import socket
import struct
from typing import List, Optional, Union


class RemoteSMBus:
    """SMBus2-compatible interface for remote I2C access over TCP"""
    
    # Command codes
    CMD_READ_BYTE = 0x01
    CMD_WRITE_BYTE = 0x02
    CMD_READ_BYTE_DATA = 0x03
    CMD_WRITE_BYTE_DATA = 0x04
    CMD_READ_WORD_DATA = 0x05
    CMD_WRITE_WORD_DATA = 0x06
    CMD_READ_BLOCK_DATA = 0x07
    CMD_WRITE_BLOCK_DATA = 0x08
    CMD_READ_I2C_BLOCK = 0x09
    CMD_WRITE_I2C_BLOCK = 0x0A
    CMD_SCAN = 0x10
    CMD_SET_SPEED = 0x11
    CMD_GET_INFO = 0x12

    CMD_READ_RAW = 0x0B  # Raw I2C read (no register)
    CMD_WRITE_RAW = 0x0C  # Raw I2C write (no register)
    CMD_WRITE_READ = 0x0D  # Combined write-then-read with repeated start
    
    # Status codes
    STATUS_OK = 0x00
    STATUS_NACK = 0x01
    STATUS_ERROR = 0x02
    STATUS_INVALID_CMD = 0x03
    STATUS_INVALID_PARAM = 0x04
    STATUS_TIMEOUT = 0x05
    STATUS_BUSY = 0x06
    
    def __init__(self, host: str, port: int = 20203, bus: Optional[int] = None):
        """
        Initialize remote SMBus connection
        
        Args:
            host: IP address or hostname of the remote I2C bridge
            port: TCP port of the remote I2C bridge (default: 20203)
            bus: Ignored for compatibility with smbus2
        """
        self.host = host
        self.port = port
        self.socket = None
        self.timeout = 1.0
        self._connect()
    
    def _connect(self):
        """Establish TCP connection to remote I2C bridge"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(self.timeout)
        self.socket.connect((self.host, self.port))
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
    
    def _send_command(self, cmd: int, addr: int, reg: int = 0, data: bytes = b'') -> bytes:
        """
        Send command and receive response
        
        Args:
            cmd: Command code
            addr: I2C device address
            reg: Register address
            data: Data payload
            
        Returns:
            Response data bytes
            
        Raises:
            IOError: On I2C communication errors
            ValueError: On protocol errors
        """
        # Build request
        data_len = len(data)
        request = struct.pack('>BBBH', cmd, addr, reg, data_len) + data
        
        # Send request
        self.socket.sendall(request)
        
        # Read response header (3 bytes: status + length)
        header = b''
        while len(header) < 3:
            chunk = self.socket.recv(3 - len(header))
            if not chunk:
                raise IOError("Connection closed by remote")
            header += chunk
        
        status = header[0]
        response_len = (header[1] << 8) | header[2]
        
        # Read response data
        response_data = b''
        while len(response_data) < response_len:
            chunk = self.socket.recv(response_len - len(response_data))
            if not chunk:
                raise IOError("Connection closed by remote")
            response_data += chunk
        
        # Check status
        if status == self.STATUS_OK:
            return response_data
        elif status == self.STATUS_NACK:
            raise IOError("No ACK from I2C device")
        elif status == self.STATUS_INVALID_CMD:
            raise ValueError("Invalid command")
        elif status == self.STATUS_INVALID_PARAM:
            raise ValueError("Invalid parameters")
        elif status == self.STATUS_TIMEOUT:
            raise IOError("I2C timeout")
        elif status == self.STATUS_BUSY:
            raise IOError("I2C bus busy")
        else:
            raise IOError(f"I2C error: status {status}")
    
    def close(self):
        """Close the connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
    
    def __enter__(self):
        """Context manager entry"""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit"""
        self.close()

    def read_raw(self, address: int, length: int) -> List[int]:
        """
        Read raw data from I2C device without specifying a register

        Args:
            address: I2C device address
            length: Number of bytes to read

        Returns:
            List of bytes read
        """
        if length > 32:
            raise ValueError("Maximum read length is 32 bytes")

        # For CMD_READ_RAW, we pass the length in the reg field, with empty data payload
        # Build request: cmd, addr, length (in reg field), 0 (data_len)
        request = struct.pack('>BBBH', self.CMD_READ_RAW, address, length, 0)

        # Send request
        self.socket.sendall(request)

        # Read response header (3 bytes: status + length)
        header = b''
        while len(header) < 3:
            chunk = self.socket.recv(3 - len(header))
            if not chunk:
                raise IOError("Connection closed by remote")
            header += chunk

        status = header[0]
        response_len = (header[1] << 8) | header[2]

        # Read response data
        response_data = b''
        while len(response_data) < response_len:
            chunk = self.socket.recv(response_len - len(response_data))
            if not chunk:
                raise IOError("Connection closed by remote")
            response_data += chunk

        # Check status
        if status == self.STATUS_OK:
            return list(response_data)
        elif status == self.STATUS_NACK:
            raise IOError("No ACK from I2C device")
        else:
            raise IOError(f"I2C error: status {status}")

    def write_raw(self, address: int, data: Union[bytes, List[int]]):
        """
        Write raw data to I2C device without specifying a register

        Args:
            address: I2C device address
            data: Data to write
        """
        if isinstance(data, list):
            data = bytes(data)

        if len(data) > 32:
            raise ValueError("Maximum write length is 32 bytes")

        self._send_command(self.CMD_WRITE_RAW, address, 0, data)

    def write_then_read(self, address: int, write_data: Union[bytes, List[int]], read_length: int) -> List[int]:
        """
        Perform a combined write-then-read transaction with repeated start

        Args:
            address: I2C device address
            write_data: Data to write first
            read_length: Number of bytes to read after write

        Returns:
            List of bytes read
        """
        if isinstance(write_data, list):
            write_data = bytes(write_data)

        if len(write_data) > 32 or read_length > 32:
            raise ValueError("Maximum length is 32 bytes")

        # For write-then-read, we pass read_length in the reg field
        response = self._send_command(self.CMD_WRITE_READ, address, read_length, write_data)
        return list(response)

    # SMBus2 compatible methods
    
    def read_byte(self, i2c_addr: int, force: bool = None) -> int:
        """
        Read a single byte from a device
        
        Args:
            i2c_addr: I2C device address
            force: Ignored for compatibility
            
        Returns:
            Byte value read
        """
        data = self._send_command(self.CMD_READ_BYTE, i2c_addr)
        return data[0]
    
    def write_byte(self, i2c_addr: int, value: int, force: bool = None):
        """
        Write a single byte to a device
        
        Args:
            i2c_addr: I2C device address
            value: Byte value to write
            force: Ignored for compatibility
        """
        self._send_command(self.CMD_WRITE_BYTE, i2c_addr, data=bytes([value]))
    
    def read_byte_data(self, i2c_addr: int, register: int, force: bool = None) -> int:
        """
        Read a single byte from a designated register
        
        Args:
            i2c_addr: I2C device address
            register: Register address
            force: Ignored for compatibility
            
        Returns:
            Byte value read
        """
        data = self._send_command(self.CMD_READ_BYTE_DATA, i2c_addr, register)
        return data[0]
    
    def write_byte_data(self, i2c_addr: int, register: int, value: int, force: bool = None):
        """
        Write a byte to a given register
        
        Args:
            i2c_addr: I2C device address
            register: Register address
            value: Byte value to write
            force: Ignored for compatibility
        """
        self._send_command(self.CMD_WRITE_BYTE_DATA, i2c_addr, register, bytes([value]))
    
    def read_word_data(self, i2c_addr: int, register: int, force: bool = None) -> int:
        """
        Read a single word (2 bytes) from a given register
        
        Args:
            i2c_addr: I2C device address
            register: Register address
            force: Ignored for compatibility
            
        Returns:
            Word value read (little-endian)
        """
        data = self._send_command(self.CMD_READ_WORD_DATA, i2c_addr, register)
        return data[0] | (data[1] << 8)  # Little-endian
    
    def write_word_data(self, i2c_addr: int, register: int, value: int, force: bool = None):
        """
        Write a word (2 bytes) to a given register
        
        Args:
            i2c_addr: I2C device address
            register: Register address
            value: Word value to write (little-endian)
            force: Ignored for compatibility
        """
        # Pack as little-endian
        data = bytes([value & 0xFF, (value >> 8) & 0xFF])
        self._send_command(self.CMD_WRITE_WORD_DATA, i2c_addr, register, data)
    
    def read_block_data(self, i2c_addr: int, register: int, force: bool = None) -> List[int]:
        """
        Read a block of up to 32 bytes from a given register
        
        Args:
            i2c_addr: I2C device address
            register: Register address
            force: Ignored for compatibility
            
        Returns:
            List of bytes read
        """
        data = self._send_command(self.CMD_READ_BLOCK_DATA, i2c_addr, register)
        # First byte is the length in SMBus block protocol
        length = data[0]
        return list(data[1:length+1])
    
    def write_block_data(self, i2c_addr: int, register: int, data: List[int], force: bool = None):
        """
        Write a block of data to a given register
        
        Args:
            i2c_addr: I2C device address
            register: Register address
            data: List of bytes to write (max 32)
            force: Ignored for compatibility
        """
        if len(data) > 32:
            raise ValueError("Block data cannot exceed 32 bytes")
        self._send_command(self.CMD_WRITE_BLOCK_DATA, i2c_addr, register, bytes(data))
    
    def read_i2c_block_data(self, i2c_addr: int, register: int, length: int, 
                           force: bool = None) -> List[int]:
        """
        Read a block of bytes from a given register
        
        Args:
            i2c_addr: I2C device address
            register: Register address
            length: Number of bytes to read
            force: Ignored for compatibility
            
        Returns:
            List of bytes read
        """
        if length > 32:
            raise ValueError("Cannot read more than 32 bytes")
        # For I2C block read, we pass the length in the LEN field
        data = self._send_command(self.CMD_READ_I2C_BLOCK, i2c_addr, register)
        return list(data[:length])
    
    def write_i2c_block_data(self, i2c_addr: int, register: int, data: List[int], 
                            force: bool = None):
        """
        Write a block of bytes to a given register
        
        Args:
            i2c_addr: I2C device address
            register: Register address
            data: List of bytes to write
            force: Ignored for compatibility
        """
        if len(data) > 32:
            raise ValueError("Block data cannot exceed 32 bytes")
        self._send_command(self.CMD_WRITE_I2C_BLOCK, i2c_addr, register, bytes(data))
    
    def i2c_rdwr(self, *i2c_msgs):
        """
        Perform I2C read/write operations
        
        Note: This is not implemented in the remote protocol.
        Use the individual read/write methods instead.
        """
        raise NotImplementedError("i2c_rdwr is not supported in remote I2C protocol")
    
    # Additional remote-specific methods
    
    def scan(self) -> List[int]:
        """
        Scan for I2C devices on the bus
        
        Returns:
            List of device addresses found
        """
        data = self._send_command(self.CMD_SCAN, 0)
        return list(data)
    
    def set_speed(self, freq_hz: int):
        """
        Set I2C bus speed
        
        Args:
            freq_hz: Frequency in Hz
        """
        data = struct.pack('>I', freq_hz)
        self._send_command(self.CMD_SET_SPEED, 0, data=data)
    
    def get_info(self) -> dict:
        """
        Get information about the remote I2C bridge
        
        Returns:
            Dictionary with bridge information
        """
        data = self._send_command(self.CMD_GET_INFO, 0)
        info_str = data.decode('utf-8')
        return eval(info_str)  # Simple parsing, could use json in production


# For compatibility
SMBus = RemoteSMBus

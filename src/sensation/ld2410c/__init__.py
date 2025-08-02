"""
Module for interacting with the HLK-LD2410C 24GHz millimeter-wave radar sensor.

This module provides a high-level interface to communicate with the LD2410C sensor
using serial communication. It allows reading presence detection data and handling 
sensor events in normal operation mode.

Example usage:
    import asyncio
    from serial import Serial
    from sensation.ld2410c import Sensor

    async def main():
        tcp_serial = serialio.serial_for_url("serial+tcp://192.168.1.110:20202", 256000)
        sensor = Sensor("test_sensor", tcp_serial)
        await tcp_serial.open()
        await sensor.start_reading()
        # Sensor will notify observers of presence changes
        await asyncio.sleep(60)
        await sensor.close()

    asyncio.run(main())
"""
import asyncio
import logging
import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Callable, Optional, Union, Awaitable

import serial
from serial.serialutil import SerialException

from sensation.common import SensorId, SensorType

log = logging.getLogger(__name__)

# Protocol constants
FRAME_HEADER_NORMAL = b'\xF4\xF3\xF2\xF1'
FRAME_FOOTER_NORMAL = b'\xF8\xF7\xF6\xF5'
FRAME_HEADER_CMD = b'\xFD\xFC\xFB\xFA'
FRAME_FOOTER_CMD = b'\x04\x03\x02\x01'

# Data type constants
DATA_TYPE_ENGINEERING = 0x01
DATA_TYPE_NORMAL = 0x02

# Target state constants
TARGET_STATE_NO_TARGET = 0x00
TARGET_STATE_MOVING = 0x01
TARGET_STATE_STATIONARY = 0x02
TARGET_STATE_BOTH = 0x03


class TargetState(IntEnum):
    """Target detection state."""
    NO_TARGET = 0x00
    MOVING = 0x01
    STATIONARY = 0x02
    BOTH = 0x03

    @property
    def has_presence(self) -> bool:
        """Check if any target is detected."""
        return self != TargetState.NO_TARGET


@dataclass
class DetectionData:
    """
    Represents detection data from the sensor in normal mode.
    
    Attributes:
        state: Current target state (no target, moving, stationary, or both)
        moving_distance: Distance to moving target in cm (0 if no moving target)
        moving_energy: Energy value of moving target (0-100)
        stationary_distance: Distance to stationary target in cm (0 if no stationary target)
        stationary_energy: Energy value of stationary target (0-100)
        detection_distance: Overall detection distance in cm
    """
    state: TargetState
    moving_distance: int
    moving_energy: int
    stationary_distance: int
    stationary_energy: int
    detection_distance: int

    @property
    def has_presence(self) -> bool:
        """Check if any presence is detected."""
        return self.state.has_presence

    @property
    def presence_info(self) -> str:
        """Get human-readable presence information."""
        if self.state == TargetState.NO_TARGET:
            return "No presence detected"
        elif self.state == TargetState.MOVING:
            return f"Moving target at {self.moving_distance}cm (energy: {self.moving_energy})"
        elif self.state == TargetState.STATIONARY:
            return f"Stationary target at {self.stationary_distance}cm (energy: {self.stationary_energy})"
        else:  # BOTH
            return (f"Moving target at {self.moving_distance}cm (energy: {self.moving_energy}), "
                   f"Stationary target at {self.stationary_distance}cm (energy: {self.stationary_energy})")


class FrameParser:
    """Parser for LD2410C data frames."""
    
    def __init__(self):
        self.buffer = bytearray()
        
    def add_data(self, data: bytes):
        """Add received data to buffer."""
        self.buffer.extend(data)
    
    def parse_frame(self) -> Optional[DetectionData]:
        """
        Parse a complete frame from the buffer.
        
        Returns:
            DetectionData if a valid frame is found and parsed, None otherwise
        """
        # Look for normal data frame header
        header_idx = self.buffer.find(FRAME_HEADER_NORMAL)
        if header_idx == -1:
            # Keep last few bytes in case header is split
            if len(self.buffer) > 8:
                self.buffer = self.buffer[-8:]
            return None
            
        # Remove data before header
        if header_idx > 0:
            self.buffer = self.buffer[header_idx:]
        
        # Check if we have minimum frame size (header + length + data type + tail + footer)
        if len(self.buffer) < 4 + 2 + 1 + 1 + 1 + 1 + 4:
            return None
            
        # Parse frame length (little-endian)
        frame_length = struct.unpack('<H', self.buffer[4:6])[0]
        
        # Check if we have complete frame
        total_frame_size = 4 + 2 + frame_length + 4  # header + length + data + footer
        if len(self.buffer) < total_frame_size:
            return None
            
        # Verify footer
        footer_start = 4 + 2 + frame_length
        if self.buffer[footer_start:footer_start + 4] != FRAME_FOOTER_NORMAL:
            # Invalid frame, skip header and try again
            self.buffer = self.buffer[4:]
            return None
            
        # Extract frame data
        frame_data = self.buffer[6:footer_start]
        
        # Parse frame data
        if len(frame_data) < 13:  # Minimum data size for normal mode
            self.buffer = self.buffer[total_frame_size:]
            return None
            
        data_type = frame_data[0]
        if data_type != DATA_TYPE_NORMAL:
            # Not normal mode data, skip
            self.buffer = self.buffer[total_frame_size:]
            return None
            
        # Check for data header 0xAA
        if frame_data[1] != 0xAA:
            self.buffer = self.buffer[total_frame_size:]
            return None
            
        # Parse target data (all little-endian)
        try:
            target_state = TargetState(frame_data[2])
            moving_distance = struct.unpack('<H', frame_data[3:5])[0]
            moving_energy = frame_data[5]
            stationary_distance = struct.unpack('<H', frame_data[6:8])[0]
            stationary_energy = frame_data[8]
            detection_distance = struct.unpack('<H', frame_data[9:11])[0]
            
            # Verify tail and checksum
            if frame_data[11] != 0x55 or frame_data[12] != 0x00:
                self.buffer = self.buffer[total_frame_size:]
                return None
                
            # Remove processed frame from buffer
            self.buffer = self.buffer[total_frame_size:]
            
            return DetectionData(
                state=target_state,
                moving_distance=moving_distance,
                moving_energy=moving_energy,
                stationary_distance=stationary_distance,
                stationary_energy=stationary_energy,
                detection_distance=detection_distance
            )
            
        except (struct.error, ValueError) as e:
            log.debug(f"Failed to parse frame data: {e}")
            self.buffer = self.buffer[total_frame_size:]
            return None


class PresenceHandler:
    """
    Handles presence detection events from the sensor.
    
    Attributes:
        observers: List of observer callbacks to be notified of presence changes
        presence_state: Current presence state
        last_data: Last received detection data
    """
    
    def __init__(self):
        self.observers: List[Callable[[bool, DetectionData], Union[None, Awaitable[None]]]] = []
        self.presence_state: Optional[bool] = None
        self.last_data: Optional[DetectionData] = None
        
    async def __call__(self, data: DetectionData):
        """
        Process detection data and notify observers if presence state changes.
        
        Args:
            data: Parsed detection data from sensor
        """
        current_presence = data.has_presence
        self.last_data = data
        
        if self.presence_state != current_presence:
            self.presence_state = current_presence
            
            for observer in self.observers:
                try:
                    result = observer(current_presence, data)
                    if isinstance(result, Awaitable):
                        await result
                except Exception:
                    log.exception(f"Error in presence observer: {observer}")


@dataclass
class SensorStatus:
    """
    Represents the current status of the sensor.
    
    Attributes:
        sensor_id: Unique identifier of the sensor
        port: Serial port to which the sensor is connected
        is_reading: Whether the sensor is currently reading data
        last_presence: Last detected presence state
        last_data: Last received detection data
    """
    sensor_id: SensorId
    port: str
    is_reading: bool
    last_presence: Optional[bool]
    last_data: Optional[DetectionData]


class Sensor:
    """
    Represents the LD2410C sensor with async operations.
    
    Attributes:
        sensor_id: Unique identifier of the sensor
        serial: Serial connection to the sensor
        presence_handler: Handler for presence detection events
    """
    
    def __init__(self, sensor_name: str, serial_con: serial.Serial):
        """
        Initialize the Sensor object.
        
        Args:
            sensor_name: Name of the sensor
            serial_con: Serial connection to the sensor
        """
        self.sensor_id = SensorId(SensorType.LD2410C, sensor_name)
        self.serial = serial_con
        self.presence_handler = PresenceHandler()
        self._parser = FrameParser()
        self._lock = asyncio.Lock()
        self._reading_task: Optional[asyncio.Task] = None
        
    async def _read_data(self, size: int = 256) -> bytes:
        """Read data from serial port asynchronously."""
        # Check if serial has async read method
        if hasattr(self.serial.read, '__call__') and asyncio.iscoroutinefunction(self.serial.read):
            return await self.serial.read(size)
        else:
            # For sync serial operations, use executor
            loop = asyncio.get_event_loop()
            return await loop.run_in_executor(None, self.serial.read, size)
        
    async def read(self):
        """
        Read sensor data continuously until stopped.
        Notifies presence handler when valid frames are received.
        """
        while True:
            try:
                # Read available data
                in_wait = self.serial.in_waiting
                if callable(in_wait):  # sockio.aio.TCP has in_waiting as method
                    in_wait = in_wait()
                if in_wait:
                    data = await self._read_data(min(in_wait, 512))
                    if data:
                        async with self._lock:
                            self._parser.add_data(data)
                            
                            # Try to parse frames
                            while detection_data := self._parser.parse_frame():
                                log.debug(f"[frame_parsed] sensor=[{self.sensor_id}] data=[{detection_data}]")
                                await self.presence_handler(detection_data)
                
                # Small delay to prevent busy waiting
                await asyncio.sleep(0.01)
                
            except asyncio.CancelledError:
                raise
            except SerialException:
                log.exception(f"[sensor_error] sensor=[{self.sensor_id}]")
                await asyncio.sleep(5)
            except Exception:
                log.exception(f"[unexpected_error] sensor=[{self.sensor_id}]")
                await asyncio.sleep(1)
                
    def start_reading(self) -> Optional[asyncio.Task]:
        """
        Start reading sensor data in a separate asyncio task.
        
        Returns:
            The created task if a new one was started, None if already reading
        """
        if self._reading_task and not self._reading_task.done():
            return None
            
        self._reading_task = asyncio.create_task(self.read())
        log.info(f"[reading_started] sensor=[{self.sensor_id}] task=[{self._reading_task}]")
        return self._reading_task
        
    async def stop_reading(self):
        """Stop reading sensor data and wait for the reading task to terminate."""
        if not self._reading_task:
            return
            
        task = self._reading_task
        self._reading_task = None
        
        try:
            task.cancel()
            await task
        except asyncio.CancelledError:
            pass  # Expected
        finally:
            log.info(f"[reading_stopped] sensor=[{self.sensor_id}]")
            
    async def status(self) -> SensorStatus:
        """
        Get the current status of the sensor.
        
        Returns:
            Current sensor status
        """
        async with self._lock:
            return SensorStatus(
                sensor_id=self.sensor_id,
                port=self.serial.port,
                is_reading=self._reading_task is not None and not self._reading_task.done(),
                last_presence=self.presence_handler.presence_state,
                last_data=self.presence_handler.last_data
            )
            
    def add_presence_observer(self, observer: Callable[[bool, DetectionData], Union[None, Awaitable[None]]]):
        """
        Add an observer to be notified of presence changes.
        
        Args:
            observer: Callback function that receives (presence_detected, detection_data)
        """
        self.presence_handler.observers.append(observer)
        
    def remove_presence_observer(self, observer: Callable[[bool, DetectionData], Union[None, Awaitable[None]]]):
        """
        Remove a presence observer.
        
        Args:
            observer: The observer to remove
        """
        if observer in self.presence_handler.observers:
            self.presence_handler.observers.remove(observer)
            
    async def _send_command(self, command_word: int, command_value: bytes = b'') -> Optional[bytes]:
        """
        Send a command to the sensor and wait for response.
        
        Args:
            command_word: 2-byte command word (little-endian)
            command_value: Command value bytes
            
        Returns:
            Response bytes if successful, None otherwise
        """
        # Build command frame
        frame_data = struct.pack('<H', command_word) + command_value
        frame_length = len(frame_data)
        
        command_frame = (
            FRAME_HEADER_CMD +
            struct.pack('<H', frame_length) +
            frame_data +
            FRAME_FOOTER_CMD
        )
        
        async with self._lock:
            # Clear any existing data
            if hasattr(self.serial, 'reset_input_buffer'):
                self.serial.reset_input_buffer()
            
            # Send command
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(None, self.serial.write, command_frame)
            if hasattr(self.serial, 'flush'):
                await loop.run_in_executor(None, self.serial.flush)
            
            log.debug(f"[command_sent] sensor=[{self.sensor_id}] cmd=0x{command_word:04X} data={command_value.hex()}")
            
            # Wait for response (up to 2 seconds)
            response_buffer = bytearray()
            start_time = asyncio.get_event_loop().time()
            
            while (asyncio.get_event_loop().time() - start_time) < 2.0:
                in_waiting = self.serial.in_waiting
                if in_waiting > 0:
                    data = await loop.run_in_executor(None, self.serial.read, min(in_waiting, 256))
                    response_buffer.extend(data)
                    
                    # Look for response frame
                    header_idx = response_buffer.find(FRAME_HEADER_CMD)
                    if header_idx != -1:
                        # Check if we have enough data for frame length
                        if len(response_buffer) >= header_idx + 6:
                            frame_length = struct.unpack('<H', response_buffer[header_idx + 4:header_idx + 6])[0]
                            total_frame_size = 4 + 2 + frame_length + 4
                            
                            if len(response_buffer) >= header_idx + total_frame_size:
                                # Extract complete frame
                                frame_end = header_idx + total_frame_size
                                if response_buffer[frame_end - 4:frame_end] == FRAME_FOOTER_CMD:
                                    response_data = response_buffer[header_idx + 6:frame_end - 4]
                                    log.debug(f"[command_response] sensor=[{self.sensor_id}] response={response_data.hex()}")
                                    return bytes(response_data)
                
                await asyncio.sleep(0.01)
            
            log.warning(f"[command_timeout] sensor=[{self.sensor_id}] cmd=0x{command_word:04X}")
            return None
    
    async def enable_configuration(self) -> bool:
        """
        Enable configuration mode. Must be called before other commands.
        
        Returns:
            True if successful, False otherwise
        """
        response = await self._send_command(0x00FF, struct.pack('<H', 0x0001))
        if response and len(response) >= 6:
            # Check for success response: command_word | 0x0100, status, protocol_version, buffer_size
            if response[0:2] == b'\xFF\x01' and response[2:4] == b'\x00\x00':
                log.info(f"[config_enabled] sensor=[{self.sensor_id}]")
                return True
        
        log.warning(f"[config_enable_failed] sensor=[{self.sensor_id}] response={response.hex() if response else 'None'}")
        return False
    
    async def end_configuration(self) -> bool:
        """
        End configuration mode.
        
        Returns:
            True if successful, False otherwise
        """
        response = await self._send_command(0x00FE)
        if response and len(response) >= 4:
            # Check for success response: command_word | 0x0100, status
            if response[0:2] == b'\xFE\x01' and response[2:4] == b'\x00\x00':
                log.info(f"[config_ended] sensor=[{self.sensor_id}]")
                return True
        
        log.warning(f"[config_end_failed] sensor=[{self.sensor_id}] response={response.hex() if response else 'None'}")
        return False
    
    async def read_firmware_version(self) -> Optional[str]:
        """
        Read firmware version from sensor.
        
        Returns:
            Firmware version string if successful, None otherwise
        """
        response = await self._send_command(0x00A0)
        if response and len(response) >= 12:
            # Parse firmware version: command_word | 0x0100, status, fw_type, major_version, minor_version
            if response[0:2] == b'\x00\x01' and response[2:4] == b'\x00\x00':
                fw_type = struct.unpack('<H', response[4:6])[0]
                major_version = struct.unpack('<H', response[6:8])[0]
                minor_version = struct.unpack('<I', response[8:12])[0]
                
                version_str = f"V{major_version}.{(minor_version >> 24) & 0xFF:02d}.{minor_version & 0xFFFFFF:08d}"
                log.info(f"[firmware_version] sensor=[{self.sensor_id}] version=[{version_str}]")
                return version_str
        
        log.warning(f"[firmware_version_failed] sensor=[{self.sensor_id}] response={response.hex() if response else 'None'}")
        return None
    
    async def test_communication(self) -> bool:
        """
        Test communication with sensor by reading firmware version.
        
        Returns:
            True if communication is working, False otherwise
        """
        if await self.enable_configuration():
            version = await self.read_firmware_version()
            await self.end_configuration()
            return version is not None
        return False

    async def close(self):
        """Close the sensor connection and stop reading."""
        await self.stop_reading()
        if hasattr(self.serial, 'close'):
            self.serial.close()
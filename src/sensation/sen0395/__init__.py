"""
Module for interacting with the DFRobot SEN0395 24GHz millimeter-wave radar sensor.

This module provides a high-level interface to control and communicate with the SEN0395 sensor
using serial communication. It allows configuring the sensor's settings, reading presence detection data,
and handling sensor events.

Example usage:
    from serial import Serial
    from sensation.sen0395 import Sensor

    sensor = Sensor("sensor_name", Serial('/dev/ttyAMA0', 115200, timeout=1))
    sensor.start_scanning()
    presence = sensor.read_presence()
    sensor.close()
"""
import asyncio
import logging
import re
import time
from dataclasses import dataclass
from enum import Enum
from functools import wraps
from threading import RLock, Thread
from typing import List, Callable, Optional, Dict, Awaitable, Union

import serial
from serial.serialutil import SerialException

from sensation.common import SensorId, SensorType

log = logging.getLogger(__name__)

SAVE_CONFIG_PARAMETERS = ("0x45670123", "0xCDEF89AB", "0x956128C6", "0xDF54AC89")

PRESENCE_PATTERN = r'^\$JYBSS,([01])'


class Command(Enum):
    """Enumeration of supported sensor commands."""
    SENSOR_START = ("sensorStart", False)
    SENSOR_STOP = ("sensorStop", False)
    RESET_SYSTEM = ("resetSystem", False)
    LATENCY_CONFIG = ("outputLatency", True)
    DETECTION_RANGE_CONFIG = ("detRangeCfg", True)
    SAVE_CONFIG = ("saveCfg", False)
    NONE = ('none', False)

    def __new__(cls, value, is_config):
        obj = object.__new__(cls)
        obj._value_ = value
        obj.is_config = is_config
        return obj

    @classmethod
    def from_value(cls, value):
        """
        Get the `Command` enum member based on its command value.

        Args:
            value (str): The string representation of the command.

        Returns:
            Command: The corresponding `Command` enum member.
        """
        for member in cls:
            if member.value == value:
                return member
        return Command.NONE

    def __bool__(self):
        return self != Command.NONE


class CommandResult(Enum):
    """Enumeration of command execution results."""
    DONE = 'Done'
    NOT_APPLICABLE = 'N/A'
    ERROR = 'Error'
    MISSING = 'Missing'
    UNKNOWN = 'Unknown'

    def __bool__(self):
        return self == CommandResult.DONE or self == CommandResult.NOT_APPLICABLE


def is_present(output) -> Optional[bool]:
    """
    Parse sensor presence output value.
    Possible values:
     - `$JYBSS,0, , , *` => no presence
     - `$JYBSS,1, , , *` => presence

    Args:
        output (str): The sensor presence output string.

    Returns:
        Optional[bool]:
            `True` if presence is detected,
            `False` if no presence,
            `None` if the value is not the presence output.
    """
    presence_match = re.match(PRESENCE_PATTERN, output)
    if not presence_match:
        return None

    return bool(int(presence_match.group(1)))


def parse_command_result(output) -> CommandResult:
    """
    Parse the command execution result from the sensor output.

    Args:
        output (str): The sensor output string.

    Returns:
        CommandResult: The command execution result or `CommandResult.UNKNOWN` if not a command result.
    """
    if not output:
        return CommandResult.MISSING
    try:
        return CommandResult(output)
    except ValueError:
        return CommandResult.UNKNOWN


def parse_command(output) -> Command:
    """
    Parse the command from the sensor output.

    Args:
        output (str): The sensor output string.

    Returns:
        Command: The parsed command or `Command.NONE` if not a command.
    """
    for c in Command:
        for o in output.split():
            if o == c.value:
                return c

    return Command.NONE


class Output:
    """
    Represents a parsed sensor output line. This helps to determine the type of the output.

    Attributes:
        output (str): The raw sensor output line string.
        presence (Optional[bool]): Indicates presence detection (True/False) or None if not a presence info.
        command_result (CommandResult): The result of the command execution or `CommandResult.UNKNOWN`.
        command (Command): The parsed command or `Command.NONE`.
        command_params (tuple): The parameters of the parsed command or empty tuple.
        message (Optional[str]): Additional message in the output, if any.
    """

    def __init__(self, value):
        self.output = value
        self.presence = is_present(value)
        self.command_result = parse_command_result(value)
        self.command = parse_command(value)
        if self.command:
            self.command_params = value.replace(self.command.value, '').split()
        else:
            self.command_params = tuple()
        if all(not bool(it) for it in (self.presence, self.command_result, self.command, self.command_params)):
            self.message = value
        else:
            self.message = None

    def __repr__(self):
        return self.output


class CommandResponse:
    """
    Represents the full (multi-line) response to a sensor command provided as a list of `Output` instances.

    Attributes:
        outputs (List[Output]): The list of parsed sensor outputs.
    """

    def __init__(self, outputs):
        self.outputs = outputs or []

    def serialize(self) -> Dict:
        """
        Serialize the CommandResponse object to a dictionary.

        Returns:
            Dict: The serialized representation of the CommandResponse.
        """
        return {
            "outputs": [output.output for output in self.outputs],
        }

    @classmethod
    def deserialize(cls, data: Dict):
        """
        Deserialize a dictionary to a CommandResponse object.

        Args:
            data (Dict): The serialized data.

        Returns:
            CommandResponse: The deserialized CommandResponse object.
        """
        outputs = [Output(output) for output in data["outputs"]]
        return cls(outputs=outputs)

    def __bool__(self):
        return bool(self.command_result)

    @property
    def command_echo(self) -> Optional[str]:
        """
        Get the echoed command from the sensor command response.

        Returns:
            Optional[str]: The echoed command string, or None if not found.
        """
        if not self.outputs:
            return None

        return self.outputs[0].output

    @property
    def command(self) -> Command:
        """
        Get the parsed command from the sensor command response.

        Returns:
            Command: The parsed command.
        """
        if not self.outputs:
            return Command.NONE

        return self.outputs[0].command

    @property
    def message(self) -> Optional[str]:
        """
        Get the additional message from the sensor command response.

        Returns:
            Optional[str]: The additional message, or None if not found.
        """
        if len(self.outputs) < 3:
            return None

        return self.outputs[-2].message

    @property
    def command_result(self) -> CommandResult:
        """
        Get the command execution result from the sensor command response.

        Returns:
            CommandResult: The command execution result.
        """
        if len(self.outputs) < 2:
            return CommandResult.MISSING

        if self.command == Command.RESET_SYSTEM:
            return CommandResult.NOT_APPLICABLE

        return self.outputs[-1].command_result

    def __str__(self):
        return f"outputs={self.outputs} result={self.command_result.value}"

    def __repr__(self):
        return f"CommandResponse(outputs={self.outputs})"


@dataclass
class ConfigChainResponse:
    """
    Represents a series of command responses to a configuration command chain.

    To successfully store a new configuration the sensor must be stopped and the "save configuration" command
    must follow the configuration change.

    Attributes:
        pause_cmd (Optional[CommandResponse]): The response to the pause command, if applicable (sensor was scanning).
        cfg_cmd (Optional[CommandResponse]): The response to the configuration command.
        save_cmd (Optional[CommandResponse]): The response to the save configuration command.
        resume_cmd (Optional[CommandResponse]): The response to the resume (scanning) command, if applicable.
    """
    pause_cmd: Optional[CommandResponse] = None
    cfg_cmd: Optional[CommandResponse] = None
    save_cmd: Optional[CommandResponse] = None
    resume_cmd: Optional[CommandResponse] = None

    def serialize(self) -> Dict:
        """
        Serialize the ConfigChainResponse object to a dictionary.

        Returns:
            Dict: The serialized representation of the ConfigChainResponse.
        """
        return {
            "pause_cmd": self.pause_cmd.serialize() if self.pause_cmd else None,
            "cfg_cmd": self.cfg_cmd.serialize() if self.cfg_cmd else None,
            "save_cmd": self.save_cmd.serialize() if self.save_cmd else None,
            "resume_cmd": self.resume_cmd.serialize() if self.resume_cmd else None,
        }

    @classmethod
    def deserialize(cls, data: Dict) -> 'ConfigChainResponse':
        """
        Deserialize a dictionary to a ConfigChainResponse object.

        Args:
            data (Dict): The serialized data.

        Returns:
            ConfigChainResponse: The deserialized ConfigChainResponse object.
        """
        return cls(
            pause_cmd=CommandResponse.deserialize(data['pause_cmd']) if data.get('pause_cmd') else None,
            cfg_cmd=CommandResponse.deserialize(data['cfg_cmd']) if data.get('cfg_cmd') else None,
            save_cmd=CommandResponse.deserialize(data['save_cmd']) if data.get('save_cmd') else None,
            resume_cmd=CommandResponse.deserialize(data['resume_cmd']) if data.get('resume_cmd') else None,
        )

    def __bool__(self):
        if not (self.cfg_cmd and self.save_cmd):
            return False

        return all(res is None or bool(res) for res in (self.pause_cmd, self.resume_cmd))


class PresenceHandler:
    """
    Handles presence detection events from the sensor.

    Attributes:
        observers (List[Callable[[bool], None]]): List of observer callbacks to be notified of presence changes.
        presence_value (Optional[bool]): The current presence detection value.
    """

    def __init__(self):
        self.observers: List[Callable[[bool], None]] = []
        self.presence_value: bool | None = None

    def __call__(self, output):
        """
        Process the sensor output and notify observers if presence detection changes.

        Args:
            output (Output): The parsed sensor output.
        """
        if output.presence is None:
            return

        if self.presence_value != output.presence:
            self.presence_value = output.presence

            for observer in self.observers:
                observer(self.presence_value)


@dataclass
class SensorStatus:
    """
    Represents the status of the sensor.

    Attributes:
        sensor_id (SensorId): The unique identifier of the sensor.
        port (str): The serial port to which the sensor is connected.
        timeout (Optional[int]): The timeout value for serial communication.
        is_reading (bool): Indicates whether this instance is currently reading the sensor data (event notification).
        is_scanning (bool): Indicates whether the sensor is currently scanning.
    """
    sensor_id: SensorId
    port: str
    timeout: Optional[int]
    is_reading: bool
    is_scanning: bool

    @classmethod
    def deserialize(cls, as_dict):
        """
        Deserialize a dictionary to a SensorStatus object.

        Args:
            as_dict (Dict): The serialized data.

        Returns:
            SensorStatus: The deserialized SensorStatus object.
        """
        return cls(
            SensorId.deserialize(as_dict["sensor_id"]),
            as_dict["port"],
            as_dict["timeout"],
            as_dict["is_reading"],
            as_dict["is_scanning"],
        )

    def serialize(self):
        """
        Serialize the SensorStatus object to a dictionary.

        Returns:
            Dict: The serialized representation of the SensorStatus.
        """
        return {
            "sensor_id": self.sensor_id.serialize(),
            "port": self.port,
            "timeout": self.timeout,
            "is_reading": self.is_reading,
            "is_scanning": self.is_scanning
        }


def synchronized(method):
    """
    A decorator to lock methods for thread-safe operation and serial execution.
    """

    @wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)

    return wrapper


def range_segments(params):
    """
    Validate and process range segment parameters (for "Sensor Detection Area Configuration" command).

    Args:
        params (List[int]): List of range segment parameters.

    Returns:
        List[Tuple[int, int]]: List of range segment tuples (start, end).

    Raises:
        ValueError: If the segment parameters are invalid.
    """
    if len(params) % 2 != 0:
        raise ValueError('Missing end index of a sensing area')
    if params[0] < 0 or params[-1] > 127:
        raise ValueError('Segment start indices must be >= 0 and end indices <= 127')

    segments = []
    for idx, param in enumerate(params):
        if idx > 0 and params[idx - 1] >= param:
            raise ValueError('Index value must be greater than previous value')
        if idx % 2 == 0:
            segments.append((param, params[idx + 1]))

    return segments


class Sensor:
    """
    Represents the SEN0395 sensor.

    Attributes:
        sensor_id (SensorId): The unique identifier of the sensor.
        serial (Serial): The serial connection to the sensor.
        handlers (List[Callable[[Output], None]]): List of output handlers.
    """

    def __init__(self, sensor_name, serial_con):
        """
        Initialize the Sensor object.

        Args:
            sensor_name (str): The name of the sensor.
            serial_con (Serial): The serial connection to the sensor.
        """
        self.sensor_id = SensorId(SensorType.SEN0395, sensor_name)
        self.serial = serial_con
        self.handlers: List[Callable[[Output], None]] = []
        self._lock = RLock()
        self._reading_stopped = False
        self._reading_thread: Optional[Thread] = None

    def _wait_for_data(self):
        wait_count = 0
        while wait_count < 6:
            if self.serial.in_waiting:
                return True
            else:
                wait_count += 1
                time.sleep(0.2)
        else:
            return False

    def _read_output(self) -> Optional[Output]:
        if not self._wait_for_data():
            return None

        terminator = b'leapMMW:/>'
        buffer = b''
        while True:
            byte = self.serial.read(1)
            # The 0xff character indicates that the read operation has timed out without receiving any data
            if byte == b'\xff':
                self.serial.read(1)  # Read new line
                return None
            buffer += byte

            if buffer.endswith(terminator):
                return None

            if byte == b'\n':
                line = buffer.decode('utf-8').rstrip()
                return Output(line)

    @synchronized
    def status(self):
        """
        Get the current status of the sensor.

        Returns:
            SensorStatus: The status of the sensor.
        """
        is_reading = self._reading_thread is not None
        is_scanning = self._read_output() is not None
        return SensorStatus(self.sensor_id, self.serial.port, self.serial.timeout, is_reading, is_scanning)

    @synchronized
    def clear_buffer(self):
        self.serial.reset_input_buffer()

    @synchronized
    def start_reading(self):
        """
        Start reading sensor data in a separate thread.
        When started, the presence handlers periodically receive the current presence value.
        """
        if self._reading_thread:
            return

        self._reading_thread = Thread(target=self.read)
        self._reading_thread.start()
        log.info(f"[reading_started] sensor=[{self.sensor_id}] thread=[{self._reading_thread}]")

    def stop_reading(self, timeout=None):
        """
        Stop reading sensor data and wait for the reading thread to terminate.
        When stopped, the presence handlers do not periodically receive the current presence value.

        Args:
            timeout (float, optional): The maximum time to wait for the reading thread to terminate.
        """
        with self._lock:
            self._reading_stopped = True
            reading_thread = self._reading_thread
            self._reading_thread = None

        if reading_thread:
            reading_thread.join(timeout)
            log.info(f"[reading_stopped] sensor=[{self.sensor_id}] thread=[{reading_thread}]")

    def read(self):
        """
        Read sensor data continuously until stopped. This is a blocking method.
        Once reading starts, the presence handlers periodically receive the current presence value.
        """
        self._reading_stopped = False

        while not self._reading_stopped:
            with self._lock:
                try:
                    output = self._read_output()
                except (SerialException, TimeoutError):
                    logging.exception(f"Error reading from sensor {self.sensor_id}")
                    time.sleep(2)
                    continue

            if output:
                for handler in self.handlers:
                    handler(output)

            # Sleep for a short duration to reduce the chance of lock starvation.
            time.sleep(0.01)

    @synchronized
    def read_presence(self) -> Optional[bool]:
        """
        Read the presence detection status from the sensor.

        Returns:
            Optional[bool]: True if presence is detected, False if no presence, None if unable to determine.
        """
        output = self._read_output()
        return output.presence if output is not None else None

    @synchronized
    def send_command(self, cmd: Command, *params) -> CommandResponse:
        """
        Send a command to the sensor via serial connection.

        Args:
            cmd (Command): The command to be sent to the sensor.
            *params: Additional parameters for the command.

        Returns:
            CommandResponse: The response received from the sensor.
        """
        cmd_str = cmd.value + (" " if params else "") + " ".join(map(str, params))
        self.clear_buffer()  # Clear the input buffer to remove any stale data
        self.serial.write((cmd_str + '\n').encode('utf-8'))
        self.serial.flush()

        term_tries = 0
        confirmed = False
        outputs = []
        while output := self._read_output():
            if output.output == cmd_str:
                confirmed = True
                outputs.clear()  # Ignore unrelated outputs before the command echo
            outputs.append(output)

            term_tries += 1
            if term_tries > 10:
                log.warning("[no_command_response_terminator]")
                break

        if not confirmed:
            log.warning(f"[command_unconfirmed] sensor=[{self.sensor_id}] command=[{cmd_str}] outputs={outputs}")
            return CommandResponse(None)

        resp = CommandResponse(outputs)
        if resp:
            log.info(f"[command_executed] sensor=[{self.sensor_id}] command=[{cmd_str}] response=[{resp}]")
        else:
            log.warning(f"[command_failed] sensor=[{self.sensor_id}] command=[{cmd_str}] response=[{resp}]")
        return resp

    @synchronized
    def configure(self, cmd: Command, *params) -> ConfigChainResponse:
        """
        Configure the sensor with the given command and parameters.
        The configuration chain consists of the following steps:
            1. Stop command, if the sensor is currently scanning.
            2. Configuration command with the provided parameters.
            3. Save configuration command to persist the changes.
            4. Start command, if the sensor was previously stopped.

        Args:
            cmd (Command): The configuration command.
            *params: Additional parameters for the configuration command.

        Returns:
            ConfigChainResponse: The response to the configuration command chain.

        Raises:
            ValueError: If the command is not a configuration command.
        """
        if not cmd.is_config:
            raise ValueError(f"Command {cmd} is not a configuration command")

        resp = ConfigChainResponse()

        is_scanning = self._read_output() is not None
        if is_scanning:
            resp.pause_cmd = self.stop_scanning()
            if not resp.pause_cmd:
                return resp

        try:
            resp.cfg_cmd = self.send_command(cmd, *params)
            if resp.cfg_cmd:
                resp.save_cmd = self.save_configuration()
        finally:
            if is_scanning:
                resp.resume_cmd = self.start_scanning()

        return resp

    def start_scanning(self) -> CommandResponse:
        """
        Start the sensor scanning.

        Returns:
            CommandResponse: The response to the start scanning command.
        """
        return self.send_command(Command.SENSOR_START)

    def stop_scanning(self) -> CommandResponse:
        """
        Stop the sensor scanning.

        Returns:
            CommandResponse: The response to the stop scanning command.
        """
        return self.send_command(Command.SENSOR_STOP)

    def set_latency(self, detection_delay, disappearance_delay) -> CommandResponse:
        """
        Set the latency configuration of the sensor.

        Args:
            detection_delay (int): The delay time for output of sensing results when a target is detected.
            disappearance_delay (int): The delay time for output of sensing results after the target disappears.

        Returns:
            CommandResponse: The response to the latency configuration command.
        """
        return self.send_command(Command.LATENCY_CONFIG, -1, detection_delay, disappearance_delay)

    def set_detection_range(self, /, seg_a, seg_b=None, seg_c=None, seg_d=None):
        """
        Set the detection range configuration of the sensor.

        Args:
            seg_a (Tuple[int, int]): The first segment of the sensing area configuration.
            seg_b (Tuple[int, int], optional): The second segment of the sensing area configuration.
            seg_c (Tuple[int, int], optional): The third segment of the sensing area configuration.
            seg_d (Tuple[int, int], optional): The fourth segment of the sensing area configuration.

        Returns:
            CommandResponse: The response to the detection range configuration command.
        """
        params = [param for seg in [seg_a, seg_b, seg_c, seg_d] if seg is not None for param in seg]
        range_segments(params)

        return self.send_command(Command.DETECTION_RANGE_CONFIG, *([-1] + params))

    def configure_latency(self, detection_delay, disappearance_delay) -> ConfigChainResponse:
        """
        Configure the latency settings of the sensor.
        The change is saved to the sensor's persistent memory.

        Args:
            detection_delay (int): The delay time for output of sensing results when a target is detected.
            disappearance_delay (int): The delay time for output of sensing results after the target disappears.

        Returns:
            ConfigChainResponse: The response to the latency configuration command chain.
        """
        return self.configure(Command.LATENCY_CONFIG, -1, detection_delay, disappearance_delay)

    def configure_detection_range(self, /, seg_a, seg_b=None, seg_c=None, seg_d=None):
        """
        Configure the detection range settings of the sensor.
        The change is saved to the sensor's persistent memory.

        Args:
            seg_a (Tuple[int, int]): The first segment of the sensing area configuration.
            seg_b (Tuple[int, int], optional): The second segment of the sensing area configuration.
            seg_c (Tuple[int, int], optional): The third segment of the sensing area configuration.
            seg_d (Tuple[int, int], optional): The fourth segment of the sensing area configuration.

        Returns:
            ConfigChainResponse: The response to the detection range configuration command chain.
        """
        params = [param for seg in [seg_a, seg_b, seg_c, seg_d] if seg is not None for param in seg]
        range_segments(params)

        return self.configure(Command.DETECTION_RANGE_CONFIG, *([-1] + params))

    def save_configuration(self):
        """
        Save the sensor configuration.

        Returns:
            CommandResponse: The response to the save configuration command.
        """
        return self.send_command(Command.SAVE_CONFIG, *SAVE_CONFIG_PARAMETERS)

    def close(self):
        """
        Close the sensor connection and stop reading.
        """
        self.stop_reading()
        self.serial.close()


def locked(method):
    """
    A decorator to lock methods for thread-safe operation and asynchronous execution.
    """

    @wraps(method)
    async def wrapper(self, *args, **kwargs):
        async with self._lock:
            return await method(self, *args, **kwargs)

    return wrapper


class PresenceHandlerAsync:
    """
    Handles presence detection events from the sensor.

    Attributes:
        observers (List[Callable[[bool], Union[None, Awaitable[None]]]]): List of observer callbacks to be notified of presence changes.
        presence_value (Optional[bool]): The current presence detection value.
    """

    def __init__(self):
        self.observers: List[Callable[[bool], Union[None, Awaitable[None]]]] = []
        self.presence_value: bool | None = None

    async def __call__(self, output):
        """
        Process the sensor output and notify observers if presence detection changes.

        Args:
            output (Output): The parsed sensor output.
        """
        if output.presence is None:
            return

        if self.presence_value != output.presence:
            self.presence_value = output.presence

            for observer in self.observers:
                result = observer(self.presence_value)
                if isinstance(result, Awaitable):
                    await result


class SensorAsync:
    """
    Represents the SEN0395 sensor.

    Attributes:
        sensor_id (SensorId): The unique identifier of the sensor.
        serial (Serial): The serial connection to the sensor.
        handlers (List[Callable[[Output], None]]): List of output handlers.
    """

    def __init__(self, sensor_name, serial_con):
        """
        Initialize the Sensor object.

        Args:
            sensor_name (str): The name of the sensor.
            serial_con (Serial): The serial connection to the sensor.
        """
        self.sensor_id = SensorId(SensorType.SEN0395, sensor_name)
        self.serial = serial_con
        self.handlers: List[Union[Callable[[Output], None], Callable[[Output], Awaitable[None]]]] = []
        self._lock = asyncio.Lock()
        self._reading_task: Optional[asyncio.Task] = None

    async def _read_output(self) -> Optional[Output]:
        terminator = b'leapMMW:/>'
        buffer = bytearray()

        while True:
            c = await self.serial.read(1)
            if not c:
                return None

            buffer += c

            if buffer[-(len(terminator)):] == terminator:
                return None

            if buffer[-(len(serial.LF)):] == serial.LF:
                line = bytes(buffer).decode('utf-8').rstrip()
                return Output(line)

    @locked
    async def status(self):
        """
        Get the current status of the sensor.

        Returns:
            SensorStatus: The status of the sensor.
        """
        is_reading = self._reading_task is not None
        is_scanning = await self._read_output() is not None
        return SensorStatus(self.sensor_id, self.serial.port, self.serial.timeout, is_reading, is_scanning)

    @locked
    async def clear_buffer(self):
        await self.serial.reset_input_buffer()

    def start_reading(self):
        """
        Start reading sensor data in a separate thread.
        When started, the presence handlers periodically receive the current presence value.
        """
        if self._reading_task:
            return

        self._reading_task = asyncio.create_task(self.read())
        log.info(f"[reading_started] sensor=[{self.sensor_id}] task=[{self._reading_task}]")
        return self._reading_task

    async def read(self):
        """
        Read sensor data continuously until stopped.
        Once reading starts, the presence handlers periodically receive the current presence value.
        """
        while True:
            async with self._lock:
                try:
                    output = await self._read_output()
                except (SerialException, TimeoutError):
                    logging.exception(f"[sensor_error] sensor=[{self.sensor_id}]")
                    await asyncio.sleep(5)
                    continue

            if output:
                for handler in self.handlers:
                    try:
                        await handler(output)
                    except Exception:
                        logging.exception(f"[sensor_handler_error] sensor=[{self.sensor_id}] handler=[{handler}]")

    async def stop_reading(self):
        """
        Stop reading sensor data and wait for the reading thread to terminate.
        When stopped, the presence handlers do not periodically receive the current presence value.
        """
        reading_task = self._reading_task
        self._reading_task = None

        if reading_task:
            try:
                reading_task.cancel()
                await reading_task
            except asyncio.CancelledError:
                pass  # This is expected due to the cancellation
            finally:
                log.info(f"[reading_stopped] sensor=[{self.sensor_id}] task=[{reading_task}]")

    @locked
    async def read_presence(self) -> Optional[bool]:
        """
        Read the presence detection status from the sensor.

        Returns:
            Optional[bool]: True if presence is detected, False if no presence, None if unable to determine.
        """
        output = await self._read_output()
        return output.presence if output is not None else None

    async def _send_command(self, cmd: Command, *params) -> CommandResponse:
        """
        Send a command to the sensor via serial connection.

        Args:
            cmd (Command): The command to be sent to the sensor.
            *params: Additional parameters for the command.

        Returns:
            CommandResponse: The response received from the sensor.
        """
        cmd_str = cmd.value + (" " if params else "") + " ".join(map(str, params))
        await self.serial.reset_input_buffer()  # Clear the input buffer to remove any stale data
        await self.serial.write((cmd_str + '\n').encode('utf-8'))
        await self.serial.flush()

        term_tries = 0
        confirmed = False
        outputs = []
        while output := await self._read_output():
            if output.output == cmd_str:
                confirmed = True
                outputs.clear()  # Ignore unrelated outputs before the command echo
            outputs.append(output)

            term_tries += 1
            if term_tries > 10:
                log.warning("[no_command_response_terminator]")
                break

        if not confirmed:
            log.warning(f"[command_unconfirmed] sensor=[{self.sensor_id}] command=[{cmd_str}] outputs={outputs}")
            return CommandResponse(None)

        resp = CommandResponse(outputs)
        if resp:
            log.info(f"[command_executed] sensor=[{self.sensor_id}] command=[{cmd_str}] response=[{resp}]")
        else:
            log.warning(f"[command_failed] sensor=[{self.sensor_id}] command=[{cmd_str}] response=[{resp}]")
        return resp

    @locked
    async def configure(self, cmd: Command, *params) -> ConfigChainResponse:
        """
        Configure the sensor with the given command and parameters.
        The configuration chain consists of the following steps:
            1. Stop command, if the sensor is currently scanning.
            2. Configuration command with the provided parameters.
            3. Save configuration command to persist the changes.
            4. Start command, if the sensor was previously stopped.

        Args:
            cmd (Command): The configuration command.
            *params: Additional parameters for the configuration command.

        Returns:
            ConfigChainResponse: The response to the configuration command chain.

        Raises:
            ValueError: If the command is not a configuration command.
        """
        if not cmd.is_config:
            raise ValueError(f"Command {cmd} is not a configuration command")

        resp = ConfigChainResponse()

        is_scanning = await self._read_output() is not None
        if is_scanning:
            resp.pause_cmd = await self._send_command(Command.SENSOR_STOP)
            if not resp.pause_cmd:
                return resp

        try:
            resp.cfg_cmd = await self._send_command(cmd, *params)
            if resp.cfg_cmd:
                resp.save_cmd = await self._send_command(Command.SAVE_CONFIG, *SAVE_CONFIG_PARAMETERS)
        finally:
            if is_scanning:
                resp.resume_cmd = await self._send_command(Command.SENSOR_START)

        return resp

    @locked
    async def start_scanning(self) -> CommandResponse:
        """
        Start the sensor scanning.

        Returns:
            CommandResponse: The response to the start scanning command.
        """
        return await self._send_command(Command.SENSOR_START)

    @locked
    async def stop_scanning(self) -> CommandResponse:
        """
        Stop the sensor scanning.

        Returns:
            CommandResponse: The response to the stop scanning command.
        """
        return await self._send_command(Command.SENSOR_STOP)

    @locked
    async def set_latency(self, detection_delay, disappearance_delay) -> CommandResponse:
        """
        Set the latency configuration of the sensor.

        Args:
            detection_delay (int): The delay time for output of sensing results when a target is detected.
            disappearance_delay (int): The delay time for output of sensing results after the target disappears.

        Returns:
            CommandResponse: The response to the latency configuration command.
        """
        return await self._send_command(Command.LATENCY_CONFIG, -1, detection_delay, disappearance_delay)

    @locked
    async def set_detection_range(self, /, seg_a, seg_b=None, seg_c=None, seg_d=None):
        """
        Set the detection range configuration of the sensor.

        Args:
            seg_a (Tuple[int, int]): The first segment of the sensing area configuration.
            seg_b (Tuple[int, int], optional): The second segment of the sensing area configuration.
            seg_c (Tuple[int, int], optional): The third segment of the sensing area configuration.
            seg_d (Tuple[int, int], optional): The fourth segment of the sensing area configuration.

        Returns:
            CommandResponse: The response to the detection range configuration command.
        """
        params = [param for seg in [seg_a, seg_b, seg_c, seg_d] if seg is not None for param in seg]
        range_segments(params)

        return await self._send_command(Command.DETECTION_RANGE_CONFIG, *([-1] + params))

    async def configure_latency(self, detection_delay, disappearance_delay) -> ConfigChainResponse:
        """
        Configure the latency settings of the sensor.
        The change is saved to the sensor's persistent memory.

        Args:
            detection_delay (int): The delay time for output of sensing results when a target is detected.
            disappearance_delay (int): The delay time for output of sensing results after the target disappears.

        Returns:
            ConfigChainResponse: The response to the latency configuration command chain.
        """
        return await self.configure(Command.LATENCY_CONFIG, -1, detection_delay, disappearance_delay)

    async def configure_detection_range(self, /, seg_a, seg_b=None, seg_c=None, seg_d=None):
        """
        Configure the detection range settings of the sensor.
        The change is saved to the sensor's persistent memory.

        Args:
            seg_a (Tuple[int, int]): The first segment of the sensing area configuration.
            seg_b (Tuple[int, int], optional): The second segment of the sensing area configuration.
            seg_c (Tuple[int, int], optional): The third segment of the sensing area configuration.
            seg_d (Tuple[int, int], optional): The fourth segment of the sensing area configuration.

        Returns:
            ConfigChainResponse: The response to the detection range configuration command chain.
        """
        params = [param for seg in [seg_a, seg_b, seg_c, seg_d] if seg is not None for param in seg]
        range_segments(params)

        return await self.configure(Command.DETECTION_RANGE_CONFIG, *([-1] + params))

    @locked
    async def save_configuration(self):
        """
        Save the sensor configuration.

        Returns:
            CommandResponse: The response to the save configuration command.
        """
        return await self._send_command(Command.SAVE_CONFIG, *SAVE_CONFIG_PARAMETERS)

    async def close(self):
        """
        Close the sensor connection and stop reading.
        """
        await self.stop_reading()
        await self.serial.close()

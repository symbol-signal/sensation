import logging
import re
import time
from dataclasses import dataclass
from enum import Enum
from functools import wraps
from threading import RLock, Thread
from typing import List, Callable, Optional, Dict

from sensation.common import SensorId

log = logging.getLogger(__name__)

SAVE_CONFIG_PARAMETERS = ("0x45670123", "0xCDEF89AB", "0x956128C6", "0xDF54AC89")

PRESENCE_PATTERN = r'^\$JYBSS,([01])'


class Command(Enum):
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
        for member in cls:
            if member.value == value:
                return member
        return Command.NONE

    def __bool__(self):
        return self != Command.NONE


class CommandResult(Enum):
    DONE = 'Done'
    NOT_APPLICABLE = 'N/A'
    ERROR = 'Error'
    MISSING = 'Missing'
    UNKNOWN = 'Unknown'

    def __bool__(self):
        return self == CommandResult.DONE or self == CommandResult.NOT_APPLICABLE


def is_present(output) -> Optional[bool]:
    presence_match = re.match(PRESENCE_PATTERN, output)
    if not presence_match:
        return None

    return bool(int(presence_match.group(1)))


def parse_command_result(output) -> CommandResult:
    if not output:
        return CommandResult.MISSING
    try:
        return CommandResult(output)
    except ValueError:
        return CommandResult.UNKNOWN


def parse_command(output) -> Command:
    for c in Command:
        for o in output.split():
            if o == c.value:
                return c

    return Command.NONE


class Output:

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

    def __init__(self, outputs):
        self.outputs = outputs or []

    def serialize(self) -> Dict:
        return {
            "outputs": [output.output for output in self.outputs],
        }

    @classmethod
    def deserialize(cls, data: Dict):
        outputs = [Output(output) for output in data["outputs"]]
        return cls(outputs=outputs)

    def __bool__(self):
        return bool(self.command_result)

    @property
    def command_echo(self) -> Optional[str]:
        if not self.outputs:
            return None

        return self.outputs[0].output

    @property
    def command(self) -> Command:
        if not self.outputs:
            return Command.NONE

        return self.outputs[0].command

    @property
    def message(self) -> Optional[str]:
        if len(self.outputs) < 3:
            return None

        return self.outputs[-2].message

    @property
    def command_result(self) -> CommandResult:
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
    pause_cmd: Optional[CommandResponse] = None
    cfg_cmd: Optional[CommandResponse] = None
    save_cmd: Optional[CommandResponse] = None
    resume_cmd: Optional[CommandResponse] = None

    def serialize(self) -> Dict:
        return {
            "pause_cmd": self.pause_cmd.serialize() if self.pause_cmd else None,
            "cfg_cmd": self.cfg_cmd.serialize() if self.cfg_cmd else None,
            "save_cmd": self.save_cmd.serialize() if self.save_cmd else None,
            "resume_cmd": self.resume_cmd.serialize() if self.resume_cmd else None,
        }

    @classmethod
    def deserialize(cls, data: Dict) -> 'ConfigChainResponse':
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

    def __init__(self):
        self.observers: List[Callable[[bool], None]] = []
        self.presence_value: bool | None = None

    def __call__(self, output):
        if output.presence is None:
            return

        if self.presence_value != output.presence:
            self.presence_value = output.presence

            for observer in self.observers:
                observer(self.presence_value)


@dataclass
class SensorStatus:
    sensor_id: SensorId
    port: str
    timeout: Optional[int]
    is_reading: bool
    is_scanning: bool

    @classmethod
    def deserialize(cls, as_dict):
        return cls(
            SensorId.deserialize(as_dict["sensor_id"]),
            as_dict["port"],
            as_dict["timeout"],
            as_dict["is_reading"],
            as_dict["is_scanning"],
        )

    def serialize(self):
        return {
            "sensor_id": self.sensor_id.serialize(),
            "port": self.port,
            "timeout": self.timeout,
            "is_reading": self.is_reading,
            "is_scanning": self.is_scanning
        }


def synchronized(method):
    """
    A decorator to lock methods for thread-safe operation.
    """

    @wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)

    return wrapper


def range_segments(params):
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

    def __init__(self, sensor_id, serial_con):
        self.sensor_id = sensor_id
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
        is_reading = self._reading_thread is not None
        is_scanning = self._read_output() is not None
        return SensorStatus(self.sensor_id, self.serial.port, self.serial.timeout, is_reading, is_scanning)

    @synchronized
    def start_reading(self):
        if self._reading_thread:
            return

        self._reading_thread = Thread(target=self.read)
        self._reading_thread.start()
        log.info(f"[reading_started] sensor=[{self.sensor_id}] thread=[{self._reading_thread}]")

    def stop_reading(self, timeout=None):
        with self._lock:
            self._reading_stopped = True
            reading_thread = self._reading_thread
            self._reading_thread = None

        if reading_thread:
            reading_thread.join(timeout)
            log.info(f"[reading_stopped] sensor=[{self.sensor_id}] thread=[{reading_thread}]")

    def read(self):
        self._reading_stopped = False

        while not self._reading_stopped:
            with self._lock:
                output = self._read_output()
            if output:
                for handler in self.handlers:
                    handler(output)

            # Sleep for a short duration to reduce the chance of lock starvation.
            time.sleep(0.01)

    @synchronized
    def read_presence(self):
        output = self._read_output()
        return output.presence if output is not None else None

    @synchronized
    def send_command(self, cmd: Command, *params) -> CommandResponse:
        """
        Sends a command to the device via serial connection.

        :param cmd: The command string to be sent to the device.
        """
        cmd_str = cmd.value + (" " if params else "") + " ".join(map(str, params))
        self.serial.reset_input_buffer()  # Clear the input buffer to remove any stale data
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
        return self.send_command(Command.SENSOR_START)

    def stop_scanning(self) -> CommandResponse:
        return self.send_command(Command.SENSOR_STOP)

    def set_latency(self, detection_delay, disappearance_delay) -> CommandResponse:
        return self.send_command(Command.LATENCY_CONFIG, detection_delay, disappearance_delay)

    def set_detection_range(self, /, seg_a, seg_b=None, seg_c=None, seg_d=None):
        params = [param for seg in [seg_a, seg_b, seg_c, seg_d] if seg is not None for param in seg]
        range_segments(params)

        return self.send_command(Command.DETECTION_RANGE_CONFIG, *([-1] + params))

    def configure_latency(self, detection_delay, disappearance_delay) -> ConfigChainResponse:
        return self.configure(Command.LATENCY_CONFIG, -1, detection_delay, disappearance_delay)

    def configure_detection_range(self, /, seg_a, seg_b=None, seg_c=None, seg_d=None):
        params = [param for seg in [seg_a, seg_b, seg_c, seg_d] if seg is not None for param in seg]
        range_segments(params)

        return self.configure(Command.DETECTION_RANGE_CONFIG, *([-1] + params))

    def save_configuration(self):
        return self.send_command(Command.SAVE_CONFIG, *SAVE_CONFIG_PARAMETERS)

    def close(self):
        self.stop_reading()

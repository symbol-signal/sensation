import asyncio
import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Callable, Awaitable, Union

from serial.serialutil import SerialException

from sensation.common import SensorId, SensorType

log = logging.getLogger(__name__)


class SensorStatus(Enum):
    OK = auto()
    ERR_CHECKSUM = auto()
    ERR_SERIAL = auto()
    ERR_OUT_OF_RANGE = auto()
    ERR_NO_DATA = auto()


@dataclass
class MeasurementResult:
    distance: int
    status: SensorStatus
    raw_data: List[int]


class PresenceHandlerAsync:
    """
    Converts distance measurement into a presence value and notifies listeners on the value change.

    Attributes:
        observers (List[Callable[[bool], Union[None, Awaitable[None]]]]): List of observer callbacks to be notified of presence changes.
        presence_value (Optional[bool]): The current presence detection value.
        presence_threshold (int): Distance threshold below which presence is detected.
        absence_threshold (int): Distance threshold above which absence is detected.
        hysteresis_count (int): Number of consecutive readings required to change presence state.
    """

    def __init__(self, presence_threshold: int, absence_threshold: int, hysteresis_count: int = 1):
        self.observers: List[Callable[[bool], Union[None, Awaitable[None]]]] = []
        self.presence_value: bool | None = None
        self.presence_threshold = presence_threshold
        self.absence_threshold = absence_threshold
        self.hysteresis_count = hysteresis_count
        self._consecutive_count = 0
        self._last_state = None

    async def __call__(self, measurement: MeasurementResult):
        """
        Process the sensor measurement, convert to presence and notify observers if presence detection changes.

        Args:
            measurement (MeasurementResult): The parsed sensor measurement.
        """
        if measurement.status is not SensorStatus.OK:
            return

        current_state = self._determine_presence(measurement.distance)

        if current_state == self._last_state:
            self._consecutive_count += 1
        else:
            self._consecutive_count = 1

        self._last_state = current_state

        if self._consecutive_count >= self.hysteresis_count:
            if self.presence_value != current_state:
                self.presence_value = current_state
                await self._notify_observers()

    def _determine_presence(self, distance: int) -> bool:
        if distance >= self.absence_threshold:
            return False

        if distance >= self.presence_threshold:
            return True

        return self.presence_value if self.presence_value is not None else False

    async def _notify_observers(self):
        for observer in self.observers:
            try:
                result = observer(self.presence_value)
                if isinstance(result, Awaitable):
                    await result
            except Exception as e:
                log.exception(f"[presence_observer_error] {observer}: {e}")

class SensorAsync:
    """
    Async implementation of DFRobot A02YYUW ultrasonic distance measurement sensor (SEN0311).
    """
    def __init__(self, sensor_name, serial_con, distance_min: int = 0, distance_max: int = 4500):
        self.sensor_id = SensorId(SensorType.SEN0311, sensor_name)
        self.distance_min = distance_min
        self.distance_max = distance_max
        self._ser = serial_con
        self._lock = asyncio.Lock()
        self._reading_task: Optional[asyncio.Task] = None
        self.handlers: List[Callable[[MeasurementResult], Awaitable[None]]] = []

    async def measure(self) -> MeasurementResult:
        raw_data = await self._read_data()
        if not raw_data:
            return MeasurementResult(0, SensorStatus.ERR_NO_DATA, [])

        checksum = sum(raw_data[:3]) & 0xFF
        if checksum != raw_data[3]:
            return MeasurementResult(0, SensorStatus.ERR_CHECKSUM, raw_data)

        distance = (raw_data[1] << 8) + raw_data[2]

        if distance < self.distance_min or distance > self.distance_max:
            return MeasurementResult(distance, SensorStatus.ERR_OUT_OF_RANGE, raw_data)

        return MeasurementResult(distance, SensorStatus.OK, raw_data)

    async def _read_data(self) -> Optional[List[int]]:
        async with self._lock:
            try:
                # Set a timeout for waiting for data
                start_time = asyncio.get_event_loop().time()
                timeout = 1  # 1-second timeout, adjust as needed

                while self._ser.in_waiting < 4:  # Wait until at least 4 bytes are available or timeout occurs
                    await asyncio.sleep(0.01)  # Sleep briefly to yield control
                    if (asyncio.get_event_loop().time() - start_time) > timeout:
                        return None  # Timeout occurred, no data available

                data_bytes = await self._ser.read(self._ser.in_waiting)  # Read all data in the buffer
                if not data_bytes:
                    return None

                data_list = list(data_bytes)
                index = len(data_list) - 4
                while index >= 0:  # Search for valid frame starting with the latest data
                    if data_list[index] == 0xFF:
                        potential_frame = data_list[index:index + 4]
                        if len(potential_frame) == 4:
                            checksum = sum(potential_frame[:3]) & 0xFF
                            if checksum == potential_frame[3]:
                                return potential_frame  # Valid frame found
                    index -= 1
                return None  # No valid frame found
            except asyncio.TimeoutError:
                return None

    async def read(self, sleep_interval=None):
        """
        Read sensor data continuously until stopped.
        Once reading starts, the measurement handlers periodically receive updates.
        """
        retry_delay = 1
        while True:
            try:
                measurement = await self.measure()
                retry_delay = 1
            except SerialException:
                log.exception(f"[sensor_error] sensor=[{self.sensor_id}]")
                await asyncio.sleep(retry_delay)
                retry_delay = min(retry_delay * 2, 60)  # Cap the delay to 60 seconds
                continue

            for handler in self.handlers:
                try:
                    await handler(measurement)
                except Exception:
                    log.exception(f"[sensor_handler_error] sensor=[{self.sensor_id}] handler=[{handler}]")

            if sleep_interval:
                await asyncio.sleep(sleep_interval)


    def start_reading(self, sleep_interval=None) -> Optional[asyncio.Task]:
        """
        Start reading sensor data in a separate asyncio task.
        When started, the handlers periodically receive measurement updates.

        Returns:
            Optional[asyncio.Task]: The created task if a new one was started, None if a task was already running.
        """
        if self._reading_task:
            return

        self._reading_task = asyncio.create_task(self.read(sleep_interval))
        log.info(f"[reading_started] sensor=[{self.sensor_id}] task=[{self._reading_task}]")
        return self._reading_task

    async def stop_reading(self):
        """
        Stop reading sensor data and wait for the reading task to terminate.
        When stopped, the handlers do not periodically receive any updates.
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

    async def close(self):
        """
        Close the sensor connection and stop reading.
        """
        await self.stop_reading()
        await self._ser.close()

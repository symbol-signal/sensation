import asyncio
import logging
from asyncio import Task
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Callable, Awaitable, Union

from serial.serialutil import SerialException

from sensation.common import SensorId, SensorType

log = logging.getLogger(__name__)


class SensorState(Enum):
    OK = auto()
    ERR_CHECKSUM = auto()
    ERR_SERIAL = auto()
    ERR_OUT_OF_RANGE = auto()
    ERR_NO_DATA = auto()


@dataclass
class MeasurementResult:
    distance: int
    state: SensorState
    raw_data: List[int]

    def serialize(self):
        """
        Serialize the MeasurementResult object to a dictionary.

        Returns:
            Dict: The serialized representation of the MeasurementResult.
        """
        return {
            "distance": self.distance,
            "state": self.state.name,
            "raw_data": self.raw_data,
        }

    @classmethod
    def deserialize(cls, as_dict):
        """
        Deserialize a dictionary to a MeasurementResult object.

        Args:
            as_dict (Dict): The serialized data.

        Returns:
            MeasurementResult: The deserialized MeasurementResult object.
        """
        return cls(
            distance=as_dict["distance"],
            state=SensorState[as_dict["state"]],
            raw_data=as_dict["raw_data"],
        )


@dataclass
class SensorStatus:
    """
    Represents the status of the sensor.

    Attributes:
        sensor_id (SensorId): The unique identifier of the sensor.
        port (str): The serial port to which the sensor is connected.
        is_reading (bool): Indicates whether this instance is currently reading the sensor data (event notification).
        measurement (MeasurementResult): Measurement recorded during the get status operation.
    """
    sensor_id: SensorId
    port: str
    is_reading: bool
    measurement: MeasurementResult

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
            as_dict["is_reading"],
            MeasurementResult.deserialize(as_dict["measurement"]),
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
            "is_reading": self.is_reading,
            "measurement": self.measurement.serialize(),
        }


class PresenceHandlerAsync:
    """
    Converts distance measurement into a presence value and notifies listeners on the value change.

    Attributes:
        observers (List[Callable[[bool], Union[None, Awaitable[None]]]]): List of observer callbacks to be notified of presence changes.
        presence_value (Optional[bool]): The current presence detection value.
        presence_threshold (int): Distance threshold below which presence is detected.
        absence_threshold (int): Distance threshold above which absence is detected.
        hysteresis_count (int, optional): Number of consecutive readings required to change presence state. Defaults to 1.
        delay_presence (float, optional): Delay in seconds before confirming a presence detection. Defaults to 0.0.
        delay_absence (float, optional): Delay in seconds before confirming an absence detection. Defaults to 0.0.
    """

    def __init__(self, *,
                 threshold_presence: int, threshold_absence: int,
                 hysteresis_count = 1,
                 delay_presence = 0.0, delay_absence = 0.0):
        self.observers: List[Callable[[bool], Union[None, Awaitable[None]]]] = []
        self.presence_value: bool | None = None
        self.presence_threshold = threshold_presence
        self.absence_threshold = threshold_absence
        self.hysteresis_count = hysteresis_count
        self.delay_presence = delay_presence
        self.delay_absence = delay_absence
        self._consecutive_count = 0
        self._last_presence = None
        self._change_task: Optional['PresenceHandlerAsync.ChangePresenceTask'] = None

    class ChangePresenceTask:

        def __init__(self, handler: 'PresenceHandlerAsync', new_value):
            self.handler = handler
            self.new_value = new_value
            self.task_instance: Optional[Task] = None
            self.executed = False

        def resolve_delay(self):
            if self.new_value:
                return self.handler.delay_presence

            return self.handler.delay_absence

        def __call__(self):
            self.task_instance = asyncio.create_task(self.run())

        async def run(self):
            if delay := self.resolve_delay():
                await asyncio.sleep(delay)

            self.executed = True
            # Cannot be cancelled after this point to prevent observers in inconsistent state

            await self.handler._change_presence(self.new_value)

            if self.handler._change_task == self:
                self.handler._change_task = None

        def cancel(self):
            if self.task_instance and not self.executed:
                self.task_instance.cancel()

    async def __call__(self, measurement: MeasurementResult):
        """
        Process the sensor measurement, convert to presence and notify observers if presence detection changes.

        Args:
            measurement (MeasurementResult): The parsed sensor measurement.
        """
        if measurement.state is not SensorState.OK:
            return

        new_presence = self._determine_presence(measurement.distance)

        if new_presence == self._last_presence:
            self._consecutive_count += 1
        else:
            self._consecutive_count = 1

        self._last_presence = new_presence

        if self._consecutive_count >= self.hysteresis_count:
            if self._change_task:
                if self._change_task.new_value == new_presence:
                    return
                self._change_task.cancel()
            elif self.presence_value == new_presence:
                return

            self._change_task = self.ChangePresenceTask(self, new_presence)
            self._change_task()

    def _determine_presence(self, distance: int) -> bool:
        if distance >= self.absence_threshold:
            return False

        if distance >= self.presence_threshold:
            return True

        return self.presence_value if self.presence_value is not None else False

    async def _change_presence(self, new_presence):
        if self.presence_value != new_presence:
            self.presence_value = new_presence
            await self._notify_observers()

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
            return MeasurementResult(0, SensorState.ERR_NO_DATA, [])

        checksum = sum(raw_data[:3]) & 0xFF
        if checksum != raw_data[3]:
            return MeasurementResult(0, SensorState.ERR_CHECKSUM, raw_data)

        distance = (raw_data[1] << 8) + raw_data[2]

        if distance < self.distance_min or distance > self.distance_max:
            return MeasurementResult(distance, SensorState.ERR_OUT_OF_RANGE, raw_data)

        return MeasurementResult(distance, SensorState.OK, raw_data)

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

    async def status(self):
        is_reading = self._reading_task is not None
        return SensorStatus(self.sensor_id, self._ser.port, is_reading, await self.measure())

    async def close(self):
        """
        Close the sensor connection and stop reading.
        """
        await self.stop_reading()
        await self._ser.close()

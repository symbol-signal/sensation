import asyncio
import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Callable, Awaitable

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
                while True:
                    if await self._ser.read(1) == b'\xFF':
                        data = await self._ser.read(3)
                        if len(data) == 3:
                            return [0xFF] + list(data)
            except asyncio.TimeoutError:
                return None

    async def read(self, interval=1.0):
        """
        Read sensor data continuously until stopped.
        Once reading starts, the measurement handlers periodically receive updates.
        """
        while True:
            try:
                measurement = await self.measure()
            except SerialException:
                log.exception(f"[sensor_error] sensor=[{self.sensor_id}]")
                await asyncio.sleep(5)
                continue

            if measurement:
                for handler in self.handlers:
                    try:
                        await handler(measurement)
                    except Exception:
                        log.exception(f"[sensor_handler_error] sensor=[{self.sensor_id}] handler=[{handler}]")

            await asyncio.sleep(interval)


    def start_reading(self) -> Optional[asyncio.Task]:
        """
        Start reading sensor data in a separate asyncio task.
        When started, the handlers periodically receive measurement updates.

        Returns:
            Optional[asyncio.Task]: The created task if a new one was started, None if a task was already running.
        """
        if self._reading_task:
            return

        self._reading_task = asyncio.create_task(self.read())
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

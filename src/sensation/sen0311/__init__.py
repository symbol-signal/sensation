import asyncio
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional


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
    def __init__(self, serial_con, distance_min: int = 0, distance_max: int = 4500):
        self._ser = serial_con
        self.distance_min = distance_min
        self.distance_max = distance_max

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
        try:
            while True:
                if await self._ser.read(1) == b'\xFF':
                    data = await self._ser.read(3)
                    if len(data) == 3:
                        return [0xFF] + list(data)
        except asyncio.TimeoutError:
            return None

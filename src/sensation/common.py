from dataclasses import dataclass
from enum import Enum


class SensorType(Enum):

    SEN0311 = 'sen0311'
    SEN0395 = 'sen0395'
    UNKNOWN = 'unknown'

    @classmethod
    def from_value(cls, value):
        for member in cls:
            if member.value == value:
                return member
        return SensorType.UNKNOWN


class SensationException(Exception):
    pass


@dataclass
class SensorId:
    sensor_type: SensorType
    sensor_name: str

    @classmethod
    def deserialize(cls, as_dict):
        return cls(SensorType.from_value(as_dict["type"]), as_dict["name"])

    def serialize(self):
        return {"type": self.sensor_type.value, "name": self.sensor_name}

    def __rich__(self):
        return f"Sensor: [bold]{self.sensor_type.value}[/bold]/[bold blue]{self.sensor_name}[/bold blue]"

    def __str__(self):
        return f"{self.sensor_type.value}/{self.sensor_name}"

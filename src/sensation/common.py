from dataclasses import dataclass


class SensationException(Exception):
    pass


@dataclass
class SensorId:
    sensor_type: str
    sensor_name: str

    @classmethod
    def deserialize(cls, as_dict):
        return cls(as_dict["type"], as_dict["name"])

    def serialize(self):
        return {"type": self.sensor_type, "name": self.sensor_name}

    def __rich__(self):
        return f"Sensor: [bold]{self.sensor_type}[/bold]/[bold blue]{self.sensor_name}[/bold blue]"

    def __str__(self):
        return f"{self.sensor_type}/{self.sensor_name}"

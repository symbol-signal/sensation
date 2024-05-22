# Sensation
This is a Python library for communicating with IoT and home automation sensors and low-level devices.

## Sensors
### SEN0395

The SEN0395 is a millimeter-wave radar motion sensor capable of detecting human presence up to 9 meters away.

| <!-- -->          | <!-- -->                                                          |
|-------------------|-------------------------------------------------------------------|
| Code              | SEN0395                                                           |
| Brand             | DFRobot                                                           |
| Type              | Millimeter-wave radar motion sensor                               |
| Name              | mmWave Radar - 24GHz Human Presence Detection Sensor (9 Meters)   |
| Alternative Names | LeapMMW HS2xx3A series                                            |
| Python Package    | [sensation.sen0395](src/sensation/sen0395/__init__.py)            |
| Product URL       | [DFRobot mmWave Radar](https://www.dfrobot.com/product-2282.html) |

For more information, refer to the [SEN0395 documentation](https://wiki.dfrobot.com/mmWave_Radar_Human_Presence_Detection_SKU_SEN0395).

#### Basic Usage Example
```pycon
>>> from serial import Serial
>>> from sensation.sen0395 import *
>>> sensor = Sensor("sensor_name",  Serial('/dev/ttyAMA0', 115200, timeout=1))
>>> sensor.status()
SensorStatus(sensor_id=SensorId(sensor_type=<SensorType.SEN0395: 'sen0395'>, sensor_name='sensor_name'), port='/dev/ttyAMA0', timeout=1, is_reading=False, is_scanning=False)
>>> sensor.set_latency(15, 35)
CommandResponse(outputs=[outputLatency -1 15 35, Done])
>>> sensor.save_configuration()
CommandResponse(outputs=[saveCfg 0x45670123 0xCDEF89AB 0x956128C6 0xDF54AC89, save cfg complete, Done])
>>> sensor.start_scanning()
CommandResponse(outputs=[sensorStart, Done])
>>> sensor.status()
SensorStatus(sensor_id=SensorId(sensor_type=<SensorType.SEN0395: 'sen0395'>, sensor_name='sensor_name'), port='/dev/ttyAMA0', timeout=1, is_reading=False, is_scanning=True)
```

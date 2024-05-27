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

For more information about the product, refer to the [SEN0395 documentation](https://wiki.dfrobot.com/mmWave_Radar_Human_Presence_Detection_SKU_SEN0395).

#### Common Usage Examples
##### Sensor Instance (common code for the following examples)
```python
from serial import Serial
from sensation.sen0395 import *

sensor = Sensor("sensor_name",  Serial('/dev/ttyAMA0', 115200, timeout=1))

sensor.status()  # Not needed, just checking that the sensor works well
# SensorStatus(sensor_id=SensorId(sensor_type=<SensorType.SEN0395: 'sen0395'>, sensor_name='sensor_name'), port='/dev/ttyAMA0', timeout=1, is_reading=False, is_scanning=False)

# >> DO YOUR OWN THINGS HERE <<

sensor.close()  # This closes also the `Serial` instance
```

##### Presence Reading
```python
sensor.read_presence()  # Returns `None` if sensor is not scanning

sensor.start_scanning()
# CommandResponse(outputs=[sensorStart, Done])

sensor.read_presence()  # Returns `False` on no presence
# False

sensor.read_presence()  # Returns `True` on presence
# True
```

##### Presence Observer
```python
handler = PresenceHandler()
handler.observers.append(lambda presence: print(f"[presence_change] presence=[{presence}]"))
sensor.handlers.append(handler)

# sensor.clear_buffer() # You may want to clear the buffer first, if the connection has been opened for a while
sensor.start_reading()  # This starts a new thread, alternatively you can run the blocking `read()` method by yourself
# [presence_change] presence=[False]
# [presence_change] presence=[True]
sensor.stop_reading()
```

##### Managed Sensor Configuration
```python
sensor.configure_latency(10, 60)
# ConfigChainResponse(pause_cmd=None, cfg_cmd=CommandResponse(outputs=[outputLatency -1 10 60, Done]), save_cmd=CommandResponse(outputs=[saveCfg 0x45670123 0xCDEF89AB 0x956 128C6 0xDF54AC89, save cfg complete, Done]), resume_cmd=None)
```

##### Manual Sensor Configuration
```python
sensor.stop_scanning()
#CommandResponse(outputs=[sensorStop, Done])

sensor.set_latency(15, 35)
# CommandResponse(outputs=[outputLatency -1 15 35, Done])

sensor.save_configuration()
# CommandResponse(outputs=[saveCfg 0x45670123 0xCDEF89AB 0x956128C6 0xDF54AC89, save cfg complete, Done])

sensor.start_scanning()
# CommandResponse(outputs=[sensorStart, Done])
#SensorStatus(sensor_id=SensorId(sensor_type=<SensorType.SEN0395: 'sen0395'>, sensor_name='sensor_name'), port='/dev/ttyAMA0', timeout=1, is_reading=False, is_scanning=True)
```

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

#### Commands not implemented yet
- resetCfg
- setInhibit/getInhibit
- setUart/getUart
- setGpioMode/getGpioMode
- setEcho/getEcho (disabling echo would likely break the current implementation)
- setUartOutput/getUartOutput
- getSWV/getHWV

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
##### Sensor Instance Async
```python
import serialio
from sensation.sen0395 import *

async def example():
    serial_con = serialio.serial_for_url("serial:///dev/ttyAMA1", 115200)
    sensor = SensorAsync("my_sensor", serial_con)
    await serial_con.open()
    
    await sensor.status()  # Not needed, just checking that the sensor works well
    
    # >> DO YOUR OWN THINGS HERE <<
    
    await sensor.close()  # This closes also the serial connection instance
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

##### Presence Reading Async
```python
await sensor.start_scanning()
await sensor.read_presence()
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

##### Presence Observer Async
```python
handler = PresenceHandlerAsync()
# Supports both sync...
handler.observers.append(lambda presence: print(f"[presence_change] presence=[{presence}]"))
# ...and async observers
async def async_observer(presence):
    print(f"[presence_change] presence=[{presence}]")
handler.observers.append(async_observer)

sensor.handlers.append(handler)

# await sensor.clear_buffer() # You may want to clear the buffer first, if the connection has been opened for a while
sensor.start_reading()  # This starts a new async task
await sensor.stop_reading()  # Wait for the reading task to complete
```

##### Managed Sensor Configuration
```python
sensor.configure_latency(10, 60)
# ConfigChainResponse(pause_cmd=None, cfg_cmd=CommandResponse(outputs=[outputLatency -1 10 60, Done]), save_cmd=CommandResponse(outputs=[saveCfg 0x45670123 0xCDEF89AB 0x956 128C6 0xDF54AC89, save cfg complete, Done]), resume_cmd=None)
```
##### Managed Sensor Configuration Async
```python
await sensor_async.configure_latency(10, 60)
```

##### Manual Sensor Configuration
**Note:** *All below methods are async in the async sensor*
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

### SEN0311

The SEN0311 is an ultrasonic distance measurement sensor that can be used to detect presence by measuring the distance to objects.

| <!-- -->          | <!-- -->                                                          |
|-------------------|-------------------------------------------------------------------|
| Code              | SEN0311                                                           |
| Brand             | DFRobot                                                           |
| Type              | Ultrasonic distance measurement sensor                            |
| Name              | A02YYUW Waterproof Ultrasonic Sensor                             |
| Python Package    | [sensation.sen0311](src/sensation/sen0311/__init__.py)            |
| Product URL       | [DFRobot Ultrasonic Sensor](https://www.dfrobot.com/product-1935.html) |

#### Common Usage Examples
##### Sensor Instance (Async)
```python
import serialio
from sensation.sen0311 import SensorAsync

async def example():
    serial_con = serialio.serial_for_url("serial:///dev/ttyAMA1", 9600)
    sensor = SensorAsync("my_sensor", serial_con, distance_min=0, distance_max=4500)
    await serial_con.open()
    
    status = await sensor.status()  # Check if the sensor is working properly
    
    # >> DO YOUR OWN THINGS HERE <<
    
    await sensor.close()  # This closes the serial connection instance
```

##### Distance Measurement
```python
# Perform a single measurement
measurement = await sensor.measure()
print(f"Distance: {measurement.distance}mm, State: {measurement.state}")

# Start continuous measurement with 0.5 second interval
sensor.start_reading(sleep_interval=0.5)

# Stop continuous measurement
await sensor.stop_reading()
```

##### Presence Detection with Thresholds
```python
# Create a presence handler with custom thresholds
handler = PresenceHandlerAsync(
    threshold_presence=1000,   # Consider present when distance < 1000mm
    threshold_absence=1500,    # Consider absent when distance > 1500mm
    hysteresis_count=3,        # Require 3 consecutive readings to change state
    delay_presence=0.0,        # No delay when detecting presence
    delay_absence=5.0,         # 5 second delay before confirming absence
)

# Add observer to handle presence changes (supports both sync and async callbacks)
handler.observers.append(lambda presence: print(f"Presence changed: {presence}"))

# Add async observer
async def async_observer(presence):
    print(f"Async presence notification: {presence}")
    # Do async operations here
    
handler.observers.append(async_observer)

# Add the handler to the sensor
sensor.handlers.append(handler)

# Start reading with 0.2 second interval
sensor.start_reading(sleep_interval=0.2)
```

The SEN0311 sensor can detect distances from a few centimeters to about 4.5 meters, making it suitable 
for various presence detection applications where ultrasonic technology is appropriate, 
such as detecting people or objects in enclosed spaces.
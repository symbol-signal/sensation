# Remote I2C Bridge

This implementation extends the US2N bridge to support I2C devices alongside UART devices, allowing you to access I2C sensors and peripherals over TCP/IP.

## Features

- **Dual Bridge Support**: Run UART and I2C bridges simultaneously on different ports
- **SMBus2 Compatibility**: Full compatibility with Python's smbus2 library
- **CircuitPython Compatibility**: Drop-in replacement for `board.I2C()` 
- **Adafruit Library Support**: Works with existing Adafruit CircuitPython libraries
- **Binary Protocol**: Efficient binary protocol for low latency
- **Authentication**: Optional password protection
- **SSL/TLS**: Secure connections supported

## Files

### Core Implementation
- `i2c_bridge.py` - MicroPython I2C bridge for ESP32
- `REMOTE_I2C_PROTOCOL.md` - Protocol specification
- `us2n-example-dual.json` - Example configuration

### Python Client Libraries
- `remote_smbus2.py` - SMBus2-compatible interface
- `remote_board.py` - CircuitPython board.I2C() compatible interface

### Examples and Tests
- `test_remote_i2c.py` - Comprehensive test script
- `example_adafruit_sensors.py` - Adafruit library examples

## Quick Start

### 1. Configure ESP32

Use the example configuration to enable both UART and I2C bridges:

```json
{
  "name": "ESP32-DualBridge",
  "bridges": [
    {
      "type": "uart",
      "tcp": {"bind": ":20202"},
      "uart": {"port": 2, "baudrate": 115200, "tx": 17, "rx": 16}
    },
    {
      "type": "i2c", 
      "tcp": {"bind": ":20203"},
      "i2c": {"sda": 21, "scl": 22, "freq": 400000}
    }
  ]
}
```

### 2. Using SMBus2 Interface

```python
from remote_smbus2 import RemoteSMBus

# Connect to ESP32 I2C bridge
with RemoteSMBus("192.168.1.100", 20203) as bus:
    # Scan for devices
    devices = bus.scan()
    print(f"Found devices: {[hex(d) for d in devices]}")
    
    # Read temperature sensor (LM75)
    temp = bus.read_word_data(0x48, 0x00)
    
    # Write to I/O expander
    bus.write_byte_data(0x20, 0x00, 0xFF)
```

### 3. Using CircuitPython/Adafruit Libraries

```python
import remote_board as board

# Configure remote connection
board.set_host("192.168.1.100", 20203)

# Use exactly like CircuitPython
i2c = board.I2C()

# Works with Adafruit libraries
import adafruit_bme280.basic as adafruit_bme280
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)

print(f"Temperature: {bme280.temperature:.1f}°C")
```

## Supported Operations

### SMBus2 Methods
- `read_byte(addr)` - Read single byte
- `write_byte(addr, value)` - Write single byte
- `read_byte_data(addr, register)` - Read from register
- `write_byte_data(addr, register, value)` - Write to register
- `read_word_data(addr, register)` - Read 16-bit word
- `write_word_data(addr, register, value)` - Write 16-bit word
- `read_block_data(addr, register)` - Read SMBus block
- `write_block_data(addr, register, data)` - Write SMBus block
- `read_i2c_block_data(addr, register, length)` - Read I2C block
- `write_i2c_block_data(addr, register, data)` - Write I2C block

### Additional Methods
- `scan()` - Scan for I2C devices
- `set_speed(freq_hz)` - Set I2C bus speed
- `get_info()` - Get bridge information

### CircuitPython Methods
- `scan()` - Scan for devices
- `readfrom_into(addr, buffer)` - Read into buffer
- `writeto(addr, buffer)` - Write from buffer  
- `writeto_then_readfrom(addr, out_buf, in_buf)` - Write then read
- `try_lock()` / `unlock()` - Bus locking

## Protocol Details

The bridge uses a binary protocol for efficiency:

**Request Format:** `[CMD:1][ADDR:1][REG:1][LEN:2][DATA:n]`
**Response Format:** `[STATUS:1][LEN:2][DATA:n]`

See `REMOTE_I2C_PROTOCOL.md` for complete specification.

## Hardware Configuration

### ESP32 I2C Pins
- **SDA**: GPIO 21 (default, configurable)
- **SCL**: GPIO 22 (default, configurable)
- **Speed**: 400kHz (default, configurable)

### Common I2C Devices
| Device | Address | Description |
|--------|---------|-------------|
| 0x48   | LM75/TMP75 | Temperature sensor |
| 0x50   | 24C02 | EEPROM |
| 0x20   | MCP23017 | I/O expander |
| 0x3C   | SSD1306 | OLED display |
| 0x76   | BME280 | Environmental sensor |

## Troubleshooting

### Connection Issues
- Verify ESP32 IP address and port
- Check network connectivity
- Ensure I2C bridge is configured in us2n.json

### I2C Issues  
- Check device addresses with `scan()`
- Verify pull-up resistors (4.7kΩ on SDA/SCL)
- Check power supply to I2C devices
- Verify wiring connections

### Library Compatibility
- Most Adafruit CircuitPython libraries work unchanged
- Some libraries may require the `busio` import style
- Use context managers (`with` statements) for proper locking

## Performance Notes

- Binary protocol minimizes overhead
- Single TCP connection per client
- Command pipelining not supported (request/response only)
- Maximum block size: 32 bytes
- Typical latency: 10-50ms over LAN

## Security

- Optional password authentication
- SSL/TLS encryption support
- No built-in access control (use network-level security)
- Consider VPN for remote access

## Future Enhancements

- Raw I2C transaction support for `i2c_rdwr()`
- Command pipelining for improved performance  
- Multi-master bus arbitration
- I2C bus monitoring and debugging tools
- WebSocket alternative transport
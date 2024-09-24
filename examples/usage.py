import asyncio

import serialio

from sensation.sen0395 import SensorAsync


async def main():
    serial_con = serialio.serial_for_url('/dev/serial0', 115200, timeout=1)
    serial_con.host = 'fake'
    s = SensorAsync('test_sensor', serial_con)
    await serial_con.open()
    print(await s.read_presence())
    await serial_con.close()

if __name__ == "__main__":
    asyncio.run(main())

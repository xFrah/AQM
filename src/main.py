import time
import json
from machine import I2C, Pin
import lib.scd4x as scd4x
from lib.pms7003 import Pms7003


def main():
    i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)

    # Initialize SCD4X sensor
    scd = scd4x.SCD4X(i2c)
    print("Serial number:", [hex(i) for i in scd.serial_number])

    # Initialize PMS7003 sensor
    # Configure UART1 with TX=25, RX=26
    pms = Pms7003(uart=2, tx=Pin(25), rx=Pin(26))

    scd.start_periodic_measurement()
    print("Waiting for data...")

    while True:
        if pms.data_ready:
            print(json.dumps({
                "sensor": "pms7003",
                "pm1_0": pms.pm1_0_atmospheric,
                "pm2_5": pms.pm2_5_atmospheric,
                "pm10_0": pms.pm10_0_atmospheric
            }))

        if scd.data_ready:
            print(json.dumps({
                "sensor": "scd41",
                "co2": scd.CO2,
                "temperature": scd.temperature,
                "humidity": scd.relative_humidity
            }))

        time.sleep(1)


if __name__ == "__main__":
    main()

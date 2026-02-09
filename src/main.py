import time
import json
from machine import I2C, Pin
import lib.scd4x as scd4x
import lib.sgp41 as sgp41
from lib.pms7003 import Pms7003


def main():
    i2c = I2C(0, sda=Pin(21), scl=Pin(22), freq=100000)

    # Initialize SCD4X sensor
    scd = scd4x.SCD4X(i2c)
    print("Serial number:", [hex(i) for i in scd.serial_number])

    # Initialize PMS7003 sensor
    # Configure UART1 with TX=25, RX=26
    pms = Pms7003(uart=2, tx=Pin(25), rx=Pin(26))

    # Initialize SGP41
    try:
        sgp = sgp41.SGP41(i2c)
        # Execute conditioning (optional, but good for first run)
        # sgp.execute_conditioning()
        print("SGP41 initialized")
    except Exception as e:
        print("Failed to initialize SGP41:", e)
        sgp = None

    scd.start_periodic_measurement()
    print("Waiting for data...")

    # Last known environment values for SGP41 compensation
    last_rh = None
    last_temp = None

    while True:
        if pms.data_ready:
            try:
                print(
                    json.dumps(
                        {
                            "sensor": "pms7003",
                            "pm1_0": pms.pm1_0_atmospheric,
                            "pm2_5": pms.pm2_5_atmospheric,
                            "pm10_0": pms.pm10_0_atmospheric,
                        }
                    )
                )
            except Exception as e:
                print("PMS7003 error:", e)

        if scd.data_ready:
            try:
                last_temp = scd.temperature
                last_rh = scd.relative_humidity
                print(
                    json.dumps(
                        {
                            "sensor": "scd41",
                            "co2": scd.CO2,
                            "temperature": last_temp,
                            "humidity": last_rh,
                        }
                    )
                )
            except Exception as e:
                print("SCD41 error:", e)

        if sgp:
            try:
                sraw_voc, sraw_nox = sgp.measure_raw(last_rh, last_temp)
                voc_index = sgp._voc_algo.process(sraw_voc)
                nox_index = sgp._nox_algo.process(sraw_nox)
                print(
                    json.dumps(
                        {
                            "sensor": "sgp41",
                            "voc_raw": sraw_voc,
                            "nox_raw": sraw_nox,
                            "voc_index": voc_index,
                            "nox_index": nox_index,
                        }
                    )
                )
            except Exception as e:
                print("SGP41 error:", e)

        time.sleep(1)


if __name__ == "__main__":
    main()
